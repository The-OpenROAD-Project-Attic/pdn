/* Authors: Lutong Wang and Bangqi Xu */
/*
 * Copyright (c) 2019, The Regents of the University of California
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE REGENTS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "global.h"
#include "io/frPinPrep.h"
#include "db/obj/frNet.h"
#include <iostream>
#include "ta/FlexTA.h"
#include <boost/polygon/polygon.hpp>
#include "io/frShapeUtil.h"
#include "FlexAccessPattern.h"
#include <iterator>
#include <chrono>
#include "drc/frDRC.h"
#include "io/frAPG.h"

using namespace boost::polygon::operators;
using namespace std;
using namespace fr;
using namespace std::chrono;

void FlexPinPrep::reportPinLoc() {
  std::cout << "Printing Macro Pin Bbox(s)...\n";
  for (auto &inst: design->getTopBlock()->getInsts()) {
    if (inst->getRefBlock()->getMacroClass() != MacroClassEnum::BLOCK) {
      continue;
    }
    
    for (auto &uinstTerm: inst->getInstTerms()) {
      frInstTerm *instTerm = uinstTerm.get();
      auto instTermType = instTerm->getTerm()->getType();
      //if (instTermType != frTermEnum::frcNormalTerm && instTermType != frTermEnum::frcClockTerm) {
      //  continue;
      //}
      std::cout << "instName: " << inst->getName() << "; ";
      std::cout << inst->getRefBlock()->getName() << "/" << instTerm->getTerm()->getName() << ":\n";
      std::cout << "Locations(s):\n";
      frTransform xform;
      frBox mbox;
      inst->getTransform(xform);
      inst->getRefBlock()->getBoundaryBBox(mbox);
      frPoint size(mbox.right(), mbox.top());
      xform.updateXform(size);

      for (auto &pin: instTerm->getTerm()->getPins()) {
        auto pinPtr = pin.get();
        map<frLayerNum, PolygonSet> layerNum2PS;
        mergePinShapes(pinPtr, xform, layerNum2PS);
        for (auto layerIt = layerNum2PS.begin(); layerIt != layerNum2PS.end(); ++layerIt) {
          auto layerNum = layerIt->first;
          auto &layerPS = layerIt->second;
          std::vector<Rectangle> maxRects;
          get_max_rectangles(maxRects, layerPS);
          for (auto &rect: maxRects) {
            std::cout << "  (" << xl(rect) << ", " << yl(rect) << ") - ("
                      << xh(rect) << ", " << yh(rect) << ") " 
                      << design->getTech()->getLayer(layerNum)->getName() << "\n";
          }
        }
      }
    }
  }
}

void FlexPinPrep::init() {
  for (auto layerNum = design->getTech()->getBottomLayerNum(); layerNum <= design->getTech()->getTopLayerNum(); ++layerNum) {
    for (auto &viaDef: design->getTech()->getLayer(layerNum)->getViaDefs()) {
      int cutNum = int(viaDef->getCutFigs().size());
      viaRawPriorityTuple priority;
      getViaRawPriority(viaDef, priority);
      layerNum2ViaDefs[layerNum][cutNum][priority] = viaDef;
    }
  }
}

void FlexPinPrep::getViaRawPriority(frViaDef* viaDef, viaRawPriorityTuple &priority) {
  bool isNotDefaultVia = !(viaDef->getDefault());
  bool isNotUpperAlign = false;
  // bool isNotLowerAlign = false;
  PolygonSet viaLayerPS1;

  for (auto &fig: viaDef->getLayer1Figs()) {
    frBox bbox;
    fig->getBBox(bbox);
    Rectangle bboxRect(bbox.left(), bbox.bottom(), bbox.right(), bbox.top());
    viaLayerPS1 += bboxRect;
  }
  // Rectangle layer1Rect;
  // extents(layer1Rect, viaLayerPS1);
  // bool isLayer1Horz = (xh(layer1Rect) - xl(layer1Rect)) > (yh(layer1Rect) - yl(layer1Rect));
  // isNotLowerAlign = (isLayer1Horz != (getTech()->getLayer(viaDef->getLayer1Num())->getDir() == frcHorzPrefRoutingDir));

  PolygonSet viaLayerPS2;
  for (auto &fig: viaDef->getLayer2Figs()) {
    frBox bbox;
    fig->getBBox(bbox);
    Rectangle bboxRect(bbox.left(), bbox.bottom(), bbox.right(), bbox.top());
    viaLayerPS2 += bboxRect;
  }
  Rectangle layer2Rect;
  extents(layer2Rect, viaLayerPS2);
  bool isLayer2Horz = (xh(layer2Rect) - xl(layer2Rect)) > (yh(layer2Rect) - yl(layer2Rect));
  isNotUpperAlign = (isLayer2Horz != (getTech()->getLayer(viaDef->getLayer2Num())->getDir() == frcHorzPrefRoutingDir));

  frCoord layer1Area = area(viaLayerPS1);
  frCoord layer2Area = area(viaLayerPS2);

  priority = std::make_tuple(isNotDefaultVia, isNotUpperAlign, layer2Area, layer1Area);
}

int FlexPinPrep::main() {
  if (VERBOSE > 0) {
    cout <<endl <<"start pin prep ..." <<endl;
  }
  // bool enableOutput = true;
  high_resolution_clock::time_point t1 = high_resolution_clock::now();
  pinPrep();  // Maze rely on this
  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
  if (VERBOSE > 0) {
    cout <<endl <<"PinPrep runtime taken (hrt): " << time_span.count()    <<endl;
    cout <<"accuSpan0: " << accuSpan0.count() << endl;
    cout <<"accuSpan1: " << accuSpan1.count() << endl;
    cout <<"accuSpan2: " << accuSpan2.count() << endl;
    cout <<"drcSpan0: " << drcSpan0.count() << endl;
    cout <<"drcSpan1: " << drcSpan1.count() << endl;
    cout <<"drcSpan2: " << drcSpan2.count() << endl;
  }
  return 0;
}


void FlexPinPrep::pinPrep() {
  // int cnt = 0;
  int instTermCnt = 0;
  int ioTermCnt = 0;
  // pin prep for inst terms
  for (auto blockIt = trackOffsetMap.begin(); blockIt != trackOffsetMap.end(); ++blockIt) {
    auto &orientMap = blockIt->second;
    for (auto orientIt = orientMap.begin(); orientIt != orientMap.end(); ++orientIt) {
      // auto orient = orientIt->first;
      auto &offsets2Inst = orientIt->second;
      for (auto offsetsIt = offsets2Inst.begin(); offsetsIt != offsets2Inst.end(); ++offsetsIt) {
        // auto &offsets = offsetsIt->first;
        auto instPtr = *(offsetsIt->second.begin());
        for (auto &uinstTerm: instPtr->getInstTerms()) {
          frInstTerm *instTerm = uinstTerm.get();
          auto instTermType = instTerm->getTerm()->getType();
          if (instTermType != frTermEnum::frcNormalTerm && instTermType != frTermEnum::frcClockTerm) {
            continue;
          }
          ++instTermCnt;
          instTermPinPrep(instTerm, offsetsIt->second);
          if (instTermCnt % 100 == 0) {
            cout <<" complete " <<instTermCnt <<" instTerms" <<endl;
          }
        }
        // update ap priority
        // updateAPCost(instPtr);

        // generate APC
        if (instPtr->getRefBlock()->getMacroClass() == MacroClassEnum::CORE ||
            instPtr->getRefBlock()->getMacroClass() == MacroClassEnum::CORE_TIEHIGH ||
            instPtr->getRefBlock()->getMacroClass() == MacroClassEnum::CORE_TIELOW ||
            instPtr->getRefBlock()->getMacroClass() == MacroClassEnum::CORE_ANTENNACELL) {
          std::cout << instPtr->getName() << "(" << instPtr->getRefBlock()->getName() << "):\n";
          APGWorker apgWorker(getDesign(), instPtr);
          apgWorker.init();
          apgWorker.setup();
          apgWorker.main();
          apgWorker.end();
        }

      }
    }
  }
  // pin prep for terms
  for (auto &term: design->getTopBlock()->getTerms()) {
    ++ioTermCnt;
    termPinPrep(term.get());
    if (ioTermCnt % 100 == 0) {
      cout <<" complete " <<ioTermCnt <<" ioTerms" <<endl;
    }
  }

  std::cout << "instTermCnt = " << instTermCnt << "\n";
  std::cout << "ioTermCnt = " << ioTermCnt << "\n";
  std::cout << "APCnt = " << APCnt << "\n";
}

void FlexPinPrep::updateAPCost(frInst *inst) {
  std::map<frTerm*, std::vector<FlexAccessPattern*> > term2APs;
  std::map<frLayerNum, bgi::rtree<std::pair<point_t, FlexAccessPattern*>, bgi::quadratic<16> > > layer2APRTree;
  for (auto &uinstTerm: inst->getInstTerms()) {
    frInstTerm *instTerm = uinstTerm.get();
    auto instTermType = instTerm->getTerm()->getType();
    if (instTermType != frTermEnum::frcNormalTerm && instTermType != frTermEnum::frcClockTerm) {
      continue;
    }
    // populate map and rtree
    for (auto &upin: instTerm->getTerm()->getPins()) {
      frPin* pin = upin.get();
      for (auto &uap: pin->getAccessPatterns()) {
        FlexAccessPattern* ap = uap.get();
        if (ap->hasValidAccess(frDirEnum::U)) {
          term2APs[instTerm->getTerm()].push_back(ap);
          frPoint bp, ep;
          ap->getPoints(bp, ep);
          layer2APRTree[ap->getBeginLayerNum() + 2].insert(std::make_pair(point_t(bp.x(), bp.y()), ap));
        }
      }
    }
  }
  // setNearest AP distance (the further the better)
  for (auto layerIt = layer2APRTree.begin(); layerIt != layer2APRTree.end(); ++layerIt) {
    //frLayerNum layerNum = layerIt->first;
    auto &apRTree = layerIt->second;
    for (auto apIt = apRTree.begin(); apIt != apRTree.end(); ++apIt) {
      auto loc = apIt->first;
      auto ap = apIt->second;
      // get nearest aps to query queue
      std::vector<std::pair<point_t, FlexAccessPattern*> > qq;
      apRTree.query(bgi::nearest(loc, term2APs[ap->getPin()->getTerm()].size() + 1), std::back_inserter(qq));
      frCoord maxDist = 0;
      for (auto &apPair: qq) {
        // auto deltaX = loc.x() - apPair.first.x();
        // auto deltaY = loc.y() - apPair.first.y();
        // auto tmpDist = deltaX * deltaX + deltaY * deltaY;
        auto tmpDist = bg::distance(loc, apPair.first);
        if (tmpDist > maxDist) {
          maxDist = tmpDist;
        }
      }
      ap->setNearestAPDist(maxDist);
    }
  }

}

void FlexPinPrep::instTermPinPrep(frInstTerm *instTerm, const std::set<frInst*, frBlockObjectComp> &instsIn) {
  bool enableOutput = false;
  int instTermAPCnt = 0;
  //bool enableOutput = true;
  // bool isPhysical = true;
  if (enableOutput) {
    cout << "instTermPinPrep\n";
  }
  frInst* inst = instTerm->getInst();
  bool isMacro = inst->getRefBlock()->getMacroClass() == MacroClassEnum::BLOCK;
  frBox mbox;
  frTransform xform, termXForm, revertCellXForm;

  std::set<frInst*> insts;
  insts.insert(instsIn.begin(), instsIn.end());



  // debug
  // if (instTerm->getInst()->getRefBlock()->getName() != "OR4X2") {
  //   return;
  // } 

  inst->getTransform(xform);
  inst->getRefBlock()->getBoundaryBBox(mbox);
  frPoint size(mbox.right(), mbox.top());
  termXForm.set(xform.orient());
  termXForm.updateXform(size);
  // xform.revert(revertCellXForm);
  revertCellXForm.set(-xform.xOffset(), -xform.yOffset());
  // std::cout << xform.xOffset() << " " << xform.yOffset() << "\n";
  // std::cout << revertCellXForm.xOffset() << " " << revertCellXForm.yOffset() << "\n";
  revertCellXForm.set(frcR0);
  xform.updateXform(size);
  // xform.revert(revertXForm);
  

  // 
  double dbu = getTech()->getDBUPerUU();
  frBox instBBox;
  inst->getBBox(instBBox);
  box_t instQueryBox(point_t(instBBox.left(), instBBox.bottom()), point_t(instBBox.right(), instBBox.top()));
  /*
  std::vector<frBlockObject*> instObjs;
  std::set<frBlockObject*> instObjSet;
  // std::map<frLayerNum, PolygonSet> layer2OBS; 
  // std::cout << "inst bbox = (" << instBBox.left() / dbu << ", " << instBBox.bottom() / dbu
  //           << ") - (" << instBBox.right() / dbu << ", " << instBBox.top() / dbu << ")\n";
  for (auto layerNum = design->getTech()->getBottomLayerNum(); layerNum <= design->getTech()->getTopLayerNum(); ++layerNum) {
    auto regionQuery = design->getRegionQuery();
    std::vector<rq_rptr_value_t<frBlockObject> > queryResult;
    regionQuery->query(instQueryBox, layerNum, queryResult);
    for (auto &objPair: queryResult) {
      instObjSet.insert(objPair.second);
    }
  }
  for (auto instObj:instObjSet) {
    if ((instObj->typeId() == frcInstTerm && (static_cast<frInstTerm*>(instObj))->getInst() == inst) || 
        (instObj->typeId() == frcInstBlockage)) {
      // if (instObj->typeId() == frcInstBlockage) {
      //   std::cout << "frcInstBlockage found\n";
      // }
      instObjs.push_back(instObj);
    }
    // // attempt to speedup macro processing
    // if (instObj->typeId() == frcInstBlockage) {
    //   frBlockage* blockage = (static_cast<frInstBlockage*>(instObj))->getBlockage();
    //   if (blockage->typeId() != frcLayerBlockage) {
    //     std::cout << "Error: unexpected blockage type in initNetRects_blockages\n";
    //   } else {
    //     frTransform xform;
    //     inst->getUpdatedXform(xform);
    //     frLayerBlockage* layerBlockage = static_cast<frLayerBlockage*>(blockage);
    //     frLayerNum layerNum = layerBlockage->getLayerNum();
    //     auto blockageFrPoints = layerBlockage->getPoints();
    //     std::vector<Point> blockagePoints;
    //     for (auto pt: blockageFrPoints) {
    //       frPoint tmpPt = pt;
    //       tmpPt.transform(xform);
    //       blockagePoints.push_back(Point(tmpPt.x(), tmpPt.y()));
    //     }
    //     Polygon blockagePoly;
    //     set_points(blockagePoly, blockagePoints.begin(), blockagePoints.end());
    //     layer2OBS[layerNum] += blockagePoly;
    //   }
    // }
  }
  // all cell objects are read in
  */
  if (enableOutput) {
  //if (true) {
    std::cout << "cell / cell master / orient / instTerm: " 
              << instTerm->getInst()->getName() << " / "
              << instTerm->getInst()->getRefBlock()->getName() << " / " 
              << xform.orient() << " / " 
              << instTerm->getTerm()->getName() << "\n" << std::flush;
  }
  // std::cout << "here-1\n";
  bool hasValidAP = false;
  for (auto &pin: instTerm->getTerm()->getPins()) {
    // map<frPoint, int> endPoint2Cost;
    // map<frPoint, pair<frOrient, FlexAccessPattern> > endPiont2AP;
    auto pinPtr = pin.get();
    map<frLayerNum, PolygonSet> layerNum2PS;
    // map<frLayerNum, Rectangle> layerNum2BBox;
    // int numLayers = int(design->getTech()->getLayers().size());
    // frLayerNum minPinLayerNum = numLayers, maxPinLayerNum = 0;
    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    mergePinShapes(pinPtr, xform, layerNum2PS);
    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    accuSpan0 += duration_cast<duration<double>>(t2 - t1);
    
    std::vector<frBlockObject*> pinObjs;
    std::set<frBlockObject*> pinObjSet;
    std::vector<Rectangle> maxRects;
    Rectangle pinBBoxRect, pinBloatBBoxRect;
    PolygonSet pinPS;
    frCoord maxPitch = 0;
    for (auto layerIt = layerNum2PS.begin(); layerIt != layerNum2PS.end(); ++layerIt) {
      pinPS += layerIt->second;
      auto currLayerPitch = getTech()->getLayer(layerIt->first)->getPitch();
      if (currLayerPitch > maxPitch) {
        maxPitch = currLayerPitch;
      }
    }
    // extents(pinBBoxRect, pinPS);
    // pinBloatBBoxRect = bloat(pinBBoxRect, maxPitch);
    // box_t pinBloatBBox(point_t(xl(pinBloatBBoxRect), yl(pinBloatBBoxRect)), point_t(xh(pinBloatBBoxRect), yh(pinBloatBBoxRect)));
    // for (auto layerNum = design->getTech()->getBottomLayerNum(); layerNum <= design->getTech()->getTopLayerNum(); ++layerNum) {
    //   auto regionQuery = design->getRegionQuery();
    //   std::vector<rq_rptr_value_t<frBlockObject> > queryResult;
    //   regionQuery->query(pinBloatBBox, layerNum, queryResult);
    //   for (auto &objPair: queryResult) {
    //     pinObjSet.insert(objPair.second);
    //   }
    // }

    get_max_rectangles(maxRects, pinPS);
    for (auto maxRect: maxRects) {
      pinBloatBBoxRect = bloat(maxRect, maxPitch);
      box_t pinBloatBBox(point_t(xl(pinBloatBBoxRect), yl(pinBloatBBoxRect)), point_t(xh(pinBloatBBoxRect), yh(pinBloatBBoxRect)));
      for (auto layerNum = design->getTech()->getBottomLayerNum(); layerNum <= design->getTech()->getTopLayerNum(); ++layerNum) {
        auto regionQuery = design->getRegionQuery();
        std::vector<rq_rptr_value_t<frBlockObject> > queryResult;
        regionQuery->query(pinBloatBBox, layerNum, queryResult);
        for (auto &objPair: queryResult) {
          pinObjSet.insert(objPair.second);
        }
      }
    }
    for (auto pinObj:pinObjSet) {
      if ((pinObj->typeId() == frcInstTerm && (static_cast<frInstTerm*>(pinObj))->getInst() == inst) || 
          (pinObj->typeId() == frcInstBlockage)) {
        pinObjs.push_back(pinObj);
      }
    }

    // if (enableOutput) {
    //   for (auto it = layerNum2PS.begin(); it != layerNum2PS.end(); ++it) {
    //     auto layerNum = it->first;
    //     auto &polySet = it->second;
    //     std::cout << " layerNum = " << layerNum << "\n";
    //     for (auto &poly: polySet) {
    //       std::cout << "  poly: ";
    //       for (auto ptIt = boost::polygon::begin_points(poly); ptIt != boost::polygon::end_points(poly); ++ptIt) {
    //         frPoint tmpPt((*ptIt).x(), (*ptIt).y());
    //         std::cout << "(" << tmpPt.x() / 2000.0 << "," << tmpPt.y() / 2000.0 << ") -- ";
    //       }
    //       std::cout << "\n";
    //     }
    //   }
    // }
    // std::cout << "here0\n";
    // getPinLayerBBox(layerNum2PS, layerNum2BBox);
    for (auto layerIt = layerNum2PS.begin(); layerIt != layerNum2PS.end(); ++layerIt) {
      bool hasPinLayerAP = false;
      map<frCoord, map<frLayerNum, frTrackPattern*> > xLoc2TrackPatterns; // track xCoord
      map<frCoord, map<frLayerNum, frTrackPattern*> > yLoc2TrackPatterns; // track yCoord
      frLayerNum currLayerNum = layerIt->first;
      PolygonSet layerPS = layerNum2PS[currLayerNum];
      map<frPoint, int> startPoints; // startPoint with cost, lower is more preferrable
      // set<pair<frLayerNum, frPoint> > endPoints;
      // Rectangle layerBBox = layerNum2BBox[currLayerNum];
      high_resolution_clock::time_point t1 = high_resolution_clock::now();
      getPinLayerBBoxTrackPatterns(currLayerNum, layerPS, xLoc2TrackPatterns, yLoc2TrackPatterns);
      getPinLayerAPStartPoints(currLayerNum, layerPS, xLoc2TrackPatterns, yLoc2TrackPatterns, startPoints);
      high_resolution_clock::time_point t2 = high_resolution_clock::now();
      // getPinLayerAPEndPoints(currLayerNum, layerPS, xLoc2TrackPatterns, yLoc2TrackPatterns, endPoints);
      // if (enableOutput) {
      //   double dbu = getTech()->getDBUPerUU();
      //   for (auto &startPoint: startPoints) {
      //     if (!isPhysical) {
      //       std::cout << "  startPoint: " << "(" << startPoint.first.x() << ", " << startPoint.first.y() << ", " << currLayerNum << ")\n";
      //     } else {
      //       std::cout << "  startPoint: " << "(" << startPoint.first.x() / dbu << ", " << startPoint.first.y() / dbu << ", " << currLayerNum << ")\n";
      //     }
      //   }
      //   for (auto &endPoint: endPoints) {
      //     if (!isPhysical) {
      //       std::cout << "  endPoint: (" << endPoint.second.x() << ", " << endPoint.second.y() << ", " << endPoint.first << ")\n";
      //     } else {
      //       std::cout << "  endPoint: (" << endPoint.second.x() / dbu << ", " << endPoint.second.y() / dbu << ", " << endPoint.first << ")\n";
      //     }
      //   }
      // }

      // get access pattern based on start point only
      for (auto &startPoint: startPoints) {
        auto &startPt = startPoint.first;
        frPoint apStartPt = startPt;
        apStartPt.transform(revertCellXForm);
        auto &startLayerNum = currLayerNum;
        FlexAccessPattern ap;
        ap.setPoints(apStartPt, apStartPt);
        ap.setBeginLayerNum(startLayerNum);
        ap.addToPin(pin.get());
        if (inst->getRefBlock()->getMacroClass() == MacroClassEnum::BLOCK) {
          ap.setConflict(false);
        }
        //ap.setEndLayerNum(startLayerNum);
        //ap.setCost(1);
        


        frPoint endPt;
        // check planar access
        if (enableOutput) {
          std::cout << "\nstartPt (" << startPt.x() / dbu << ", " << startPt.y() / dbu << ")\n";
        }
        // E
        bool skipE = false;
        getPlanarEP(startLayerNum, startPt, frDirEnum::E, endPt);
        if (enableOutput) {
          std::cout << "  E endPt at (" << endPt.x() / dbu << ", " << endPt.y() / dbu << ")\n";
        }
        // for (auto &obsPoly: layer2OBS[startLayerNum]) {
        //   if (contains(obsPoly, Point(endPt.x(), endPt.y()))) {
        //     skipE = true;
        //     break;
        //   }
        // }
        if (!skipE) {
          if (isMacro || isValidPlanarAccess(pinPtr, instTerm, pinObjs, startLayerNum, startPt, endPt, layerPS)) {
            ap.setValidAccess(frDirEnum::E, true);
            ++APCnt;
            ++instTermAPCnt;
          } else {
            ap.setValidAccess(frDirEnum::E, false);
          }
        }
        // S
        bool skipS = false;
        getPlanarEP(startLayerNum, startPt, frDirEnum::S, endPt);
        if (enableOutput) {
          std::cout << "  S endPt at (" << endPt.x() / dbu << ", " << endPt.y() / dbu << ")\n";
        }
        // for (auto &obsPoly: layer2OBS[startLayerNum]) {
        //   if (contains(obsPoly, Point(endPt.x(), endPt.y()))) {
        //     skipS = true;
        //     break;
        //   }
        // }
        if (!skipS) {
          if (isMacro || isValidPlanarAccess(pinPtr, instTerm, pinObjs, startLayerNum, startPt, endPt, layerPS)) {
            ap.setValidAccess(frDirEnum::S, true);
            ++APCnt;
            ++instTermAPCnt;
          } else {
            ap.setValidAccess(frDirEnum::S, false);
          }
        }
        // W
        bool skipW = false;
        getPlanarEP(startLayerNum, startPt, frDirEnum::W, endPt);
        if (enableOutput) {
          std::cout << "  W endPt at (" << endPt.x() / dbu << ", " << endPt.y() / dbu << ")\n";
        }
        // for (auto &obsPoly: layer2OBS[startLayerNum]) {
        //   if (contains(obsPoly, Point(endPt.x(), endPt.y()))) {
        //     skipW = true;
        //     break;
        //   }
        // }
        if (!skipW) {
          if (isMacro || isValidPlanarAccess(pinPtr, instTerm, pinObjs, startLayerNum, startPt, endPt, layerPS)) {
            ap.setValidAccess(frDirEnum::W, true);
            ++APCnt;
            ++instTermAPCnt;
          } else {
            ap.setValidAccess(frDirEnum::W, false);
          }
        }
        // N
        bool skipN = false;
        getPlanarEP(startLayerNum, startPt, frDirEnum::N, endPt);
        if (enableOutput) {
          std::cout << "  N endPt at (" << endPt.x() / dbu << ", " << endPt.y() / dbu << ")\n";
        }
        // for (auto &obsPoly: layer2OBS[startLayerNum]) {
        //   if (contains(obsPoly, Point(endPt.x(), endPt.y()))) {
        //     skipN = true;
        //     break;
        //   }
        // }
        if (!skipN) {
          if (isMacro || isValidPlanarAccess(pinPtr, instTerm, pinObjs, startLayerNum, startPt, endPt, layerPS)) {
            ap.setValidAccess(frDirEnum::N, true);
            ++APCnt;
            ++instTermAPCnt;
          } else {
            ap.setValidAccess(frDirEnum::N, false);
          }
        }


        // U
        if (enableOutput) {
          std::cout << "  U\n";
        }
        
        if (inst->getRefBlock()->getMacroClass() == MacroClassEnum::CORE ||
            inst->getRefBlock()->getMacroClass() == MacroClassEnum::CORE_TIEHIGH ||
            inst->getRefBlock()->getMacroClass() == MacroClassEnum::CORE_TIELOW ||
            inst->getRefBlock()->getMacroClass() == MacroClassEnum::CORE_ANTENNACELL) {
          // get via accesses
          std::map<viaPriorityTuple, frViaDef*> priority2ViaDef;
          auto upperViaDef = getTech()->getLayer(startLayerNum + 1)->getDefaultViaDef();
          viaPriorityTuple priority;
          if (isValidViaAccess(pinPtr, instTerm, pinObjs, startLayerNum, startPt, upperViaDef, priority)) {
            ap.setValidAccess(frDirEnum::U, true);
            ++APCnt;
            ++instTermAPCnt;
            // ap.addAccessViaDef(frDirEnum::U, upperViaDef);
            priority2ViaDef[priority] = upperViaDef;
          }

          for (auto layerIt = layerNum2ViaDefs.begin(); layerIt != layerNum2ViaDefs.end(); ++layerIt) {
            auto currLayerNum = layerIt->first;
            if (currLayerNum != startLayerNum + 1) {
              continue;
            }
            auto &numCut2ViaDefs = layerIt->second;
            int numCut = -1;
            for (auto numCutIt = numCut2ViaDefs.begin(); numCutIt != numCut2ViaDefs.end(); ++numCutIt) {
              if (numCut != -1) {
                break;
              }
              numCut = numCutIt->first;
              auto &viaRawPriority2ViaDefs = numCutIt->second;
              int attemptCnt = 0;
              if (ap.hasValidAccess(frDirEnum::U)) {
                ++attemptCnt;
              }
              for (auto viaDefIt = viaRawPriority2ViaDefs.begin(); viaDefIt != viaRawPriority2ViaDefs.end(); ++viaDefIt) {
                if (attemptCnt >= maxViaAttemptLimit) {
                  break;
                }
                auto viaDef = viaDefIt->second;
                if (!viaDef->getDefault() || viaDef == upperViaDef) {
                  continue;
                }
                if (isValidViaAccess(pinPtr, instTerm, pinObjs, startLayerNum, startPt, viaDef, priority)) {
                  ap.setValidAccess(frDirEnum::U, true);
                  ++APCnt;
                  ++instTermAPCnt;
                  ++attemptCnt;
                  priority2ViaDef[priority] = viaDef;
                }
              }
            }
          }
          for (auto mapIt = priority2ViaDef.begin(); mapIt != priority2ViaDef.end(); ++mapIt) {
            ap.addAccessViaDef(frDirEnum::U, mapIt->second);
          }
        }

        if (ap.hasValidAccess()) {
          hasValidAP = true;
          hasPinLayerAP = true;
          ap.addInsts(insts);
          pinPtr->addAccessPattern(xform.orient(), ap);
        }

      }
      high_resolution_clock::time_point t3 = high_resolution_clock::now();
      accuSpan1 += duration_cast<duration<double>>(t2 - t1);
      accuSpan2 += duration_cast<duration<double>>(t3 - t2);
      if (!hasPinLayerAP) {
        // if (enableOutput) {
        if (true) {
          std::cout << "Warning: " << instTerm->getInst()->getName() << "/" << instTerm->getTerm()->getName() 
                    << " has no valid AP on layer " << currLayerNum << ". Checking shape-center along pref-dir option\n";
        }
        startPoints.clear();
        high_resolution_clock::time_point t1 = high_resolution_clock::now();
        getPinLayerAPStartPoints_shapeCenter(currLayerNum, layerPS, xLoc2TrackPatterns, yLoc2TrackPatterns, startPoints);
        high_resolution_clock::time_point t2 = high_resolution_clock::now();
        accuSpan1 += duration_cast<duration<double>>(t2 - t1);
        
        for (auto &startPoint: startPoints) {
          auto &startPt = startPoint.first;
          frPoint apStartPt = startPt;
          apStartPt.transform(revertCellXForm);
          auto &startLayerNum = currLayerNum;
          FlexAccessPattern ap;
          ap.setPoints(apStartPt, apStartPt);
          ap.setBeginLayerNum(startLayerNum);
          ap.addToPin(pin.get());
          //ap.setEndLayerNum(startLayerNum);
          //ap.setCost(1);
          


          frPoint endPt;
          // check planar access
          if (enableOutput) {
            std::cout << "\nstartPt (" << startPt.x() / dbu << ", " << startPt.y() / dbu << ")\n";
          }
          // E
          bool skipE = false;
          getPlanarEP(startLayerNum, startPt, frDirEnum::E, endPt);
          if (enableOutput) {
            std::cout << "  E endPt at (" << endPt.x() / dbu << ", " << endPt.y() / dbu << ")\n";
          }
          // for (auto &obsPoly: layer2OBS[startLayerNum]) {
          //   if (contains(obsPoly, Point(endPt.x(), endPt.y()))) {
          //     skipE = true;
          //     break;
          //   }
          // }
          if (!skipE) {
            if (isMacro || isValidPlanarAccess(pinPtr, instTerm, pinObjs, startLayerNum, startPt, endPt, layerPS)) {
              ap.setValidAccess(frDirEnum::E, true);
              ++APCnt;
              ++instTermAPCnt;
            } else {
              ap.setValidAccess(frDirEnum::E, false);
            }
          }
          // S
          bool skipS = false;
          getPlanarEP(startLayerNum, startPt, frDirEnum::S, endPt);
          if (enableOutput) {
            std::cout << "  S endPt at (" << endPt.x() / dbu << ", " << endPt.y() / dbu << ")\n";
          }
          // for (auto &obsPoly: layer2OBS[startLayerNum]) {
          //   if (contains(obsPoly, Point(endPt.x(), endPt.y()))) {
          //     skipS = true;
          //     break;
          //   }
          // }
          if (!skipS) {
            if (isMacro || isValidPlanarAccess(pinPtr, instTerm, pinObjs, startLayerNum, startPt, endPt, layerPS)) {
              ap.setValidAccess(frDirEnum::S, true);
              ++APCnt;
              ++instTermAPCnt;
            } else {
              ap.setValidAccess(frDirEnum::S, false);
            }
          }
          // W
          bool skipW = false;
          getPlanarEP(startLayerNum, startPt, frDirEnum::W, endPt);
          if (enableOutput) {
            std::cout << "  W endPt at (" << endPt.x() / dbu << ", " << endPt.y() / dbu << ")\n";
          }
          // for (auto &obsPoly: layer2OBS[startLayerNum]) {
          //   if (contains(obsPoly, Point(endPt.x(), endPt.y()))) {
          //     skipW = true;
          //     break;
          //   }
          // }
          if (!skipW) {
            if (isMacro || isValidPlanarAccess(pinPtr, instTerm, pinObjs, startLayerNum, startPt, endPt, layerPS)) {
              ap.setValidAccess(frDirEnum::W, true);
              ++APCnt;
              ++instTermAPCnt;
            } else {
              ap.setValidAccess(frDirEnum::W, false);
            }
          }
          // N
          bool skipN = false;
          getPlanarEP(startLayerNum, startPt, frDirEnum::N, endPt);
          if (enableOutput) {
            std::cout << "  N endPt at (" << endPt.x() / dbu << ", " << endPt.y() / dbu << ")\n";
          }
          // for (auto &obsPoly: layer2OBS[startLayerNum]) {
          //   if (contains(obsPoly, Point(endPt.x(), endPt.y()))) {
          //     skipN = true;
          //     break;
          //   }
          // }
          if (!skipN) {
            if (isMacro || isValidPlanarAccess(pinPtr, instTerm, pinObjs, startLayerNum, startPt, endPt, layerPS)) {
              ap.setValidAccess(frDirEnum::N, true);
              ++APCnt;
              ++instTermAPCnt;
            } else {
              ap.setValidAccess(frDirEnum::N, false);
            }
          }
          // U
          if (enableOutput) {
            std::cout << "  U\n";
          }
          
          if (inst->getRefBlock()->getMacroClass() == MacroClassEnum::CORE ||
              inst->getRefBlock()->getMacroClass() == MacroClassEnum::CORE_TIEHIGH ||
              inst->getRefBlock()->getMacroClass() == MacroClassEnum::CORE_TIELOW ||
              inst->getRefBlock()->getMacroClass() == MacroClassEnum::CORE_ANTENNACELL) {
            // get via accesses
            std::map<viaPriorityTuple, frViaDef*> priority2ViaDef;
            auto upperViaDef = getTech()->getLayer(startLayerNum + 1)->getDefaultViaDef();
            viaPriorityTuple priority;
            if (isValidViaAccess(pinPtr, instTerm, pinObjs, startLayerNum, startPt, upperViaDef, priority)) {
              ap.setValidAccess(frDirEnum::U, true);
              ++APCnt;
              ++instTermAPCnt;
              // ap.addAccessViaDef(frDirEnum::U, upperViaDef);
              priority2ViaDef[priority] = upperViaDef;
            }

            for (auto layerIt = layerNum2ViaDefs.begin(); layerIt != layerNum2ViaDefs.end(); ++layerIt) {
              auto currLayerNum = layerIt->first;
              if (currLayerNum != startLayerNum + 1) {
                continue;
              }
              auto &numCut2ViaDefs = layerIt->second;
              int numCut = -1;
              for (auto numCutIt = numCut2ViaDefs.begin(); numCutIt != numCut2ViaDefs.end(); ++numCutIt) {
                if (numCut != -1) {
                  break;
                }
                numCut = numCutIt->first;
                auto &viaRawPriority2ViaDefs = numCutIt->second;
                int attemptCnt = 0;
                for (auto viaDefIt = viaRawPriority2ViaDefs.begin(); viaDefIt != viaRawPriority2ViaDefs.end(); ++viaDefIt) {
                  if (attemptCnt >= maxViaAttemptLimit) {
                    break;
                  }
                  auto viaDef = viaDefIt->second;
                  if (!viaDef->getDefault() || viaDef == upperViaDef) {
                    continue;
                  }
                  if (isValidViaAccess(pinPtr, instTerm, pinObjs, startLayerNum, startPt, viaDef, priority)) {
                    ap.setValidAccess(frDirEnum::U, true);
                    ++APCnt;
                    ++instTermAPCnt;
                    ++attemptCnt;
                    priority2ViaDef[priority] = viaDef;
                  }
                }
              }
            }

            for (auto mapIt = priority2ViaDef.begin(); mapIt != priority2ViaDef.end(); ++mapIt) {
              ap.addAccessViaDef(frDirEnum::U, mapIt->second);
            }
          }

          if (ap.hasValidAccess()) {
            hasValidAP = true;
            hasPinLayerAP = true;
            ap.addInsts(insts);
            pinPtr->addAccessPattern(xform.orient(), ap);
          }

        }
        high_resolution_clock::time_point t3 = high_resolution_clock::now();
        accuSpan2 += duration_cast<duration<double>>(t3 - t2);
      }

      if (!hasPinLayerAP) {
        std::cout << "Warning: " << instTerm->getInst()->getName() << "/" << instTerm->getTerm()->getName() 
                    << " has no valid AP on layer " << currLayerNum << ". Need shape-center option\n";
      }

    }

  }
  if (!hasValidAP) {
    std::cout << instTerm->getInst()->getName() << "/" << instTerm->getTerm()->getName() << " has no valid AP\n";
  }
}

void FlexPinPrep::termPinPrep(frTerm *term) {
  bool enableOutput = false;
  // bool enableOutput = true;
  // bool isPhysical = true;
  if (enableOutput) {
    cout << "termPinPrep\n";
  }

  double dbu = getTech()->getDBUPerUU();
  std::vector<frBlockObject*> instObjs;
  std::set<frBlockObject*> instObjSet;
  
  for (auto &pin: term->getPins()) {
    auto pinPtr = pin.get();
    map<frLayerNum, PolygonSet> layerNum2PS = pinPtr->getLayer2PolySet();

    for (auto layerIt = layerNum2PS.begin(); layerIt != layerNum2PS.end(); ++layerIt) {
      frLayerNum currLayerNum = layerIt->first;
      PolygonSet layerPS = layerIt->second;
      Rectangle layerBBox;
      extents(layerBBox, layerPS);
      box_t layerQueryBox(point_t(xl(layerBBox), yl(layerBBox)), point_t(xh(layerBBox), yh(layerBBox)));
      auto regionQuery = design->getRegionQuery();
      std::vector<rq_rptr_value_t<frBlockObject> > queryResult;
      regionQuery->query(layerQueryBox, currLayerNum, queryResult);
      for (auto &objPair: queryResult) {
        instObjSet.insert(objPair.second);
      }
    }
    for (auto instObj:instObjSet) {
      if (instObj->typeId() == frcTerm) {
        instObjs.push_back(instObj);
      }
    }
  }


  
  for (auto &pin: term->getPins()) {
    auto pinPtr = pin.get();
    map<frLayerNum, PolygonSet> layerNum2PS = pinPtr->getLayer2PolySet();
    for (auto layerIt = layerNum2PS.begin(); layerIt != layerNum2PS.end(); ++layerIt) {
      map<frCoord, map<frLayerNum, frTrackPattern*> > xLoc2TrackPatterns; // track xCoord
      map<frCoord, map<frLayerNum, frTrackPattern*> > yLoc2TrackPatterns; // track yCoord
      frLayerNum currLayerNum = layerIt->first;
      PolygonSet layerPS = layerNum2PS[currLayerNum];
      map<frPoint, int> startPoints; // startPoint with cost, lower is more preferrable
      // set<pair<frLayerNum, frPoint> > endPoints;
      // Rectangle layerBBox = layerNum2BBox[currLayerNum];
      getPinLayerBBoxTrackPatterns(currLayerNum, layerPS, xLoc2TrackPatterns, yLoc2TrackPatterns);
      getPinLayerAPStartPoints(currLayerNum, layerPS, xLoc2TrackPatterns, yLoc2TrackPatterns, startPoints);
      // get access pattern based on start/end point pair
      for (auto &startPoint: startPoints) {
        auto &startPt = startPoint.first;
        auto startLayerNum = currLayerNum;
        FlexAccessPattern ap;
        ap.setPoints(startPt, startPt);
        ap.setBeginLayerNum(startLayerNum);
        ap.setConflict(false);
        //ap.setEndLayerNum(startLayerNum);
        //ap.setCost(1);
        // ap.addInsts(insts);

        frPoint endPt;

        // check planar access
        if (enableOutput) {
          std::cout << "\nstartPt (" << startPt.x() / dbu << ", " << startPt.y() / dbu << ")\n";
        }
        // E
        
        getPlanarEP(startLayerNum, startPt, frDirEnum::E, endPt);
        if (enableOutput) {
          std::cout << "  E endPt at (" << endPt.x() / dbu << ", " << endPt.y() / dbu << ")\n";
        }
        if (isValidPlanarAccess(pinPtr, term, instObjs, startLayerNum, startPt, endPt, layerPS)) {
          ap.setValidAccess(frDirEnum::E, true);
          ++APCnt;
        } else {
          ap.setValidAccess(frDirEnum::E, false);
        }
        // S
        getPlanarEP(startLayerNum, startPt, frDirEnum::S, endPt);
        if (enableOutput) {
          std::cout << "  S endPt at (" << endPt.x() / dbu << ", " << endPt.y() / dbu << ")\n";
        }
        if (isValidPlanarAccess(pinPtr, term, instObjs, startLayerNum, startPt, endPt, layerPS)) {
          ap.setValidAccess(frDirEnum::S, true);
          ++APCnt;
        } else {
          ap.setValidAccess(frDirEnum::S, false);
        }
        // W
        getPlanarEP(startLayerNum, startPt, frDirEnum::W, endPt);
        if (enableOutput) {
          std::cout << "  W endPt at (" << endPt.x() / dbu << ", " << endPt.y() / dbu << ")\n";
        }
        if (isValidPlanarAccess(pinPtr, term, instObjs, startLayerNum, startPt, endPt, layerPS)) {
          ap.setValidAccess(frDirEnum::W, true);
          ++APCnt;
        } else {
          ap.setValidAccess(frDirEnum::W, false);
        }
        // N
        getPlanarEP(startLayerNum, startPt, frDirEnum::N, endPt);
        if (enableOutput) {
          std::cout << "  N endPt at (" << endPt.x() / dbu << ", " << endPt.y() / dbu << ")\n";
        }
        if (isValidPlanarAccess(pinPtr, term, instObjs, startLayerNum, startPt, endPt, layerPS)) {
          ap.setValidAccess(frDirEnum::N, true);
          ++APCnt;
        } else {
          ap.setValidAccess(frDirEnum::N, false);
        }
        if (ap.hasValidAccess()) {
          pinPtr->addAccessPattern(frOrient(frcR0), ap);
        }

      }
    }
  }
}

void FlexPinPrep::getPinLayerAPStartPoints(const frLayerNum &currLayerNum,
                                           const PolygonSet &currLayerPS,
                                           map<frCoord, map<frLayerNum, frTrackPattern*> > &xLoc2TrackPatterns,
                                           map<frCoord, map<frLayerNum, frTrackPattern*> > &yLoc2TrackPatterns,
                                           map<frPoint, int> &startPoints) {
  // bool enableOutput = true;
  map<frCoord, int> xLoc2Cost, yLoc2Cost;
  // initialize loc cost
  // xcost
  for (auto xLocIt = xLoc2TrackPatterns.begin(); xLocIt != xLoc2TrackPatterns.end(); ++xLocIt) {
    xLoc2Cost[xLocIt->first] = 1;
  }

  // ycost
  for (auto yLocIt = yLoc2TrackPatterns.begin(); yLocIt != yLoc2TrackPatterns.end(); ++yLocIt) {
    yLoc2Cost[yLocIt->first] = 1;
  }

  if (getTech()->getLayer(currLayerNum)->getDir() == frcVertPrefRoutingDir) {
    // if there is on grid start, then return
    for (auto xLocIt = xLoc2Cost.begin(); xLocIt != xLoc2Cost.end(); ++xLocIt) {
      for (auto yLocIt = yLoc2Cost.begin(); yLocIt != yLoc2Cost.end(); ++yLocIt) {
        Point tempPt(xLocIt->first, yLocIt->first);
        for (auto &poly: currLayerPS) {
          if (contains(poly, tempPt)) {
            startPoints[frPoint(tempPt.x(), tempPt.y())] = 2;
            break;
          }
        }
      }
    }

    if (!startPoints.empty()) {
      return;
    }


    // add half pitch if possible
    if (xLoc2Cost.size() > 1) {
      // x
      auto xLoc2CostTemp = xLoc2Cost;
      auto xLocNextIt = xLoc2CostTemp.begin();
      ++xLocNextIt;
      for (auto xLocIt = xLoc2CostTemp.begin(); xLocNextIt != xLoc2CostTemp.end(); ++xLocNextIt) {
        xLoc2Cost[(xLocIt->first + xLocNextIt->first) / 2] = xLocIt->second + xLocNextIt->second;
        ++xLocIt;
      }
    }

    // get lower center coord
    vector<Rectangle> rects;
    get_max_rectangles(rects, currLayerPS);
    for (auto &rect: rects) {
      Point centerPt;
      boost::polygon::center(centerPt, rect);
      if (xLoc2Cost.find(centerPt.x()) == xLoc2Cost.end()) {
        xLoc2Cost[centerPt.x()] = 2;
      }
    }

    for (auto xLocIt = xLoc2Cost.begin(); xLocIt != xLoc2Cost.end(); ++xLocIt) {
      for (auto yLocIt = yLoc2Cost.begin(); yLocIt != yLoc2Cost.end(); ++yLocIt) {
        Point tempPt(xLocIt->first, yLocIt->first);
        for (auto &poly: currLayerPS) {
          if (contains(poly, tempPt)) {
            startPoints[frPoint(tempPt.x(), tempPt.y())] = 2;
            break;
          }
        }
      }
    }

    if (startPoints.empty()) {
      if (yLoc2Cost.size() > 1) {
        // y
        auto yLoc2CostTemp = yLoc2Cost;
        auto yLocNextIt = yLoc2CostTemp.begin();
        ++yLocNextIt;
        for (auto yLocIt = yLoc2CostTemp.begin(); yLocNextIt != yLoc2CostTemp.end(); ++yLocNextIt) {
          yLoc2Cost[(yLocIt->first + yLocNextIt->first) / 2] = yLocIt->second + yLocNextIt->second;
          ++yLocIt;
        }
      }

      // get upper center coord
      for (auto &rect: rects) {
        Point centerPt;
        boost::polygon::center(centerPt, rect);
        if (yLoc2Cost.find(centerPt.y()) == yLoc2Cost.end()) {
          yLoc2Cost[centerPt.y()] = 2;
        }
      }

      for (auto xLocIt = xLoc2Cost.begin(); xLocIt != xLoc2Cost.end(); ++xLocIt) {
        for (auto yLocIt  = yLoc2Cost.begin(); yLocIt != yLoc2Cost.end(); ++yLocIt) {
          Point tempPt(xLocIt->first, yLocIt->first);
          for (auto &poly: currLayerPS) {
            if (contains(poly, tempPt)) {
              startPoints[frPoint(tempPt.x(), tempPt.y())] = 2;
              break;
            }
          }
        }
      }

    }
  } else if (getTech()->getLayer(currLayerNum)->getDir() == frcHorzPrefRoutingDir) {
    // if there is on grid start, then return
    for (auto xLocIt = xLoc2Cost.begin(); xLocIt != xLoc2Cost.end(); ++xLocIt) {
      for (auto yLocIt = yLoc2Cost.begin(); yLocIt != yLoc2Cost.end(); ++yLocIt) {
        Point tempPt(xLocIt->first, yLocIt->first);
        for (auto &poly: currLayerPS) {
          if (contains(poly, tempPt)) {
            startPoints[frPoint(tempPt.x(), tempPt.y())] = 2;
            break;
          }
        }
      }
    }

    if (!startPoints.empty()) {
      return;
    }

    // add half pitch if possible
    if (yLoc2Cost.size() > 1) {
      // y
      auto yLoc2CostTemp = yLoc2Cost;
      auto yLocNextIt = yLoc2CostTemp.begin();
      ++yLocNextIt;
      for (auto yLocIt = yLoc2CostTemp.begin(); yLocNextIt != yLoc2CostTemp.end(); ++yLocNextIt) {
        yLoc2Cost[(yLocIt->first + yLocNextIt->first) / 2] = yLocIt->second + yLocNextIt->second;
        ++yLocIt;
      }
    }

    // get lower center coord
    vector<Rectangle> rects;
    get_max_rectangles(rects, currLayerPS);
    for (auto &rect: rects) {
      Point centerPt;
      boost::polygon::center(centerPt, rect);
      if (yLoc2Cost.find(centerPt.y()) == yLoc2Cost.end()) {
        yLoc2Cost[centerPt.y()] = 2;
      }
    }

    for (auto xLocIt = xLoc2Cost.begin(); xLocIt != xLoc2Cost.end(); ++xLocIt) {
      for (auto yLocIt = yLoc2Cost.begin(); yLocIt != yLoc2Cost.end(); ++yLocIt) {
        Point tempPt(xLocIt->first, yLocIt->first);
        for (auto &poly: currLayerPS) {
          if (contains(poly, tempPt)) {
            startPoints[frPoint(tempPt.x(), tempPt.y())] = 2;
            break;
          }
        }
      }
    }

    if (startPoints.empty()) {
      if (yLoc2Cost.size() > 1) {
        // x
        auto xLoc2CostTemp = xLoc2Cost;
        auto xLocNextIt = xLoc2CostTemp.begin();
        ++xLocNextIt;
        for (auto xLocIt = xLoc2CostTemp.begin(); xLocNextIt != xLoc2CostTemp.end(); ++xLocNextIt) {
          xLoc2Cost[(xLocIt->first + xLocNextIt->first) / 2] = xLocIt->second + xLocNextIt->second;
          ++xLocIt;
        }
      }

      // get upper center coord
      for (auto &rect: rects) {
        Point centerPt;
        boost::polygon::center(centerPt, rect);
        if (xLoc2Cost.find(centerPt.x()) == xLoc2Cost.end()) {
          xLoc2Cost[centerPt.x()] = 2;
        }
      }

      for (auto xLocIt = xLoc2Cost.begin(); xLocIt != xLoc2Cost.end(); ++xLocIt) {
        for (auto yLocIt  = yLoc2Cost.begin(); yLocIt != yLoc2Cost.end(); ++yLocIt) {
          Point tempPt(xLocIt->first, yLocIt->first);
          for (auto &poly: currLayerPS) {
            if (contains(poly, tempPt)) {
              startPoints[frPoint(tempPt.x(), tempPt.y())] = 2;
              break;
            }
          }
        }
      }
    }
  }

  if (startPoints.empty()) {
    std::cout << "Error: no start point generated\n";
  }
  return;
}

void FlexPinPrep::getPinLayerAPStartPoints_shapeCenter(const frLayerNum &currLayerNum,
                                                       const PolygonSet &currLayerPS,
                                                       map<frCoord, map<frLayerNum, frTrackPattern*> > &xLoc2TrackPatterns,
                                                       map<frCoord, map<frLayerNum, frTrackPattern*> > &yLoc2TrackPatterns,
                                                       map<frPoint, int> &startPoints) {
  frPrefRoutingDirEnum currDir = getTech()->getLayer(currLayerNum)->getDir();
  // bool enableOutput = true;
  map<frCoord, int> xLoc2Cost, yLoc2Cost;
  // initialize loc cost
  std::vector<Rectangle> rects;
  get_max_rectangles(rects, currLayerPS);
  // xcost
  if (currDir == frcHorzPrefRoutingDir) {
    for (auto xLocIt = xLoc2TrackPatterns.begin(); xLocIt != xLoc2TrackPatterns.end(); ++xLocIt) {
      xLoc2Cost[xLocIt->first] = 1;
    }
    // shape center
    for (auto &rect: rects) {
      Point centerPt;
      center(centerPt, rect);
      yLoc2Cost[centerPt.y()] = 1;
    }
  }

  // ycost
  if (currDir == frcVertPrefRoutingDir) {
    for (auto yLocIt = yLoc2TrackPatterns.begin(); yLocIt != yLoc2TrackPatterns.end(); ++yLocIt) {
      yLoc2Cost[yLocIt->first] = 1;
    }
    // shape center
    for (auto &rect: rects) {
      Point centerPt;
      center(centerPt, rect);
      xLoc2Cost[centerPt.x()] = 1;
    }
  }

  for (auto xLocIt = xLoc2Cost.begin(); xLocIt != xLoc2Cost.end(); ++xLocIt) {
    for (auto yLocIt = yLoc2Cost.begin(); yLocIt != yLoc2Cost.end(); ++yLocIt) {
      Point tempPt(xLocIt->first, yLocIt->first);
      for (auto &poly: currLayerPS) {
        if (contains(poly, tempPt)) {
          startPoints[frPoint(tempPt.x(), tempPt.y())] = 2;
          break;
        }
      }
    }
  }

  if (startPoints.empty()) {
    std::cout << "Error: no start point generated\n";
  }
  return;
}

void FlexPinPrep::getPinLayerBBoxTrackPatterns(const frLayerNum &currLayerNum,
                                               const PolygonSet &currLayerPS,
                                               map<frCoord, map<frLayerNum, frTrackPattern*> > &xLoc2TrackPatterns,
                                               map<frCoord, map<frLayerNum, frTrackPattern*> > &yLoc2TrackPatterns) {
  map<frLayerNum, set<frCoord> > layerNum2TrackLocs;
  frLayerNum nextLayerNum = -1;
  if (currLayerNum + 2 < int(getTech()->getLayers().size())) {
    nextLayerNum = currLayerNum + 2;
  } else if (currLayerNum - 2 >= 0) {
    nextLayerNum = currLayerNum - 2;
  } else {
    cout << "Error: Current Design has only one metal layer in getPinLayerBBoxTrackPatterns\n";
    return;
  }
  // cout << " getPinLayerBBoxTrackPatterns " << currLayerNum << " " << nextLayerNum << "\n";
  auto currLayer = getTech()->getLayer(currLayerNum);
  auto nextLayer = getTech()->getLayer(nextLayerNum);
  // cout << " xxx0\n";
  frPrefRoutingDirEnum currPrefRouteDir = currLayer->getDir();
  frPrefRoutingDirEnum nextPrefRouteDir = nextLayer->getDir();
  // cout << " xxx1\n";
  auto currLayerPitch = currLayer->getPitch();
  auto nextLayerPitch = nextLayer->getPitch();
  // cout << " xxx2\n";
  Rectangle bloatBBox;
  extents(bloatBBox, currLayerPS);
  // bloatBBox = bloat(bloatBBox, currLayerPitch);
  frBox bbox(xl(bloatBBox), yl(bloatBBox), xh(bloatBBox), yh(bloatBBox));
  // get pref dir tracks 
  while (layerNum2TrackLocs.find(currLayerNum) == layerNum2TrackLocs.end()) {
    for (auto &tp: getDesign()->getTopBlock()->getTrackPatterns(currLayerNum)) {
      if (tp->isHorizontal()  && currPrefRouteDir == frcVertPrefRoutingDir ||
          !tp->isHorizontal() && currPrefRouteDir == frcHorzPrefRoutingDir) {
        int minTrackNum, maxTrackNum;
        frCoord lowerCoord = (tp->isHorizontal() ? bbox.left() : bbox.bottom());
        frCoord upperCoord = (tp->isHorizontal() ? bbox.right() : bbox.top());
        minTrackNum = (lowerCoord - tp->getStartCoord()) / (int)tp->getTrackSpacing();
        if (minTrackNum * (int)tp->getTrackSpacing() + tp->getStartCoord() < lowerCoord) {
          ++minTrackNum;
        }
        maxTrackNum = (upperCoord - tp->getStartCoord()) / (int)tp->getTrackSpacing();
        if (maxTrackNum * (int)tp->getTrackSpacing() + tp->getStartCoord() > upperCoord) {
          --maxTrackNum;
        }
        // cout << "minTrackNum / maxTrackNum = " << minTrackNum << " / " << maxTrackNum << "\n";


        for (int trackNum = minTrackNum; trackNum <= maxTrackNum; ++trackNum) {
          frCoord trackLoc = trackNum * tp->getTrackSpacing() + tp->getStartCoord();
          if (tp->isHorizontal()) {
            xLoc2TrackPatterns[trackLoc][currLayerNum] = tp.get();
          } else {
            yLoc2TrackPatterns[trackLoc][currLayerNum] = tp.get();
          }
          layerNum2TrackLocs[currLayerNum].insert(trackLoc);
        }
      }
    }
    // bloat the bbox if there is no track found in the current bbox;
    bloatBBox = bloat(bloatBBox, currLayerPitch);
    bbox = frBox(xl(bloatBBox), yl(bloatBBox), xh(bloatBBox), yh(bloatBBox));
  }

  extents(bloatBBox, currLayerPS);
  // bloatBBox = bloat(bloatBBox, nextLayerPitch);
  bbox = frBox(xl(bloatBBox), yl(bloatBBox), xh(bloatBBox), yh(bloatBBox));
  while (layerNum2TrackLocs.find(nextLayerNum) == layerNum2TrackLocs.end()) {
    for (auto &tp: getDesign()->getTopBlock()->getTrackPatterns(nextLayerNum)) {
      if (tp->isHorizontal()  && nextPrefRouteDir == frcVertPrefRoutingDir ||
          !tp->isHorizontal() && nextPrefRouteDir == frcHorzPrefRoutingDir) {
        int minTrackNum, maxTrackNum;
        frCoord lowerCoord = (tp->isHorizontal() ? bbox.left() : bbox.bottom());
        frCoord upperCoord = (tp->isHorizontal() ? bbox.right() : bbox.top());
        minTrackNum = (lowerCoord - tp->getStartCoord()) / (int)tp->getTrackSpacing();
        if (minTrackNum * (int)tp->getTrackSpacing() + tp->getStartCoord() < lowerCoord) {
          ++minTrackNum;
        }
        maxTrackNum = (upperCoord - tp->getStartCoord()) / (int)tp->getTrackSpacing();
        if (maxTrackNum * (int)tp->getTrackSpacing() + tp->getStartCoord() > upperCoord) {
          --maxTrackNum;
        }


        for (int trackNum = minTrackNum; trackNum <= maxTrackNum; ++trackNum) {
          frCoord trackLoc = trackNum * tp->getTrackSpacing() + tp->getStartCoord();
          if (tp->isHorizontal()) {
            xLoc2TrackPatterns[trackLoc][nextLayerNum] = tp.get();
          } else {
            yLoc2TrackPatterns[trackLoc][nextLayerNum] = tp.get();
          }
          layerNum2TrackLocs[nextLayerNum].insert(trackLoc);
        }
      }
    }
    // bloat the bbox if there is no track found in the current bbox;
    bloatBBox = bloat(bloatBBox, nextLayerPitch);
    bbox = frBox(xl(bloatBBox), yl(bloatBBox), xh(bloatBBox), yh(bloatBBox));
  }

}


void FlexPinPrep::getPinLayerBBox(const map<frLayerNum, PolygonSet> &layerNum2PS, 
                                  map<frLayerNum, Rectangle> &layerNum2BBox) {
  for (auto mapIt = layerNum2PS.begin(); mapIt != layerNum2PS.end(); ++mapIt) {
    auto currLayerNum = mapIt->first;
    Rectangle layerBBox;
    if (extents(layerBBox, mapIt->second)) {
      layerNum2BBox[currLayerNum] = layerBBox;
    }
  }
}

void FlexPinPrep::mergePinShapes(frPin* pin, frTransform &xform, map<frLayerNum, PolygonSet> &layerNum2PS) {
  auto pinLayer2PolySet = pin->getLayer2PolySet();
  for (auto layerIt = pinLayer2PolySet.begin(); layerIt != pinLayer2PolySet.end(); ++layerIt) {
    auto &layerNum = layerIt->first;
    auto &polySet = layerIt->second;
    for (auto &poly: polySet) {
      std::vector<Point> transformedPoints;
      for (auto it = boost::polygon::begin_points(poly); it != boost::polygon::end_points(poly); ++it) {
        frPoint tmpPt((*it).x(), (*it).y());
        tmpPt.transform(xform);
        transformedPoints.push_back(Point(tmpPt.x(), tmpPt.y()));
        // std::cout << "(" << tmpPt.x() << ", " << tmpPt.y() << ")\n";
      }
      Polygon tmpPoly;
      boost::polygon::set_points(tmpPoly, transformedPoints.begin(), transformedPoints.end());
      layerNum2PS[layerNum] += tmpPoly;
    }
  }
}

void FlexPinPrep::mergePinShapes(frPin* pin, map<frLayerNum, PolygonSet> &layerNum2PS) {
  auto pinLayer2PolySet = pin->getLayer2PolySet();
  for (auto layerIt = pinLayer2PolySet.begin(); layerIt != pinLayer2PolySet.end(); ++layerIt) {
    auto &layerNum = layerIt->first;
    auto &polySet = layerIt->second;
    for (auto &poly: polySet) {
      std::vector<Point> transformedPoints;
      for (auto it = boost::polygon::begin_points(poly); it != boost::polygon::end_points(poly); ++it) {
        frPoint tmpPt((*it).x(), (*it).y());
        // tmpPt.transform(xform);
        transformedPoints.push_back(Point(tmpPt.x(), tmpPt.y()));
        // std::cout << "(" << tmpPt.x() << ", " << tmpPt.y() << ")\n";
      }
      Polygon tmpPoly;
      boost::polygon::set_points(tmpPoly, transformedPoints.begin(), transformedPoints.end());
      layerNum2PS[layerNum] += tmpPoly;
    }
  }
}


void FlexPinPrep::mergePinShapes(const shared_ptr<frPin> &pin, map<frLayerNum, PolygonSet> &layerNum2PS) {
  // cout << "mergePinShapes\n";
  for (auto &uPinFig: pin->getFigs()) {
    auto pinFig = uPinFig.get();
    if (pinFig->typeId() == frcRect) {
      auto rectIn = static_cast<frRect*>(pinFig);
      auto currLayerNum = rectIn->getLayerNum();
      // cout << "currLayerNum = " << currLayerNum << "\n" << flush;
      // cout << "total layerNum = " << getTech()->getLayers().size() << "\n";
      if (getTech()->getLayer(currLayerNum)->getType() != frLayerTypeEnum::ROUTING) {
        continue;
      }
      // cout << "here " << currLayerNum << "\n" << flush;
      Rectangle rectOut;
      frRect2Rectangle(*rectIn, rectOut);
      layerNum2PS[currLayerNum] += rectOut;
    } else if (pinFig->typeId() == frcPolygon) {
      cout << "Warning: TODO: support polygon in mergePinShapes\n";
    } else {
      cout << "Warning: unsupported shape in mergePinShapes\n";
      continue;
    }
  }
}

// get endpoint of planar access given a direction
void FlexPinPrep::getPlanarEP(const frLayerNum &layerNum, const frPoint &startPt, const frDirEnum &dir, frPoint &endPt) {
  frCoord endX = startPt.x(), endY = startPt.y();
  frCoord stepSize = getTech()->getLayer(layerNum)->getPitch();
  if (getTech()->getTopLayerNum() >= layerNum + 2) {
    stepSize = getTech()->getLayer(layerNum + 2)->getPitch();
  }
  switch (dir) {
    case (frDirEnum::E):
      endX += stepSize;
      break;
    case (frDirEnum::S):
      endY -= stepSize;
      break;
    case (frDirEnum::W):
      endX -= stepSize;
      break;
    case (frDirEnum::N):
      endY += stepSize;
      break;
    default:
      std::cout << "unexpected direction in getPlanarEP\n";
  }
  endPt.set(endX, endY);
}


bool FlexPinPrep::isValidViaAccess(frPin *pinPtr,
                                   frInstTerm *instTerm,
                                   const std::vector<frBlockObject*> instObjs,
                                   const frLayerNum &layerNum,
                                   const frPoint &startPt,
                                   frViaDef* viaDef,
                                   viaPriorityTuple &priority) {
  bool enableOutput = false;
  bool isNotDefaultVia, isNotUpperAlign, isNotLowerAlign;
  frCoord nonEnclosedArea = 0, upperArea = 0, lowerArea = 0;
  
  

  double dbu = getTech()->getDBUPerUU();
  frInst* inst = instTerm->getInst();
  frBox mbox;
  frTransform xform, revertCellXForm;

  

  // inst->getTransform(xform);
  // inst->getRefBlock()->getBoundaryBBox(mbox);
  // frPoint size(mbox.right(), mbox.top());
  // termXForm.set(xform.orient());
  // termXForm.updateXform(size);
  inst->getUpdatedXform(xform);
  // xform.revert(revertCellXForm);
  revertCellXForm.set(-xform.xOffset(), -xform.yOffset());
  // std::cout << xform.xOffset() << " " << xform.yOffset() << "\n";
  // std::cout << revertCellXForm.xOffset() << " " << revertCellXForm.yOffset() << "\n";
  revertCellXForm.set(frcR0);

  auto cellObjs = instObjs;
  std::unique_ptr<frVia> via = std::make_unique<frVia>(viaDef);
  via->setOrigin(startPt);
  frTransform viaXform(startPt);
  via->setTransform(viaXform);
  if (instTerm->hasNet()) {
    via->addToNet(instTerm->getNet());
  } else {
    via->addToPin(pinPtr);
  }
  cellObjs.push_back(via.get());
  DRCWorker drcWorker(design, cellObjs);
  if (instTerm->hasNet()) {
    drcWorker.setTarget(instTerm->getNet());
  } else {
    drcWorker.setTarget(instTerm);
  }
  high_resolution_clock::time_point t0 = high_resolution_clock::now();
  drcWorker.init();
  high_resolution_clock::time_point t1 = high_resolution_clock::now();
  drcWorker.setup();
  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  drcWorker.main();
  high_resolution_clock::time_point t3 = high_resolution_clock::now();
  drcSpan0 += duration_cast<duration<double>>(t1 - t0);
  drcSpan1 += duration_cast<duration<double>>(t2 - t1);
  drcSpan2 += duration_cast<duration<double>>(t3 - t2);
  // drcWorker.report();

  // get priority information
  // isNotDefaultVia = (getTech()->getLayer(viaDef->getCutLayerNum())->getDefaultViaDef() != viaDef);
  isNotDefaultVia = !(viaDef->getDefault());
  frCoord ovlpArea;
  PolygonSet viaLayerPS1;
  for (auto &fig: via->getViaDef()->getLayer1Figs()) {
    frBox bbox;
    fig->getBBox(bbox);
    bbox.transform(viaXform);
    Rectangle bboxRect(bbox.left(), bbox.bottom(), bbox.right(), bbox.top());
    // std::cout << bbox.left() << " " << bbox.bottom() << " " << bbox.right() << " " << bbox.top() << "\n";
    viaLayerPS1 += bboxRect;
  }
  lowerArea = area(viaLayerPS1);

  Rectangle layer1Rect;
  extents(layer1Rect, viaLayerPS1);
  bool isLayer1Horz = (xh(layer1Rect) - xl(layer1Rect)) > (yh(layer1Rect) - yl(layer1Rect));
  isNotLowerAlign = (isLayer1Horz != (getTech()->getLayer(viaDef->getLayer1Num())->getDir() == frcHorzPrefRoutingDir));

  PolygonSet viaLayerPS2;
  for (auto &fig: via->getViaDef()->getLayer2Figs()) {
    frBox bbox;
    fig->getBBox(bbox);
    Rectangle bboxRect(bbox.left(), bbox.bottom(), bbox.right(), bbox.top());
    viaLayerPS2 += bboxRect;
  }

  Rectangle layer2Rect;
  extents(layer2Rect, viaLayerPS2);
  bool isLayer2Horz = (xh(layer2Rect) - xl(layer2Rect)) > (yh(layer2Rect) - yl(layer2Rect));
  isNotUpperAlign = (isLayer2Horz != (getTech()->getLayer(viaDef->getLayer2Num())->getDir() == frcHorzPrefRoutingDir));



  for (auto &pin: instTerm->getTerm()->getPins()) {
    auto pinLayer2PolySet = pin->getLayer2PolySet();
    if (pinLayer2PolySet.find(viaDef->getLayer1Num()) != pinLayer2PolySet.end()) {
      PolygonSet pinLayer1PS;
      auto &polySet = pinLayer2PolySet[viaDef->getLayer1Num()];
      for (auto &poly: polySet) {
        std::vector<Point> transformedPts;
        for (auto it = boost::polygon::begin_points(poly); it != boost::polygon::end_points(poly); ++it) {
          frPoint tmpPt((*it).x(), (*it).y());
          tmpPt.transform(xform);
          transformedPts.push_back(Point(tmpPt.x(), tmpPt.y()));
        }
        Polygon tmpPoly;
        boost::polygon::set_points(tmpPoly, transformedPts.begin(), transformedPts.end());
        pinLayer1PS += tmpPoly;
      }
      ovlpArea = area(viaLayerPS1 & pinLayer1PS);
    } else {
      ovlpArea = 0;
    }
    nonEnclosedArea += (area(viaLayerPS1) - ovlpArea);
  }

  // priority = make_tuple(isNotDefaultVia, nonEnclosedArea, isNotUpperAlign, isNotLowerAlign, upperArea, lowerArea);
  priority = make_tuple(isNotDefaultVia, isNotUpperAlign, nonEnclosedArea, isNotLowerAlign, upperArea, lowerArea);

  if (drcWorker.getViolations().empty()) {
    if (enableOutput) {
      std::cout << "  @@@clean via access\n";
    }
    // output for via access check script
    if (enableOutput) {
      frPoint transformedSP = startPt;
      transformedSP.transform(revertCellXForm);
      std::map<frOrientEnum, std::string> orient2Name = {{frcR0, "R0"}, 
                                           {frcR90, "R90"}, 
                                           {frcR180, "R180"}, 
                                           {frcR270, "R270"}, 
                                           {frcMY, "MY"}, 
                                           {frcMXR90, "MX90"},
                                           {frcMX, "MX"},
                                           {frcMYR90, "MY90"}};
      std::cout << "libPrepOutViaClean " << instTerm->getInst()->getRefBlock()->getName() << " "
                << instTerm->getTerm()->getName() << " " << viaDef->getName() << " "
                << transformedSP.x() << " " << transformedSP.y() 
                << " " << orient2Name[instTerm->getInst()->getOrient()] << " " << nonEnclosedArea << "\n";
    }
    return true;
  } else {
    if (enableOutput) {
      std::cout << "  @@@dirty via access\n";
      frPoint transformedSP = startPt;
      transformedSP.transform(revertCellXForm);
      std::map<frOrientEnum, std::string> orient2Name = {{frcR0, "R0"}, 
                                           {frcR90, "R90"}, 
                                           {frcR180, "R180"}, 
                                           {frcR270, "R270"}, 
                                           {frcMY, "MY"}, 
                                           {frcMXR90, "MX90"},
                                           {frcMX, "MX"},
                                           {frcMYR90, "MY90"}};
      std::cout << "libPrepOutViaDirty " << instTerm->getInst()->getRefBlock()->getName() << " "
                << instTerm->getTerm()->getName() << " " << viaDef->getName() << " "
                << transformedSP.x() << " " << transformedSP.y() 
                << " " << orient2Name[instTerm->getInst()->getOrient()] << "\n";
      auto violations = drcWorker.getViolations();
      for (auto &marker: violations) {
        // std::cout << "  #marker = " << violations.size() << "\n";
        frBox bbox;
        marker.getBBox(bbox);
        std::cout << "  violation type: " << int(marker.getConstraint()->typeId()) << "\n";
        std::cout << "    bbox = (" << bbox.left() / dbu << ", " << bbox.bottom() / dbu << ") - ("
                  << bbox.right() / dbu << ", " << bbox.top() / dbu << ")\n";
      }
    }
    return false;
  }
}

bool FlexPinPrep::isValidViaAccess(frPin *pinPtr,
                                   frTerm *term,
                                   const std::vector<frBlockObject*> instObjs,
                                   const frLayerNum &layerNum,
                                   const frPoint &startPt,
                                   frViaDef* viaDef,
                                   viaPriorityTuple &priority) {
  bool enableOutput = false;
  bool isNotDefaultVia, isNotUpperAlign, isNotLowerAlign;
  frCoord nonEnclosedArea = 0, upperArea = 0, lowerArea = 0;

  double dbu = getTech()->getDBUPerUU();

  auto cellObjs = instObjs;
  std::unique_ptr<frVia> via = std::make_unique<frVia>(viaDef);
  via->setOrigin(startPt);
  frTransform viaXform(startPt);
  via->setTransform(viaXform);
  if (term->hasNet()) {
    via->addToNet(term->getNet());
  } else {
    via->addToPin(pinPtr);
  }
  cellObjs.push_back(via.get());
  DRCWorker drcWorker(design, cellObjs);
  if (term->hasNet()) {
    drcWorker.setTarget(term->getNet());
  } else {
    drcWorker.setTarget(term);
  }
  high_resolution_clock::time_point t0 = high_resolution_clock::now();
  drcWorker.init();
  high_resolution_clock::time_point t1 = high_resolution_clock::now();
  drcWorker.setup();
  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  drcWorker.main();
  high_resolution_clock::time_point t3 = high_resolution_clock::now();
  drcSpan0 += duration_cast<duration<double>>(t1 - t0);
  drcSpan1 += duration_cast<duration<double>>(t2 - t1);
  drcSpan2 += duration_cast<duration<double>>(t3 - t2);

  // get priority information
  // isNotDefaultVia = (getTech()->getLayer(viaDef->getCutLayerNum())->getDefaultViaDef() != viaDef); 
  isNotDefaultVia = !(viaDef->getDefault());
  frCoord ovlpArea;
  // std::unique_ptr<frVia> origVia = std::make_unique<frVia>(viaDef);
  // frPoint transformedSP = startPt;
  // origVia->setOrigin(transformedSP);
  // frTransform transformedViaXform(startPt);
  // origVia->setTransform(transformedViaXform);
  for (auto &pin: term->getPins()) {
    auto pinLayer2PolySet = pin->getLayer2PolySet();
    PolygonSet viaLayerPS1;
    for (auto &fig: via->getViaDef()->getLayer1Figs()) {
      frBox bbox;
      fig->getBBox(bbox);
      bbox.transform(viaXform);
      Rectangle bboxRect(bbox.left(), bbox.bottom(), bbox.right(), bbox.top());
      viaLayerPS1 += bboxRect;
    }
    lowerArea = area(viaLayerPS1);
    if (pinLayer2PolySet.find(viaDef->getLayer1Num()) != pinLayer2PolySet.end()) {
      PolygonSet pinLayer1PS;
      auto &polySet = pinLayer2PolySet[viaDef->getLayer1Num()];
      for (auto &poly: polySet) {
        std::vector<Point> transformedPts;
        for (auto it = boost::polygon::begin_points(poly); it != boost::polygon::end_points(poly); ++it) {
          frPoint tmpPt((*it).x(), (*it).y());
          // tmpPt.transform(xform);
          transformedPts.push_back(Point(tmpPt.x(), tmpPt.y()));
        }
        Polygon tmpPoly;
        boost::polygon::set_points(tmpPoly, transformedPts.begin(), transformedPts.end());
        pinLayer1PS += tmpPoly;
      }
      ovlpArea = area(viaLayerPS1 & pinLayer1PS);
    } else {
      ovlpArea = 0;
    }
    nonEnclosedArea += (area(viaLayerPS1) - ovlpArea);

    Rectangle layer1Rect;
    extents(layer1Rect, viaLayerPS1);
    bool isLayer1Horz = (xh(layer1Rect) - xl(layer1Rect)) > (yh(layer1Rect) - yl(layer1Rect));
    isNotLowerAlign = (isLayer1Horz != (getTech()->getLayer(viaDef->getLayer1Num())->getDir() == frcHorzPrefRoutingDir));

    PolygonSet viaLayerPS2;
    for (auto &fig: via->getViaDef()->getLayer2Figs()) {
      frBox bbox;
      fig->getBBox(bbox);
      Rectangle bboxRect(bbox.left(), bbox.bottom(), bbox.right(), bbox.top());
      viaLayerPS2 += bboxRect;
    }
    // upperArea = area(viaLayerPS2);
    // if (pinLayer2PolySet.find(viaDef->getLayer2Num()) != pinLayer2PolySet.end()) {
    //   ovlpArea = area(viaLayerPS2 & pinLayer2PolySet[viaDef->getLayer2Num()]);
    // } else {
    //   ovlpArea = 0;
    // }
    // nonEnclosedArea += (area(viaLayerPS2) - ovlpArea);

    Rectangle layer2Rect;
    extents(layer2Rect, viaLayerPS2);
    bool isLayer2Horz = (xh(layer2Rect) - xl(layer2Rect)) > (yh(layer2Rect) - yl(layer2Rect));
    isNotUpperAlign = (isLayer2Horz != (getTech()->getLayer(viaDef->getLayer2Num())->getDir() == frcHorzPrefRoutingDir));


  }

  // priority = make_tuple(isNotDefaultVia, nonEnclosedArea, isNotUpperAlign, isNotLowerAlign, upperArea, lowerArea);
  priority = make_tuple(isNotDefaultVia, isNotUpperAlign, nonEnclosedArea, isNotLowerAlign, upperArea, lowerArea);





  if (drcWorker.getViolations().empty()) {
    if (enableOutput) {
      std::cout << "  @@@clean via access\n";
      // output for via access check script
      frPoint transformedSP = startPt;
      std::map<frOrientEnum, std::string> orient2Name = {{frcR0, "R0"}, 
                                           {frcR90, "R90"}, 
                                           {frcR180, "R180"}, 
                                           {frcR270, "R270"}, 
                                           {frcMY, "MY"}, 
                                           {frcMXR90, "MX90"},
                                           {frcMX, "MX"},
                                           {frcMYR90, "MY90"}};
      std::cout << "termPrepOutViaClean " << term->getName() << " " << viaDef->getName() << " "
                << transformedSP.x() << " " << transformedSP.y() << "\n";
    }
    return true;
  } else {
    if (enableOutput) {
      std::cout << "  @@@dirty via access\n";
      frPoint transformedSP = startPt;
      std::map<frOrientEnum, std::string> orient2Name = {{frcR0, "R0"}, 
                                           {frcR90, "R90"}, 
                                           {frcR180, "R180"}, 
                                           {frcR270, "R270"}, 
                                           {frcMY, "MY"}, 
                                           {frcMXR90, "MX90"},
                                           {frcMX, "MX"},
                                           {frcMYR90, "MY90"}};
      std::cout << "libPrepOutViaDirty " << term->getName() << " " << viaDef->getName() << " "
                << transformedSP.x() << " " << transformedSP.y() << "\n";
      auto violations = drcWorker.getViolations();
      for (auto &marker: violations) {
        // std::cout << "  #marker = " << violations.size() << "\n";
        frBox bbox;
        marker.getBBox(bbox);
        std::cout << "  violation type: " << int(marker.getConstraint()->typeId()) << "\n";
        std::cout << "    bbox = (" << bbox.left() / dbu << ", " << bbox.bottom() / dbu << ") - ("
                  << bbox.right() / dbu << ", " << bbox.top() / dbu << ")\n";
      }
    }
    return false;
  }
}

bool FlexPinPrep::isValidPlanarAccess(frPin* pinPtr,
                                      frInstTerm *instTerm,
                                      const std::vector<frBlockObject*> instObjs,
                                      const frLayerNum &layerNum,
                                      const frPoint &startPt,
                                      const frPoint &endPt,
                                      const PolygonSet &layerPS) {
  bool enableOutput = false;
  double dbu = getTech()->getDBUPerUU();
  
  for (auto &poly: layerPS) {
    if (contains(poly, Point(endPt.x(), endPt.y()))) {
      return false;
    }
  }

  frInst* inst = instTerm->getInst();
  frBox mbox;
  frTransform xform, termXForm, revertCellXForm;

  

  inst->getTransform(xform);
  inst->getRefBlock()->getBoundaryBBox(mbox);
  frPoint size(mbox.right(), mbox.top());
  termXForm.set(xform.orient());
  termXForm.updateXform(size);
  // xform.revert(revertCellXForm);
  revertCellXForm.set(-xform.xOffset(), -xform.yOffset());
  // std::cout << xform.xOffset() << " " << xform.yOffset() << "\n";
  // std::cout << revertCellXForm.xOffset() << " " << revertCellXForm.yOffset() << "\n";
  revertCellXForm.set(frcR0);
  // xform.updateXform(size);
  // xform.revert(revertXForm);


  auto cellObjs = instObjs;

  // instantiate a wire and check DRC
  std::unique_ptr<frPathSeg> apSeg = std::make_unique<frPathSeg>() ;
  apSeg->setLayerNum(layerNum);
  if (startPt < endPt) {
    apSeg->setPoints(startPt, endPt);
  } else {
    apSeg->setPoints(endPt, startPt);
  }
  apSeg->setStyle(getTech()->getLayer(layerNum)->getDefaultSegStyle());
  if (instTerm->hasNet()) {
    apSeg->addToNet(instTerm->getNet());
  } else {
    apSeg->addToPin(pinPtr);
  }
  cellObjs.push_back(apSeg.get());
  DRCWorker drcWorker(design, cellObjs);
  if (instTerm->hasNet()) {
    drcWorker.setTarget(instTerm->getNet());
  } else {
    drcWorker.setTarget(instTerm);
  }
  high_resolution_clock::time_point t0 = high_resolution_clock::now();
  drcWorker.init();
  high_resolution_clock::time_point t1 = high_resolution_clock::now();
  drcWorker.setup();
  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  drcWorker.main();
  high_resolution_clock::time_point t3 = high_resolution_clock::now();
  drcSpan0 += duration_cast<duration<double>>(t1 - t0);
  drcSpan1 += duration_cast<duration<double>>(t2 - t1);
  drcSpan2 += duration_cast<duration<double>>(t3 - t2);

  frCoord layerWidth = getTech()->getLayer(layerNum)->getWidth();

  if (drcWorker.getViolations().empty()) {
    if (enableOutput) {
      std::cout << "  @@@clean planar access\n";
      frPoint transSP, transEP;
      if (startPt < endPt) {
        transSP = startPt;
        transEP = endPt;
      } else {
        transSP = endPt;
        transEP = startPt;
      }
      transSP.transform(revertCellXForm);
      transEP.transform(revertCellXForm);
      std::map<frOrientEnum, std::string> orient2Name = {{frcR0, "R0"}, 
                                                       {frcR90, "R90"}, 
                                                       {frcR180, "R180"}, 
                                                       {frcR270, "R270"}, 
                                                       {frcMY, "MY"}, 
                                                       {frcMXR90, "MX90"},
                                                       {frcMX, "MX"},
                                                       {frcMYR90, "MY90"}};
      if (transSP.x() == transEP.x()) {
        std::cout << "libPrepOutSegClean " << instTerm->getInst()->getRefBlock()->getName() << " "
                  << instTerm->getTerm()->getName() << " "
                 << transSP.x() << " " << transSP.y() - layerWidth / 2 << " "
                 << transEP.x() << " " << transEP.y() + layerWidth / 2 << " "
                 << " " << orient2Name[instTerm->getInst()->getOrient()] << "\n";
      } else {
        std::cout << "libPrepOutSegClean " << instTerm->getInst()->getRefBlock()->getName() << " "
                  << instTerm->getTerm()->getName() << " "
                 << transSP.x() - layerWidth / 2 << " " << transSP.y() << " "
                 << transEP.x() + layerWidth / 2 << " " << transEP.y() << " "
                 << " " << orient2Name[instTerm->getInst()->getOrient()] << "\n";
      }
    }
    return true;
  } else {
    if (enableOutput) {
      std::cout << "  @@@dirty planar access\n";
      frPoint transSP, transEP;
      if (startPt < endPt) {
        transSP = startPt;
        transEP = endPt;
      } else {
        transSP = endPt;
        transEP = startPt;
      }
      transSP.transform(revertCellXForm);
      transEP.transform(revertCellXForm);
      std::map<frOrientEnum, std::string> orient2Name = {{frcR0, "R0"}, 
                                                       {frcR90, "R90"}, 
                                                       {frcR180, "R180"}, 
                                                       {frcR270, "R270"}, 
                                                       {frcMY, "MY"}, 
                                                       {frcMXR90, "MX90"},
                                                       {frcMX, "MX"},
                                                       {frcMYR90, "MY90"}};
      if (transSP.x() == transEP.x()) {
        std::cout << "libPrepOutSegDirty " << instTerm->getInst()->getRefBlock()->getName() << " "
                  << instTerm->getTerm()->getName() << " "
                 << transSP.x() << " " << transSP.y() - layerWidth / 2 << " "
                 << transEP.x() << " " << transEP.y() + layerWidth / 2 << " "
                 << " " << orient2Name[instTerm->getInst()->getOrient()] << "\n";
      } else {
        std::cout << "libPrepOutSegDirty " << instTerm->getInst()->getRefBlock()->getName() << " "
                  << instTerm->getTerm()->getName() << " "
                 << transSP.x() - layerWidth / 2 << " " << transSP.y() << " "
                 << transEP.x() + layerWidth / 2 << " " << transEP.y() << " "
                 << " " << orient2Name[instTerm->getInst()->getOrient()] << "\n";
      }
      auto violations = drcWorker.getViolations();
      for (auto &marker: violations) {
        // std::cout << "  #marker = " << violations.size() << "\n";
        frBox bbox;
        marker.getBBox(bbox);
        std::cout << "  violation type: " << int(marker.getConstraint()->typeId()) << "\n";
        std::cout << "    bbox = (" << bbox.left() / dbu << ", " << bbox.bottom() / dbu << ") - ("
                  << bbox.right() / dbu << ", " << bbox.top() / dbu << ")\n";
      }
    }

    return false;

  }

}


bool FlexPinPrep::isValidPlanarAccess(frPin* pinPtr,
                                      frTerm *term,
                                      const std::vector<frBlockObject*> instObjs,
                                      const frLayerNum &layerNum,
                                      const frPoint &startPt,
                                      const frPoint &endPt,
                                      const PolygonSet &layerPS) {
  bool enableOutput = false;
  double dbu = getTech()->getDBUPerUU();

  auto cellObjs = instObjs;
  for (auto &poly: layerPS) {
    if (contains(poly, Point(endPt.x(), endPt.y()))) {
      return false;
    }
  }

  // instantiate a wire and check DRC
  std::unique_ptr<frPathSeg> apSeg = std::make_unique<frPathSeg>() ;
  apSeg->setLayerNum(layerNum);
  if (startPt < endPt) {
    apSeg->setPoints(startPt, endPt);
  } else {
    apSeg->setPoints(endPt, startPt);
  }
  apSeg->setStyle(getTech()->getLayer(layerNum)->getDefaultSegStyle());
  if (term->hasNet()) {
    apSeg->addToNet(term->getNet());
  } else {
    apSeg->addToPin(pinPtr);
  }
  cellObjs.push_back(apSeg.get());
  DRCWorker drcWorker(design, cellObjs);
  if (term->hasNet()) {
    drcWorker.setTarget(term->getNet());
  } else {
    drcWorker.setTarget(term);
  }
  high_resolution_clock::time_point t0 = high_resolution_clock::now();
  drcWorker.init();
  high_resolution_clock::time_point t1 = high_resolution_clock::now();
  drcWorker.setup();
  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  drcWorker.main();
  high_resolution_clock::time_point t3 = high_resolution_clock::now();
  drcSpan0 += duration_cast<duration<double>>(t1 - t0);
  drcSpan1 += duration_cast<duration<double>>(t2 - t1);
  drcSpan2 += duration_cast<duration<double>>(t3 - t2);

  frCoord layerWidth = getTech()->getLayer(layerNum)->getWidth();

  if (drcWorker.getViolations().empty()) {
    if (enableOutput) {
      std::cout << "  @@@clean planar access\n";
      frPoint transSP, transEP;
      if (startPt < endPt) {
        transSP = startPt;
        transEP = endPt;
      } else {
        transSP = endPt;
        transEP = startPt;
      }
      std::map<frOrientEnum, std::string> orient2Name = {{frcR0, "R0"}, 
                                                       {frcR90, "R90"}, 
                                                       {frcR180, "R180"}, 
                                                       {frcR270, "R270"}, 
                                                       {frcMY, "MY"}, 
                                                       {frcMXR90, "MX90"},
                                                       {frcMX, "MX"},
                                                       {frcMYR90, "MY90"}};
      if (transSP.x() == transEP.x()) {
        std::cout << "termPrepOutSegClean " << term->getName() << " "
                 << transSP.x() << " " << transSP.y() - layerWidth / 2 << " "
                 << transEP.x() << " " << transEP.y() + layerWidth / 2 << " " << "\n";
      } else {
        std::cout << "termPrepOutSegClean " << term->getName() << " "
                 << transSP.x() - layerWidth / 2 << " " << transSP.y() << " "
                 << transEP.x() + layerWidth / 2 << " " << transEP.y() << " " << "\n";
      }
    }
    return true;
  } else {
    if (enableOutput) {
      std::cout << "  @@@dirty planar access\n";
      frPoint transSP, transEP;
      if (startPt < endPt) {
        transSP = startPt;
        transEP = endPt;
      } else {
        transSP = endPt;
        transEP = startPt;
      }
      std::map<frOrientEnum, std::string> orient2Name = {{frcR0, "R0"}, 
                                                       {frcR90, "R90"}, 
                                                       {frcR180, "R180"}, 
                                                       {frcR270, "R270"}, 
                                                       {frcMY, "MY"}, 
                                                       {frcMXR90, "MX90"},
                                                       {frcMX, "MX"},
                                                       {frcMYR90, "MY90"}};
      if (transSP.x() == transEP.x()) {
        std::cout << "termPrepOutSegDirty " << term->getName() << " "
                 << transSP.x() << " " << transSP.y() - layerWidth / 2 << " "
                 << transEP.x() << " " << transEP.y() + layerWidth / 2 << " " << "\n";
      } else {
        std::cout << "libPrepOutSegDirty " << term->getName() << " "
                 << transSP.x() - layerWidth / 2 << " " << transSP.y() << " "
                 << transEP.x() + layerWidth / 2 << " " << transEP.y() << " " << "\n";
      }
      auto violations = drcWorker.getViolations();
      for (auto &marker: violations) {
        // std::cout << "  #marker = " << violations.size() << "\n";
        frBox bbox;
        marker.getBBox(bbox);
        std::cout << "  violation type: " << int(marker.getConstraint()->typeId()) << "\n";
        std::cout << "    bbox = (" << bbox.left() / dbu << ", " << bbox.bottom() / dbu << ") - ("
                  << bbox.right() / dbu << ", " << bbox.top() / dbu << ")\n";
      }
    }

    return false;

  }

}
