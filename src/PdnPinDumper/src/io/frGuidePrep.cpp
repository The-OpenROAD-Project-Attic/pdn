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

#include <algorithm>
#include <iterator>
#include <iostream>
#include <map>
#include <boost/graph/connected_components.hpp>
#include "global.h"
#include "io/frGuidePrep.h"
#include "io/frShapeUtil.h"
#include "io/frUtil.h"
#include "db/obj/frRoute.h"
//#include <typeinfo>


namespace fr {
  frCoord GCELLGRIDX = -1;
  frCoord GCELLGRIDY = -1;
  frCoord GCELLOFFSETX = -1;
  frCoord GCELLOFFSETY = -1;
  std::vector<std::shared_ptr<fr::frLayer> > *designLayers;

  frCoord getGCELLGRIDX() {
    return GCELLGRIDX;
  }

  frCoord getGCELLGRIDY() {
    return GCELLGRIDY;
  }

  frCoord getGCELLOFFSETX() {
    return GCELLOFFSETX;
  }

  frCoord getGCELLOFFSETY() {
    return GCELLOFFSETY;
  }

  
  void setGCELLGRIDX(frCoord &gCellGridXIn) {
    GCELLGRIDX = gCellGridXIn;
  }
  void setGCELLGRIDY(frCoord &gCellGridYIn) {
    GCELLGRIDY = gCellGridYIn;
  }
  void setGCELLOFFSETX(frCoord &gCellOffsetXIn) {
    GCELLOFFSETX = gCellOffsetXIn;
  }
  void setGCELLOFFSETY(frCoord &gCellOffsetYIn) {
    GCELLOFFSETY = gCellOffsetYIn;
  }

  void getGCellParam(std::map<std::string, fr::frCollection<fr::frRect> > &rawGuide, std::vector<std::shared_ptr<fr::frLayer> > &vlayers) {
    getGCellSizeParam(rawGuide, vlayers);
    getGCellOffsetParam(rawGuide, vlayers);
  }

  void getGCellSizeParam(std::map<std::string, fr::frCollection<fr::frRect> > &rawGuide, std::vector<std::shared_ptr<fr::frLayer> > &vlayers) {
    std::map<frCoord, int> guideGridXMap, guideGridYMap;
    // get GCell size information loop
    for (auto guideIt = rawGuide.begin(); guideIt != rawGuide.end(); ++guideIt) {
      std::string netName = guideIt->first;
      auto &rects = guideIt->second;
      for (auto &rect: rects) {
        frLayerNum layerNum = rect.getLayerNum();
        frBox guideBBox;
        rect.getBBox(guideBBox);

        // get GCell width
        frCoord guideWidth = -1;
        if (vlayers[layerNum]->getDir() == frcHorzPrefRoutingDir) {
          guideWidth = guideBBox.top() - guideBBox.bottom();
        } else if (vlayers[layerNum]->getDir() == frcVertPrefRoutingDir) {
          guideWidth = guideBBox.right() - guideBBox.left();
        }

        // update GCell size info
        if (guideWidth == -1) {
          continue;
        } else {
          if (vlayers[layerNum]->getDir() == frcHorzPrefRoutingDir) {
            if (guideGridYMap.find(guideWidth) == guideGridYMap.end()) {
              guideGridYMap[guideWidth] = 0;
            }
            guideGridYMap[guideWidth]++;
          } else if (vlayers[layerNum]->getDir() == frcVertPrefRoutingDir) {
            if (guideGridXMap.find(guideWidth) == guideGridXMap.end()) {
              guideGridXMap[guideWidth] = 0;
            }
            guideGridXMap[guideWidth]++;
          }
        }

      }

    }

    frCoord tmpGCELLGRIDX = -1, tmpGCELLGRIDY = -1;
    int tmpGCELLGRIDXCnt = -1, tmpGCELLGRIDYCnt = -1;
    for (auto mapIt = guideGridXMap.begin(); mapIt != guideGridXMap.end(); ++mapIt) {
      auto cnt = mapIt->second;
      if (cnt > tmpGCELLGRIDXCnt) {
        tmpGCELLGRIDXCnt = cnt;
        tmpGCELLGRIDX = mapIt->first;
      }
    }

    for (auto mapIt = guideGridYMap.begin(); mapIt != guideGridYMap.end(); ++mapIt) {
      auto cnt = mapIt->second;
      if (cnt > tmpGCELLGRIDYCnt) {
        tmpGCELLGRIDYCnt = cnt;
        tmpGCELLGRIDY = mapIt->first;
      }
    }

    if (tmpGCELLGRIDX != -1) {
      setGCELLGRIDX(tmpGCELLGRIDX);
    }
    if (tmpGCELLGRIDY != -1) {
      setGCELLGRIDY(tmpGCELLGRIDY);
    }

  }
  


  void getGCellOffsetParam(std::map<std::string, fr::frCollection<fr::frRect> > &rawGuide, std::vector<std::shared_ptr<fr::frLayer> > &vlayers) {
    std::map<frCoord, int> guideOffsetXMap, guideOffsetYMap;
    frCoord GCELLGRIDX = getGCELLGRIDX();
    frCoord GCELLGRIDY = getGCELLGRIDY();

    // get GCell offset information loop
    for (auto guideIt = rawGuide.begin(); guideIt != rawGuide.end(); ++guideIt) {
      std::string netName = guideIt->first;
      auto &rects = guideIt->second;
      for (auto &rect: rects) {
        frLayerNum layerNum = rect.getLayerNum();
        frBox guideBBox;
        rect.getBBox(guideBBox);

        // get GCell width
        frCoord guideOffset = -1;
        if (vlayers[layerNum]->getDir() == frcHorzPrefRoutingDir) {
          guideOffset = guideBBox.left() % GCELLGRIDX;
        } else if (vlayers[layerNum]->getDir() == frcVertPrefRoutingDir) {
          guideOffset = guideBBox.bottom() % GCELLGRIDY;
        }

        // update GCell size info
        if (guideOffset == -1) {
          continue;
        } else {
          if (vlayers[layerNum]->getDir() == frcHorzPrefRoutingDir) {
            if (guideOffsetXMap.find(guideOffset) == guideOffsetXMap.end()) {
              guideOffsetXMap[guideOffset] = 0;
            }
            guideOffsetXMap[guideOffset]++;
          } else if (vlayers[layerNum]->getDir() == frcVertPrefRoutingDir) {
            if (guideOffsetYMap.find(guideOffset) == guideOffsetYMap.end()) {
              guideOffsetYMap[guideOffset] = 0;
            }
            guideOffsetYMap[guideOffset]++;
          }
        }

      }

    }
    frCoord tmpGCELLOFFSETX = -1, tmpGCELLOFFSETY = -1;
    int tmpGCELLOFFSETXCnt = -1, tmpGCELLOFFSETYCnt = -1;
    for (auto mapIt = guideOffsetXMap.begin(); mapIt != guideOffsetXMap.end(); ++mapIt) {
      auto cnt = mapIt->second;
      if (cnt > tmpGCELLOFFSETXCnt) {
        tmpGCELLOFFSETXCnt = cnt;
        tmpGCELLOFFSETX = mapIt->first;
      }
    }

    for (auto mapIt = guideOffsetYMap.begin(); mapIt != guideOffsetYMap.end(); ++mapIt) {
      auto cnt = mapIt->second;
      if (cnt > tmpGCELLOFFSETYCnt) {
        tmpGCELLOFFSETYCnt = cnt;
        tmpGCELLOFFSETY = mapIt->first;
      }
    }

    if (tmpGCELLOFFSETX != -1) {
      setGCELLOFFSETX(tmpGCELLOFFSETX);
    }
    if (tmpGCELLOFFSETY != -1) {
      setGCELLOFFSETY(tmpGCELLOFFSETY);
    }

  }

  void genGCell2PinMap(std::map<std::string, std::shared_ptr<fr::frBlock> > &macros,
                       std::map<std::string, std::shared_ptr<fr::frNet> > &nets,
                       std::map<std::string, std::map<frPoint, std::set<std::pair<frBlockObject*, frLayerNum> > > > &gCell2PinMap) {
    //bool debugMode = false;
    //frCoord GCELLGRIDX = getGCELLGRIDX();
    //frCoord GCELLGRIDY = getGCELLGRIDY();
    //frCoord GCELLOFFSETX = getGCELLOFFSETX();
    //frCoord GCELLOFFSETY = getGCELLOFFSETY();
    if (VERBOSE > 0) {
      std::cout <<std::endl <<"gen gcell to pin map ..." <<std::endl;
    }
    int cnt = 0;
    for (auto netIt = nets.begin(); netIt != nets.end(); ++netIt) {
      auto netName = netIt->first;
      auto &net = netIt->second;
      // std::cout << "net: " << netName << std::endl;
      // update for inst term
      for (auto &instTerm: net->getInstTerms()) {
        frString cellName;
        frTransform xform;
        frBox   mbox;
        instTerm->getInst()->getTransform(xform);
        instTerm->getInst()->getRefName(cellName);
        macros[cellName]->getBBox(mbox);
        frPoint size(mbox.right(), mbox.top());
        updateXform(xform, size);
        auto absInstTerm = std::make_shared<frInstTerm>(*instTerm, xform);

        frString termName, instName;
        auto term = absInstTerm->getTerm();
        updateGCell2PinMap(static_cast<frBlockObject*>(instTerm.get()), term, netName, gCell2PinMap);
      }
      // update for term
      for (auto term: net->getTerms()) {
        updateGCell2PinMap(static_cast<frBlockObject*>(term), term, netName, gCell2PinMap);
      }
      cnt++;
      if (VERBOSE > 0) {
        if (cnt % 10000 == 0) {
          std::cout <<"  complete " <<cnt <<" nets" <<std::endl;
        }
      }
    }

  }

  // update GCell2PinMap based on whether the term is intersecting with any GCell
  void updateGCell2PinMap(frBlockObject* blockObj,
                          frTerm *term,
                          std::string netName,
                          std::map<std::string, std::map<frPoint, std::set<std::pair<frBlockObject*, frLayerNum> > > > &gCell2PinMap) {
    bool debugMode = false;
    frCoord GCELLGRIDX = getGCELLGRIDX();
    frCoord GCELLGRIDY = getGCELLGRIDY();
    frCoord GCELLOFFSETX = getGCELLOFFSETX();
    frCoord GCELLOFFSETY = getGCELLOFFSETY();
    frLayerNum layerNum;

    for (auto &pin: term->getPins()) {
      //std::cout << term->getName() << "\n"; 
      for (auto &uPinFig: pin->getFigs()) {
        auto pinFig = uPinFig.get();
        boostPolygon pinFigPoly;
        frCoord llx, lly, urx, ury;
        frCoord gCellMinX, gCellMinY, gCellMaxX, gCellMaxY;
        
        // get the exact pinFigPoly
        // get the GCells which the BBox of the pinFig is intersecting with
        frBox pinFigBBox;
        pinFig->getBBox(pinFigBBox);
        llx = pinFigBBox.left();
        lly = pinFigBBox.bottom();
        urx = pinFigBBox.right();
        ury = pinFigBBox.top();
        // get min/max GCell center x y
        gCellMinX = (llx - GCELLOFFSETX) / GCELLGRIDX * GCELLGRIDX + GCELLOFFSETX + GCELLGRIDX / 2;
        gCellMinY = (lly - GCELLOFFSETY) / GCELLGRIDY * GCELLGRIDY + GCELLOFFSETY + GCELLGRIDY / 2;
        gCellMaxX = (urx - GCELLOFFSETX) / GCELLGRIDX * GCELLGRIDX + GCELLOFFSETX + GCELLGRIDX / 2;
        gCellMaxY = (ury - GCELLOFFSETY) / GCELLGRIDY * GCELLGRIDY + GCELLOFFSETY + GCELLGRIDY / 2;
        // get exact pin shape
        switch(pinFig->typeId()) {
          case frcRect:
            frRect2Poly(*(static_cast<frRect*>(pinFig)), pinFigPoly);
            layerNum = (static_cast<frRect*>(pinFig))->getLayerNum();
            break;
          case frcPolygon:
            frPolygon2Poly(*(static_cast<frPolygon*>(pinFig)), pinFigPoly);
            layerNum = (static_cast<frPolygon*>(pinFig))->getLayerNum();
            break;
          default:
            continue;
            break;
        }
        // get exact overlappping between pin and gcells
        for (frCoord gCellLocX = gCellMinX; gCellLocX <= gCellMaxX; gCellLocX += GCELLGRIDX) {
          for (frCoord gCellLocY = gCellMinY; gCellLocY <= gCellMaxY; gCellLocY += GCELLGRIDY) {
            frPoint gCellCenter(gCellLocX, gCellLocY);
            frBox gCellBox = point2GCellBox(gCellCenter);
            boostPolygon gCellPoly;
            frBox2Poly(gCellBox, gCellPoly);
            if (boost::geometry::intersects(gCellPoly, pinFigPoly)) {
              if (debugMode) {
                std::cout << "  GCell: (" << gCellBox.left() << ", " << gCellBox.bottom()
                          << ")--(" << gCellBox.right() << ", " << gCellBox.top()
                          << ") intersecting with Term/InstTerm on layer " << layerNum << std::endl;
              }
              if (gCell2PinMap.find(netName) == gCell2PinMap.end()) {
                gCell2PinMap[netName] = std::map<frPoint, std::set<std::pair<frBlockObject*, frLayerNum> > >();
              }
              if (gCell2PinMap[netName].find(gCellCenter) == gCell2PinMap[netName].end()) {
                gCell2PinMap[netName][gCellCenter] = std::set<std::pair<frBlockObject*, frLayerNum> >();
              }
              gCell2PinMap[netName][gCellCenter].insert(std::make_pair(blockObj, layerNum));
            }
          }
        }
      }
    }
  }

  frBox point2GCellBox(frPoint &pointIn) {
    
    frCoord llx, lly, urx, ury;
    frCoord GCELLGRIDX = getGCELLGRIDX();
    frCoord GCELLGRIDY = getGCELLGRIDY();
    frCoord GCELLOFFSETX = getGCELLOFFSETX();
    frCoord GCELLOFFSETY = getGCELLOFFSETY();

    llx = (pointIn.x() - GCELLOFFSETX) / GCELLGRIDX * GCELLGRIDX + GCELLOFFSETX;
    urx = llx + GCELLGRIDX;
    lly = (pointIn.y() - GCELLOFFSETY) / GCELLGRIDY * GCELLGRIDY + GCELLOFFSETY;
    ury = lly + GCELLGRIDY;

    frBox gCellBox(llx, lly, urx, ury);
    return gCellBox;

  }

  void updateXform(frTransform &xform, frPoint &size) {
    switch(xform.orient()) {
      //case frcR0: == default
      case frcR90:
        xform.set(xform.xOffset() + size.y(), xform.yOffset()           );
        break;
      case frcR180: // verified
        xform.set(xform.xOffset() + size.x(), xform.yOffset() + size.y());
        break;
      case frcR270:
        xform.set(xform.xOffset(),            xform.yOffset() + size.x());
        break;
      case frcMY: // verified
        xform.set(xform.xOffset() + size.x(), xform.yOffset()           );
        break;
      case frcMXR90:
        xform.set(xform.xOffset(),            xform.yOffset()           );
        break;
      case frcMX: // verified
        xform.set(xform.xOffset(),            xform.yOffset() + size.y());
        break;
      case frcMYR90:
        xform.set(xform.xOffset() + size.y(), xform.yOffset() + size.x());
        break;
      default      : // verified
        xform.set(xform.xOffset(),            xform.yOffset()           );
        break;
    }
  }

  // standardize guide to unit gcell width / height
  // void getStandardGuide(frCollection<frRect> &guidesIn,
  //                       frCollection<frRect> &guidesOut,
  //                       std::vector<std::shared_ptr<fr::frLayer> > layers) {
  //   std::map<frLayerNum, frCollection<frBox> > layer2BoxMap;
    
  //   frCoord GCELLGRIDX = getGCELLGRIDX();
  //   frCoord GCELLGRIDY = getGCELLGRIDY();
  //   frCoord GCELLOFFSETX = getGCELLOFFSETX();
  //   frCoord GCELLOFFSETY = getGCELLOFFSETY();

  //   // gen guide by layer map
  //   for (auto &guide: guidesIn) {
  //     frLayerNum layerNum = guide.getLayerNum();
  //     if (layer2BoxMap.find(layerNum) == layer2BoxMap.end()) {
  //       layer2BoxMap[layerNum] = frCollection<frBox>();
  //     }
  //     frBox guideBBox;
  //     guide.getBBox(guideBBox);
  //     layer2BoxMap[layerNum].push_back(guideBBox);
  //   }


  //   // process initial guide layer by layer
  //   for (auto mapIt = layer2BoxMap.begin(); mapIt != layer2BoxMap.end(); ++mapIt) {

  //   }

  // }

  void getStandardGuide(std::map<std::string, fr::frCollection<fr::frRect> > &tmpGuides,
                        std::vector<std::shared_ptr<fr::frLayer> > &vlayers,
                        std::map<std::string, std::shared_ptr<fr::frNet> > &nets,
                        std::map<std::string, std::map<frPoint, std::set<std::pair<frBlockObject*, fr::frLayerNum> > > > &gCell2PinMap) {
    bool debugMode = false;
    // debug
    // std::cout << "*************\n";
    // boostSegment seg1, seg2;
    // seg1 = boostSegment(boostPoint(28600, 415500), boostPoint(142600, 415500));
    // seg2 = boostSegment(boostPoint(115600, 406500), boostPoint(115600, 520500));
    // boost::geometry::correct(seg1);
    // boost::geometry::correct(seg2);
    // frCollection<boostPoint> intersectPoints;
    // bg::intersection(seg1, seg2, intersectPoints);
    // for (auto intersectPoint: intersectPoints) {
    //   frPoint tmpSteiner;
    //   tmpSteiner.set(bg::get<0>(intersectPoint), bg::get<1>(intersectPoint));
    //   std::cout << "(" << tmpSteiner.x() << ", " << tmpSteiner.y() << ")\n";
    // }

    // exit(0);
    if (VERBOSE > 0) {
      std::cout <<std::endl <<"get standard guide ..." <<std::endl;
    }
    int cnt = 0;
    for (auto guideIt = tmpGuides.begin(); guideIt != tmpGuides.end(); ++guideIt) {
      auto &netName = guideIt->first;
      auto &guides = guideIt->second;
      if (debugMode) {
        std::cout << "===============" << netName << "===============" << std::endl;
      }
      getStandardGuide(guides, vlayers, nets[netName], gCell2PinMap[netName]);
      cnt++;
      if (VERBOSE > 0) {
        if (cnt % 10000 == 0) {
          std::cout <<"  complete " <<cnt <<" nets" <<std::endl;
        }
      }
    }

    
  }

  void getStandardGuide(frCollection<frRect> &guideRects,
                        std::vector<std::shared_ptr<fr::frLayer> > &vlayers,
                        std::shared_ptr<fr::frNet> &net,
                        std::map<frPoint, std::set<std::pair<frBlockObject*, fr::frLayerNum> > > &gCell2PinMap) {
    bool enableOutput = false;
    frCoord GCELLGRIDX = getGCELLGRIDX();
    frCoord GCELLGRIDY = getGCELLGRIDY();
    frCoord GCELLOFFSETX = getGCELLOFFSETX();
    frCoord GCELLOFFSETY = getGCELLOFFSETY();
    frLayerNum layerNum;
    frPrefRoutingDir dir;
    std::map<frLayerNum, std::map<frCoord, boost::icl::interval_set<frCoord> > > layerGCell2IntvSet;
    // frCollection<set<int> > indexCluster;
    // frCollection<bool> visited(guideRects.size(), false);
    frString netName;
    net->getName(netName);
    //std::cout << netName << "\n";
    // get interval set per layer per GCell row
    for (int i = 0; i < (int)guideRects.size(); ++i) {
      frPoint ll, ur;
      frCoord llx, lly, urx, ury;
      layerNum = guideRects[i].getLayerNum();
      dir = vlayers[layerNum]->getDir();
      //if (layerGCell2IntvSet.find(layerNum) == layerGCell2IntvSet.end()) {
      //  layerGCell2IntvSet[layerNum] = std::map<frCoord, boost::icl::interval_set<int> >();
      //}
      // get lowerLeft and upperRight GCell centers
      frBox guideBox;
      guideRects[i].getBBox(guideBox);
      ll = getGCellCenter(frPoint(guideBox.left(), guideBox.bottom()));
      ur = getGCellCenter(frPoint(guideBox.right(), guideBox.top()));
      // modify if ur are on boundary
      if ((guideBox.right() - GCELLOFFSETX) % GCELLGRIDX == 0) {
        ur.set(ur.x() - GCELLGRIDX, ur.y());
      }
      if ((guideBox.top() - GCELLOFFSETY) % GCELLGRIDY == 0) {
        ur.set(ur.x(), ur.y() - GCELLGRIDY);
      }
      // insert interval map 
      llx = ll.x();
      lly = ll.y();
      urx = ur.x();
      ury = ur.y();
      if (enableOutput) {
        std::cout << "guideRectCenter (" << llx << ", " << lly << ") - (" << urx << ", " << ury << ") " << vlayers[layerNum]->getName() << "\n";
      }
      if (dir == frcHorzPrefRoutingDir) {
        for (frCoord gCellRowCenter = lly; gCellRowCenter <= ury; gCellRowCenter += GCELLGRIDY) {
          //if (layerGCell2IntvSet[layerNum].find(gCellRowCenter) == layerGCell2IntvSet[layerNum].end()) {
          //  layerGCell2IntvSet[layerNum][gCellRowCenter] = boost::icl::interval_set<int>();
          //}
          layerGCell2IntvSet[layerNum][gCellRowCenter].insert(boost::icl::interval_set<frCoord>::interval_type::closed(llx, urx));
        }
      } else if (dir == frcVertPrefRoutingDir) {
        for (frCoord gCellRowCenter = llx; gCellRowCenter <= urx; gCellRowCenter += GCELLGRIDX) {
          //if (layerGCell2IntvSet[layerNum].find(gCellRowCenter) == layerGCell2IntvSet[layerNum].end()) {
          //  layerGCell2IntvSet[layerNum][gCellRowCenter] = boost::icl::interval_set<int>();
          //}
          layerGCell2IntvSet[layerNum][gCellRowCenter].insert(boost::icl::interval_set<frCoord>::interval_type::closed(lly, ury));
        }
      }
    }

    appendTouchingGuideIntv(layerGCell2IntvSet, vlayers);


    if (enableOutput) {
      std::cout <<"layerGCell2IntvSet " <<std::endl;
      for (auto layerIt = layerGCell2IntvSet.begin(); layerIt != layerGCell2IntvSet.end(); ++layerIt) {
        auto layerNum = layerIt->first;
        auto &gCell2RowMap = layerIt->second;
        std::cout << "  layerNum = " << layerNum << std::endl;
        for (auto rowIt = gCell2RowMap.begin(); rowIt != gCell2RowMap.end(); ++rowIt) {
          auto gCellRowCenter = rowIt->first;
          auto &intvSet = rowIt->second;
          std::cout << "    gCellRowCenter = " << gCellRowCenter << std::endl;
          // std::cout << "      ";
          for (auto intvSetIt = intvSet.begin(); intvSetIt != intvSet.end(); ++intvSetIt) {
            std::cout << "      (" << intvSetIt->lower() << ", " << intvSetIt->upper() << ")\n";
          }
        }
      }
    }
    // collect steiner point / pin (vertex) to be created in the graph
    // along with the corresponding edges
    std::set<std::pair<frPoint, frLayerNum> > steinerSet;
    // std::set<std::pair<std::pair<frPoint, frLayerNum>, std::pair<frPoint, frLayerNum> >, guideSteinerCmp> steinerEdgeSet;
    std::set<std::pair<std::pair<frPoint, frLayerNum>, std::pair<frPoint, frLayerNum> > > steinerEdgeSet;
    std::map<std::pair<frPoint, frLayerNum>, frBlockObject*> steiner2Ptr;

    // layerGCell2IntvSet is layerNum --> (trackCoord --> intv set)
    for (auto layerIt = layerGCell2IntvSet.begin(); layerIt != layerGCell2IntvSet.end(); ++layerIt) {
      auto layerNum = layerIt->first;
      auto &gCell2RowMap = layerIt->second; // trackCoord --> intv set
      // auto layers = getLayers();
      // std::cout << layers->size() << std::endl;
      // auto dir = layers->at(layerNum)->getDir();
      auto dir = vlayers[layerNum]->getDir();
      for (auto rowIt = gCell2RowMap.begin(); rowIt != gCell2RowMap.end(); ++rowIt) {
        auto &gCellRowCenter = rowIt->first; // track Coord
        auto &intvSet = rowIt->second;
        boostSegment currBoostSeg;
        // iterate current layer gseg
        for (auto currIntvSetIt = intvSet.begin(); currIntvSetIt != intvSet.end(); ++currIntvSetIt) {
          if (dir == frcHorzPrefRoutingDir) {
            currBoostSeg = boostSegment(boostPoint(currIntvSetIt->lower(), gCellRowCenter), 
                                        boostPoint(currIntvSetIt->upper(), gCellRowCenter));
          } else if (dir == frcVertPrefRoutingDir) {
            currBoostSeg = boostSegment(boostPoint(gCellRowCenter, currIntvSetIt->lower()), 
                                        boostPoint(gCellRowCenter, currIntvSetIt->upper()));
          }
          // iterate lower layer gseg
          std::set<frPoint> lowerSteiners;
          frLayerNum lowerLayerNum = layerNum - 2;
          frLayerNum upperLayerNum = layerNum + 2;
          // std::map<frCoord, boost::icl::interval_set<frCoord> > lowerGCell2RowMap;
          // if (netName == "net107211" && lowerLayerNum == 4) {
          //   std::cout << "  debug1: currGCellRowCenter = " << gCellRowCenter << " (" 
          //             << currIntvSetIt->lower() << ", " << currIntvSetIt->upper() << ")\n";
          // }

          if (layerGCell2IntvSet.find(lowerLayerNum) != layerGCell2IntvSet.end()) {
            // if (netName == "net107211" && lowerLayerNum == 4) {
            //   for (auto setIt = layerGCell2IntvSet[lowerLayerNum].begin(); setIt != layerGCell2IntvSet[lowerLayerNum].end(); ++setIt) {
            //     std::cout << "    debug:\n";
            //     std::cout << "      lowerGCellRowCenter = " << setIt->first << std::endl;
            //     for (auto it = setIt->second.begin(); it != setIt->second.end(); ++it) {
            //       std::cout << "         (" << it->lower() << ", " << it->upper() << ")\n";
            //     }
            //   }
            // }
            getGSegSteiner(currBoostSeg, dir, layerGCell2IntvSet[lowerLayerNum], lowerSteiners);
          }
          // if (debugMode) {
          // if (netName == "net107211") {
          //   for (auto setIt = lowerSteiners.begin(); setIt != lowerSteiners.end(); ++setIt) {
          //     std::cout << "    lower steiner on layer " << layerNum << " at (" << setIt->x() << ", " << setIt->y() << ")\n";
          //   }
          // }
          for (auto setIt = lowerSteiners.begin(); setIt != lowerSteiners.end(); ++setIt) {
            getInterLayerSteinerEdge(*setIt, lowerLayerNum, layerNum, steinerSet, steinerEdgeSet);
          }

          // iterate upper layer gseg
          std::set<frPoint> upperSteiners;
          if (layerGCell2IntvSet.find(upperLayerNum) != layerGCell2IntvSet.end()) {
            getGSegSteiner(currBoostSeg, dir, layerGCell2IntvSet[upperLayerNum], upperSteiners);
          }
          // if (debugMode) {
          // if (netName == "net107211") {
          //   for (auto setIt = upperSteiners.begin(); setIt != upperSteiners.end(); ++setIt) {
          //     std::cout << "    upper steiner on layer " << layerNum << " at (" << setIt->x() << ", " << setIt->y() << ")\n";
          //   }
          // }
          for (auto setIt = upperSteiners.begin(); setIt != upperSteiners.end(); ++setIt) {
            getInterLayerSteinerEdge(*setIt, layerNum, upperLayerNum, steinerSet, steinerEdgeSet);
          }


          // seg steiners
          std::set<frPoint> segSteiners;
          segSteiners.insert(lowerSteiners.begin(), lowerSteiners.end());
          segSteiners.insert(upperSteiners.begin(), upperSteiners.end());
          
          frPoint start, end;
          if (dir == frcHorzPrefRoutingDir) {
            start = frPoint(currIntvSetIt->lower(), gCellRowCenter);
            end = frPoint(currIntvSetIt->upper(), gCellRowCenter);
          } else if (dir == frcVertPrefRoutingDir) {
            start = frPoint(gCellRowCenter, currIntvSetIt->lower());
            end = frPoint(gCellRowCenter, currIntvSetIt->upper());
          }
          // fix for term at segment endpoint gcells
          steinerSet.insert(std::make_pair(start, layerNum));
          steinerSet.insert(std::make_pair(end, layerNum));
          segSteiners.insert(start);
          segSteiners.insert(end);
          if (enableOutput) {
          
          // if (netName == "net107211") {

            std::cout << "Curr Seg: (" << start.x() << ", " << start.y() << ") -- ("
                      << end.x() << ", " << end.y() << ") layer " << layerNum << "\n";
            std::cout << "Steiners:\n";
            for (auto setIt = segSteiners.begin(); setIt != segSteiners.end(); ++setIt) {
              std::cout << "  (" << setIt->x() << ", " << setIt->y() << ")\n";
              // if (dir == frcHorzPrefRoutingDir) { 
              //   std::cout << "  (" << *setIt << ", " << gCellRowCenter <<")\n";
              // } else if (dir == frcVertPrefRoutingDir) {
              //   std::cout << "  (" << gCellRowCenter << ", " << *setIt <<")\n";
              // }
            }
          }

          // add current layer 
          if (!segSteiners.empty()) {
            for (auto setIt = segSteiners.begin(); std::next(setIt) != segSteiners.end(); ++setIt) {
              steinerEdgeSet.insert(std::make_pair(std::make_pair(*setIt, layerNum), std::make_pair(*(std::next(setIt)), layerNum)) );
            }
          }

        }
        
      }
    }

    // add steiner vertex 
    for (auto steinerIt = steinerSet.begin(); steinerIt != steinerSet.end(); ++steinerIt) {

      std::unique_ptr<frSteiner> steiner = std::make_unique<frSteiner>();
      steiner->setPoint(steinerIt->first);
      steiner->setLayerNum(steinerIt->second);
      steiner->addToNet(net);
      auto rptr = steiner.get();
      steiner2Ptr[*steinerIt] = rptr;
      net->addSteiner(steiner);
      net->addVertex(rptr);
      //std::cout << "Steiner: " << steinerIt->first << ", layer " << steinerIt->second << "\n";
    }
    // add edge between steiners
    for (auto edgeIt = steinerEdgeSet.begin(); edgeIt != steinerEdgeSet.end(); ++edgeIt) {
      auto &startSteiner = edgeIt->first;
      auto &endSteiner = edgeIt->second;
      std::shared_ptr<frGuideGlobal> edge = std::make_shared<frGuideGlobal>();
      edge->setPoints(startSteiner.first, endSteiner.first);
      edge->setBeginLayerNum(startSteiner.second);
      edge->setEndLayerNum(endSteiner.second);
      edge->addToNet(net);
      // TODO: may need to chagne from frRoute to frGuide
      // std::shared_ptr<frRoute> route = std::make_shared<frRoute>();
      // frCollection<std::shared_ptr<frConnFig> > tmpObjs;
      // tmpObjs.push_back(edge);
      // route->setObjects(tmpObjs);
      // route->addToNet(net);
      // std::cout << "edge: (" << startSteiner.first.x() << ", " << startSteiner.first.y() << ", " << startSteiner.second
      //           << "), ("    << endSteiner.first.x() << ", " << endSteiner.first.y() << ", " << endSteiner.second << "\n";
      //net->addEdge(steiner2Ptr[startSteiner], steiner2Ptr[endSteiner], route);
      net->addEdge(steiner2Ptr[startSteiner], steiner2Ptr[endSteiner], edge);

    }

    // temp solution: add pin Obj as vertex
    std::set<frBlockObject*> termInstTermSet;

    // add edge between steiner and term/instTerm
    for (auto pinGCellIt = gCell2PinMap.begin(); pinGCellIt != gCell2PinMap.end(); ++pinGCellIt) {
      auto &gCellCenter = pinGCellIt->first;
      for (auto setIt = pinGCellIt->second.begin(); setIt != pinGCellIt->second.end(); ++setIt) {
        auto pinObj = setIt->first;
        auto pinLayer = setIt->second;
        auto pinSteiner = std::make_pair(gCellCenter, pinLayer);
        if (steiner2Ptr.find(pinSteiner) != steiner2Ptr.end()) {
          //auto steinerPtr = steiner2Ptr[pinSteiner];
          std::shared_ptr<frGuideShortConn> edge = std::make_shared<frGuideShortConn>();
          edge->setPoints(pinSteiner.first, pinSteiner.first);
          edge->setBeginLayerNum(pinLayer);
          edge->setEndLayerNum(pinLayer);
          edge->addToNet(net);
          // TODO: may need to chagne from frRoute to frGuide
          // std::shared_ptr<frRoute> route = std::make_shared<frRoute>();
          // frCollection<std::shared_ptr<frConnFig> > tmpObjs;
          // tmpObjs.push_back(edge);
          // route->setObjects(tmpObjs);
          // route->addToNet(net);
          
          // temp solution: add pin Obj as vertex
          if (termInstTermSet.find(pinObj) == termInstTermSet.end()) {
            net->addVertex(pinObj);
            termInstTermSet.insert(pinObj);
          }

          //net->addEdge(pinObj, steiner2Ptr[pinSteiner], edge);
          net->addEdge(steiner2Ptr[pinSteiner], pinObj, edge);

        } else {
          // TODO: may need to append route guide
          // std::cout << "Warning: Part of pin not covered by GR solution.\n";
        }
      }


    }


  }

  void getInterLayerSteinerEdge(frPoint steiner, 
                      frLayerNum lowerLayerNum, 
                      frLayerNum upperLayerNum,
                      std::set<std::pair<frPoint, frLayerNum> > &steinerSet,
                      // std::set<std::pair<std::pair<frPoint, frLayerNum>, std::pair<frPoint, frLayerNum> >, guideSteinerCmp> &steinerEdgeSet) {
                      std::set<std::pair<std::pair<frPoint, frLayerNum>, std::pair<frPoint, frLayerNum> > > &steinerEdgeSet) {
    std::pair<frPoint, frLayerNum> upperSteiner, lowerSteiner;
    lowerSteiner = std::make_pair(steiner, lowerLayerNum);
    upperSteiner = std::make_pair(steiner, upperLayerNum);
    steinerSet.insert(lowerSteiner);
    steinerSet.insert(upperSteiner);
    auto steinerEdge = std::make_pair(lowerSteiner, upperSteiner);
    steinerEdgeSet.insert(steinerEdge);
  }

  void getGSegSteiner(boostSegment &currGSeg,
                          frPrefRoutingDir &currDir,
                          std::map<frCoord, boost::icl::interval_set<frCoord> > &neighborGCell2RowMap,
                          std::set<frPoint> &steiners) {
    boostSegment neighborGSeg;
    for (auto rowIt = neighborGCell2RowMap.begin(); rowIt != neighborGCell2RowMap.end(); ++rowIt) {
      auto &neighborGCellRowCenter = rowIt->first;
      auto &neightborIntvSet = rowIt->second;
      for (auto neightborIntvSetIt = neightborIntvSet.begin(); neightborIntvSetIt != neightborIntvSet.end(); ++neightborIntvSetIt) {
        if (currDir == frcHorzPrefRoutingDir) {
          neighborGSeg = boostSegment(boostPoint(neighborGCellRowCenter, neightborIntvSetIt->lower()),
                                      boostPoint(neighborGCellRowCenter, neightborIntvSetIt->upper()));
        } else if (currDir == frcVertPrefRoutingDir) {
          neighborGSeg = boostSegment(boostPoint(neightborIntvSetIt->lower(), neighborGCellRowCenter),
                                      boostPoint(neightborIntvSetIt->upper(), neighborGCellRowCenter));
        }
        // if (neighborGCellRowCenter == 115600) {
        //   std::cout << neighborGCellRowCenter << "\n";
        // }
        // TODO: not sure whether intersects is buggy or not
        if (bg::intersects(currGSeg, neighborGSeg)) {
          // BOOST INTERSECTION IS BUGGY!!!
          frCollection<boostPoint> intersectPoints;
          bg::intersection(currGSeg, neighborGSeg, intersectPoints);
          // for (auto intersectPoint: intersectPoints) {
          //   frPoint tmpSteiner;
          //   tmpSteiner.set(bg::get<0>(intersectPoint), bg::get<1>(intersectPoint));
          //   steiners.insert(tmpSteiner);
          // }
          frPoint boostSteiner;
          boostSteiner.set(bg::get<0>(intersectPoints[0]), bg::get<1>(intersectPoints[0]));
          frPoint tmpSteiner;
          if (currDir == frcHorzPrefRoutingDir) {
            tmpSteiner.set(neighborGCellRowCenter, bg::get<0,1>(currGSeg));
          } else if (currDir == frcVertPrefRoutingDir) {
            tmpSteiner.set(bg::get<0,0>(currGSeg), neighborGCellRowCenter);
          }
          // if (tmpSteiner.x() != boostSteiner.x() || tmpSteiner.y() != boostSteiner.y()) {
          //   std::cout << "Warning: inconsistent restlt from Boost\n";
          //   std::cout << "Boost (" << boostSteiner.x() << ", " << boostSteiner.y() 
          //             << ") and our (" << tmpSteiner.x() << ", " << tmpSteiner.y() << ")\n";
          // }
          steiners.insert(tmpSteiner);

        }
      }
    }
  }

  void appendTouchingGuideIntv(std::map<frLayerNum, std::map<frCoord, boost::icl::interval_set<frCoord> > > &layerGCell2IntvSet,
                               std::vector<std::shared_ptr<fr::frLayer> > &vlayers) {
    frCoord GCELLGRIDX = getGCELLGRIDX();
    frCoord GCELLGRIDY = getGCELLGRIDY();
    //frCoord GCELLOFFSETX = getGCELLOFFSETX();
    //frCoord GCELLOFFSETY = getGCELLOFFSETY();

    for (auto layerIt = layerGCell2IntvSet.begin(); layerIt != layerGCell2IntvSet.end(); ++layerIt) {
      auto currLayer = layerIt->first;
      auto currDir = vlayers[currLayer]->getDir();
      auto &gCell2IntvSet = layerIt->second;
      frLayerNum nextLayer = -1;
      if ((currLayer + 2) < (int)vlayers.size()) {
        nextLayer = currLayer + 2;
      } else if ((currLayer - 2) >= 0) {
        nextLayer = currLayer - 2;
      }
      if (nextLayer == -1) {
        continue;
      }
      frCoord gCellTrackPitch = (currDir == frcHorzPrefRoutingDir) ? GCELLGRIDY : GCELLGRIDX;
      //frCoord gCellTrackPitchNext = (currDir == frcHorzPrefRoutingDir) ? GCELLGRIDX : GCELLGRIDY;
      // std::cout << "gCellTrackPitchNext = " << gCellTrackPitchNext << ", gCellTrackPitch = " << gCellTrackPitch << "\n";
      if (gCell2IntvSet.empty()) {
        continue;
      } else {
        for (auto gCellRowIt = gCell2IntvSet.begin(); std::next(gCellRowIt) != gCell2IntvSet.end(); ++gCellRowIt) {
          frCoord currRow = gCellRowIt->first;
          frCoord nextRow = (std::next(gCellRowIt))->first;
          auto &currIntvSet = gCellRowIt->second;
          auto &nextIntvSet = (std::next(gCellRowIt))->second;
          if ((nextRow - currRow) != gCellTrackPitch) {
            continue;
          }

          // check and get append guide intv;
          auto overlapIntvSet = currIntvSet & nextIntvSet;
          // std::cout << "overlapped intvSet size = " << overlapIntvSet.size() << std::endl;
          for (auto overlapIntvSetIt = overlapIntvSet.begin(); overlapIntvSetIt != overlapIntvSet.end(); ++overlapIntvSetIt) {
            bool needBridge = true;
            auto nextLayerStartRow = overlapIntvSetIt->lower();
            auto nextLayerEndRow = overlapIntvSetIt->upper();
            // std::cout << nextLayerStartRow << " " << nextLayerEndRow << "\n";
            // if (nextLayerStartRow != nextLayerEndRow) {
            //   std::cout << "Warning: parallel touching guide acrossing " << (nextLayerEndRow - nextLayerStartRow) / gCellTrackPitchNext + 1 << " gcells\n";

            // }
            // for (auto nextLayerRow = nextLayerStartRow; nextLayerRow <= nextLayerEndRow; nextLayerRow += gCellTrackPitchNext) {
            //   layerGCell2IntvSet[nextLayer][nextLayerRow].insert(boost::icl::interval_set<frCoord>::interval_type::closed(currRow, nextRow));
            // }

            // check if bridge already exists
            for (auto nextLayerRow = nextLayerStartRow; nextLayerRow <= nextLayerEndRow; ++nextLayerRow) {
              if (currLayer + 2 < (int)vlayers.size()) {
                frLayerNum tmpNextLayer = currLayer + 2;
                if (layerGCell2IntvSet.find(tmpNextLayer) != layerGCell2IntvSet.end()) {
                  if (layerGCell2IntvSet[tmpNextLayer].find(nextLayerRow) != layerGCell2IntvSet[tmpNextLayer].end()) {
                    // std::cout << "here\n";
                    auto bridgeIntv = boost::icl::interval_set<frCoord>::interval_type::closed(currRow, nextRow);
                    boost::icl::interval_set<frCoord> bridgeIntvSet;
                    bridgeIntvSet.insert(bridgeIntv);
                    auto bridgeIntersectIntvSet = bridgeIntvSet & layerGCell2IntvSet[tmpNextLayer][nextLayerRow];
                    for (auto it = bridgeIntersectIntvSet.begin(); it != bridgeIntersectIntvSet.end(); ++it) {
                      if (*it == bridgeIntv) {
                        needBridge = false;
                        // std::cout << "existing upper bridge\n";
                        break;
                      }
                    }
                    // if (!bridgeIntersectIntvSet.empty() && bridgeIntersectIntvSet.size() == 1 && *(bridgeIntersectIntvSet.begin()) == bridgeIntv) {
                    //   std::cout << "bridge exists\n";
                    //   needBridge = false;
                    //   break;
                    // }
                  }
                }
                

              }
              if (currLayer - 2 >= 0) {
                frLayerNum tmpNextLayer = currLayer - 2;

                if (layerGCell2IntvSet.find(tmpNextLayer) != layerGCell2IntvSet.end()) {
                  if (layerGCell2IntvSet[tmpNextLayer].find(nextLayerRow) != layerGCell2IntvSet[tmpNextLayer].end()) {
                    auto bridgeIntv = boost::icl::interval_set<frCoord>::interval_type::closed(currRow, nextRow);
                    boost::icl::interval_set<frCoord> bridgeIntvSet;
                    bridgeIntvSet.insert(bridgeIntv);
                    auto bridgeIntersectIntvSet = bridgeIntvSet & layerGCell2IntvSet[tmpNextLayer][nextLayerRow];
                    for (auto it = bridgeIntersectIntvSet.begin(); it != bridgeIntersectIntvSet.end(); ++it) {
                      if (*it == bridgeIntv) {
                        needBridge = false;
                        // std::cout << "existing lower bridge\n";
                        break;
                      }
                    }
                    // if (!bridgeIntersectIntvSet.empty() && bridgeIntersectIntvSet.size() == 1 && *(bridgeIntersectIntvSet.begin()) == bridgeIntv) {
                    //   needBridge = false;
                    //   std::cout << "bridge exists\n";
                    //   break;
                    // }
                  }
                }

              }   
            }

            if (needBridge) {
              layerGCell2IntvSet[nextLayer][nextLayerStartRow].insert(boost::icl::interval_set<frCoord>::interval_type::closed(currRow, nextRow));
            }

          }
        }
      }
    }
  }


  void checkGRConnectivity(std::map<std::string, std::shared_ptr<fr::frNet> > &nets) {
    bool enableOutput = false;
    std::cout << "Checking GR connectivity\n";
    for (auto &net: nets) {
      frString netName;
      net.second->getName(netName);
      
      vertex_descriptor_map_t idxMap;
      boost::associative_property_map<vertex_descriptor_map_t> indexMap(idxMap);
      vertex_iterator_t di, dj;
      boost::tie(di, dj) = boost::vertices(net.second->g);
      for(int i = 0; di != dj; ++di,++i){
          boost::put(indexMap, (*di), i);
      }
      std::map<vertex_descriptor_t, size_t> compMap;
      boost::associative_property_map<vertex_descriptor_map_t> componentMap(compMap);            
      auto num_comps = boost::connected_components(net.second->g, componentMap, boost::vertex_index_map(indexMap)); 
      
      // std::cout << netName << std::endl;
      if (enableOutput) {
        std::cout << netName << ": graph info: num_v = " <<boost::num_vertices(net.second->g) 
             <<", num_e = " <<boost::num_edges(net.second->g) 
             <<", num_parts = " <<num_comps << std::endl;
      }
      // if (boost::num_vertices(net.second->g) == 0) {
      //   std::cout <<"Warning: " <<netName <<" has no wires" << std::endl;
      // }
      // if (boost::num_vertices(net.second->g) > 0 && 
      //     boost::num_vertices(net.second->g) - boost::num_edges(net.second->g) != 1) {
      //   std::cout <<"Warning: " <<netName <<" has redundant wires" << std::endl;
      // }
      if (boost::num_vertices(net.second->g) > 0 && num_comps != 1) {
        std::cout <<"Warning: " <<netName <<" is open and it has " << num_comps << " parts" << std::endl;
        std::map<size_t, int> ccCnt;
        for (auto mapIt = compMap.begin(); mapIt != compMap.end(); ++mapIt) {
          auto ccIdx = mapIt->second;
          if (ccCnt.find(ccIdx) == ccCnt.end()) {
            ccCnt[ccIdx] = 0;
          }
          ccCnt[ccIdx]++;
        }

        for (auto setIt = ccCnt.begin(); setIt != ccCnt.end(); ++setIt) {
          std::cout << "  ccIdx = " << setIt->first << ", count = " << setIt->second << "\n";
        }

        // debug
        //if (netName == "net107211") {
        //  std::map<size_t, std::set<vertex_descriptor_t> > ccVertex;
        //  for (auto mapIt = compMap.begin(); mapIt != compMap.end(); ++mapIt) {
        //    auto ccIdx = mapIt->second;
        //    ccVertex[ccIdx].insert(mapIt->first);
        //  }

        //  for (auto mapIt = ccVertex.begin(); mapIt != ccVertex.end(); ++mapIt) {
        //    auto ccIdx = mapIt->first;
        //    std::cout << "====CCIdx " << ccIdx << "====\n";
        //    int tmpCnt = 0;
        //    for (auto vertexIt = mapIt->second.begin(); vertexIt != mapIt->second.end(); ++vertexIt) {
        //      auto vertexPtr = net.second->g[*vertexIt].objPtr;
        //      std::cout << "location " << tmpCnt << ": ";
        //      if (vertexPtr->typeId() == frcInstTerm) {
        //        auto pins = std::dynamic_pointer_cast<frInstTerm>(vertexPtr)->getTerm()->getPins();
        //        auto pinFigs = pins[0]->getFigs();
        //        frString instName;
        //        std::dynamic_pointer_cast<frInstTerm>(vertexPtr)->getInst()->getName(instName);
        //        frBox pinBox;
        //        pinFigs[0]->getBBox(pinBox);
        //        std::cout << "(" << pinBox.left() << ", " << pinBox.bottom() << ") of inst " << instName << "\n";
        //      } else if (vertexPtr->typeId() == frcTerm) {
        //        auto pins = std::dynamic_pointer_cast<frTerm>(vertexPtr)->getPins();
        //        auto pinFigs = pins[0]->getFigs();
        //        frBox pinBox;
        //        pinFigs[0]->getBBox(pinBox);
        //        std::cout << "(" << pinBox.left() << ", " << pinBox.bottom() << ")\n";
        //      } else if (vertexPtr->typeId() == frcSteiner) {
        //        auto steiner = std::dynamic_pointer_cast<frSteiner>(vertexPtr);
        //        frBox steinerBox;
        //        steiner->getBBox(steinerBox);
        //        std::cout << "(" << steinerBox.left() << ", " << steinerBox.bottom() << ", " << steiner->getLayerNum() << ")\n";
        //      }
        //      tmpCnt++;

        //    }
        //  }

        //}

      }

    }
  }

  frPoint getGCellCenter(frPoint pointIn) {
    frCoord centerX, centerY;
    frPoint pointOut;
    frCoord GCELLGRIDX = getGCELLGRIDX();
    frCoord GCELLGRIDY = getGCELLGRIDY();
    frCoord GCELLOFFSETX = getGCELLOFFSETX();
    frCoord GCELLOFFSETY = getGCELLOFFSETY();

    centerX = (pointIn.x() - GCELLOFFSETX) / GCELLGRIDX * GCELLGRIDX + GCELLOFFSETX + GCELLGRIDX / 2;
    centerY = (pointIn.y() - GCELLOFFSETY) / GCELLGRIDY * GCELLGRIDY + GCELLOFFSETY + GCELLGRIDY / 2;
    pointOut.set(centerX, centerY);
    return pointOut;


  }

  void printGCellInfo() {
    std::cout << "GCELLGRIDX = " << getGCELLGRIDX() << std::endl;
    std::cout << "GCELLGRIDY = " << getGCELLGRIDY() << std::endl;
    std::cout << "GCELLOFFSETX = " << getGCELLOFFSETX() << std::endl;
    std::cout << "GCELLOFFSETY = " << getGCELLOFFSETY() << std::endl;

  }

  void getLocalNetStats(const std::map<std::string, std::map<frPoint, std::set<std::pair<std::shared_ptr<frBlockObject>, frLayerNum> > > > &gCell2PinMap) {
    std::cout << "Getting local net stats...\n";
    std::set<std::string> netNames;
    std::set<frPoint> gCellCenters;
    int totLocalNetCnt = 0;
    for (auto netIt = gCell2PinMap.begin(); netIt != gCell2PinMap.end(); ++netIt) {
      auto netName = netIt->first;
      netNames.insert(netName);
      auto &gCellMap = netIt->second;
      for (auto gCellIt = gCellMap.begin(); gCellIt != gCellMap.end(); ++gCellIt) {
        auto gCellCenter = gCellIt->first;
        auto &pins = gCellIt->second;
        gCellCenters.insert(gCellCenter);
        if (pins.size() >= 2) {
          totLocalNetCnt++;
        }

      }
    }

    std::cout << "#local net = " << totLocalNetCnt 
              << ", among #total net = " << netNames.size() 
              << ", in #non-empty gcell = " << gCellCenters.size() << "\n"; 
    
  }


  void KMBTermSteinerTree(frNet &net,
                          graph_t &netG,
                          const std::shared_ptr<frCMap> &cMap,
                          std::set<std::shared_ptr<frConnFig> > &routes) {
    bool enableOutput = false;
    if (enableOutput) {
      std::cout << "net: " << net.getName() << std::endl << std::flush;
    }
    //if (net.getName() == "rtr/rof_0__bmrb/fifo_data[201]") {
    // return;
    //}
    //if (net.getName() == "rtr/rof_0__bmrb/fifo_data[217]") {
    // return;
    //}

    vertex_descriptor_map_t idxMap;
    boost::associative_property_map<vertex_descriptor_map_t> indexMap(idxMap);
    vertex_iterator_t di, dj;
    // boost::tie(di, dj) = boost::vertices(net.g);
    boost::tie(di, dj) = boost::vertices(netG);
    for(int i = 0; di != dj; ++di,++i){
      boost::put(indexMap, (*di), i);
    }
    std::map<vertex_descriptor_t, size_t> compMap;
    boost::associative_property_map<vertex_descriptor_map_t> componentMap(compMap);            
    // auto num_comps = boost::connected_components(net.g, componentMap, boost::vertex_index_map(indexMap)); 
    auto num_comps = boost::connected_components(netG, componentMap, boost::vertex_index_map(indexMap)); 
    if (num_comps != 1) {
      return;
    }
    
    
    int numVertex = boost::num_vertices(netG);
    double LARGEDOUBLE = std::numeric_limits<double>::max() / numVertex;
    std::vector<std::pair<boost::graph_traits<KMBGraph>::vertex_descriptor, boost::graph_traits<KMBGraph>::vertex_descriptor > > pathEdges;
    std::vector<double> pathWeights;
    KMBGraph g;
    boost::property_map<KMBGraph, boost::edge_weight_t>::type weightmap = boost::get(boost::edge_weight, g);
    vertex_iterator_t v, v_end;
    std::map<frBlockObject*, boost::graph_traits<KMBGraph>::vertex_descriptor> iteratorMap;
    if (numVertex <= 1) {
      return;
    }

    // add vertex
    for (boost::tie(v, v_end) = boost::vertices(netG); v != v_end; ++v) {
      auto tmpV = boost::add_vertex(g);
      g[tmpV].objPtr = netG[*v].objPtr;
      iteratorMap[netG[*v].objPtr] = tmpV;
    }
    // add edge
    for (boost::tie(v, v_end) = boost::vertices(netG); v != v_end; ++v) {
      adjacency_iterator_t adj_v, adj_v_end;
      for (tie(adj_v, adj_v_end) = boost::adjacent_vertices(*v, netG); adj_v != adj_v_end; ++adj_v) {
        double edgeWeight = getKMBEdgeWeight(netG[*v].objPtr, netG[*adj_v].objPtr, cMap);
        boost::add_edge( (iteratorMap[netG[*v].objPtr]), (iteratorMap[netG[*adj_v].objPtr]), edgeWeight, g);
      } 
    }
    // KMB initialization
    std::vector<boost::graph_traits < KMBGraph >::vertex_descriptor> predecessors(num_vertices(g));
    std::vector<double> distances(num_vertices(g));
    std::vector<std::vector<boost::graph_traits < KMBGraph >::vertex_descriptor> > paths;
    std::map<std::pair<boost::graph_traits<KMBGraph>::vertex_descriptor, boost::graph_traits<KMBGraph>::vertex_descriptor >, int > gEdge2PathIdxMap;
    // shortest path from each terminal to others
    boost::graph_traits< KMBGraph >::vertex_iterator term_v, term_v_end;
    int terminalCnt = 0;
    for (boost::tie(term_v, term_v_end) = boost::vertices(g); term_v != term_v_end; ++term_v) {
      if ((g[*term_v].objPtr)->typeId() == frcSteiner) {
        continue;
      } else {
        terminalCnt++;
        boost::dijkstra_shortest_paths(g, *term_v,
                                       boost::predecessor_map(boost::make_iterator_property_map(predecessors.begin(), boost::get(boost::vertex_index, g))).
                                       distance_map(boost::make_iterator_property_map(distances.begin(), boost::get(boost::vertex_index, g))));
        boost::graph_traits < KMBGraph >::vertex_iterator vi, vend;
        for (boost::tie(vi, vend) = boost::vertices(g); vi != vend; ++vi) {
          if (*vi == *term_v || (g[*vi].objPtr)->typeId() == frcSteiner) {
            continue;
          }
          std::vector< boost::graph_traits<KMBGraph>::vertex_descriptor > path;
          boost::graph_traits<KMBGraph>::vertex_descriptor current, end;
          double pathWeight = distances[*vi];
          end = *vi;
          current = end;
          while (current != *term_v) {
            //std::cout << "here1\n" << std::flush;
            path.push_back(current);
            current = predecessors[current];
          }
          path.push_back(current);
          pathEdges.push_back(std::make_pair(path.front(), path.back()));
          pathWeights.push_back(pathWeight);
          gEdge2PathIdxMap[std::make_pair(std::min(path.front(), path.back()), std::max(path.front(), path.back()))] = (int)paths.size();
          paths.push_back(path);
        }
      }
    }

    // generate pathgraph
    KMBGraph pathG(pathEdges.begin(), pathEdges.end(), pathWeights.begin(), terminalCnt);
    // std::vector<boost::graph_traits<KMBGraph>::vertex_descriptor> pathP(num_vertices(pathG));
    // std::vector<double> pathD(num_vertices(pathG));
    // std::vector<std::vector< boost::graph_traits< KMBGraph >::vertex_descriptor > > pathPaths;
    std::vector<boost::graph_traits < KMBGraph >::edge_descriptor> spanning_tree;
    std::set<std::pair<boost::graph_traits<KMBGraph>::vertex_descriptor, boost::graph_traits<KMBGraph>::vertex_descriptor > > mstEdgeSet;
    // generate mst from pathGraph
    boost::kruskal_minimum_spanning_tree(pathG, back_inserter(spanning_tree));
    for (auto ei = spanning_tree.begin(); ei != spanning_tree.end(); ++ei) {
      auto u = source(*ei, pathG);
      auto v = target(*ei, pathG);
      std::pair<boost::graph_traits<KMBGraph>::vertex_descriptor, boost::graph_traits<KMBGraph>::vertex_descriptor >  tempEdge = std::make_pair(std::min(u, v), std::max(u, v));
      for (int i = 0; i < (int)paths[gEdge2PathIdxMap[tempEdge]].size() - 1; i++) {
        auto u = paths[gEdge2PathIdxMap[tempEdge]][i];
        auto v = paths[gEdge2PathIdxMap[tempEdge]][i + 1];
        mstEdgeSet.insert(std::make_pair(std::min(u, v), std::max(u, v)));
      }
    }
    // Iterate all edges in the original graph, set weight to infinity if 
    // it is not in msgEdgeSet
    auto es = boost::edges(g);
    for (auto eit = es.first; eit != es.second; eit++) {
      auto u = source(*eit, g);
      auto v = target(*eit, g);
      auto tempEdge = std::make_pair(std::min(u,v), std::max(u,v));
      if (mstEdgeSet.find(tempEdge) == mstEdgeSet.end()) {
        weightmap[*eit] = LARGEDOUBLE;
      }
    }
    // generate mst in updated graph
    //boost::graph_traits<KMBGraph>::vertex_descriptor firstTermVertex;
    //boost::graph_traits<KMBGraph>::vertex_descriptor firstVertex;

    //boost::graph_traits < KMBGraph >::vertex_iterator vi, vend;
    //for (boost::tie(vi, vend) = boost::vertices(g); vi != vend; ++vi) {
    //  if (std::dynamic_pointer_cast<frSteiner>(g[*vi].objPtr)) {
    //    continue;
    //  }
    //  firstTermVertex = *vi;
    //  break;
    //}
    //for (boost::tie(vi, vend) = boost::vertices(g); vi != vend; ++vi) {
    //  firstVertex = *vi;
    //  break;
    //}

    // std::vector<boost::graph_traits < KMBGraph >::vertex_descriptor> resultPred(num_vertices(g));
    std::vector<boost::graph_traits < KMBGraph >::edge_descriptor> result_spanning_tree;
    boost::kruskal_minimum_spanning_tree(g, back_inserter(result_spanning_tree));
    // std::cout << "result\n";
    for (auto ei = result_spanning_tree.begin(); ei != result_spanning_tree.end(); ++ei) {
      auto u = source(*ei, g);
      auto v = target(*ei, g);
      // for (int i = 0; i < (int)paths[gEdge2PathIdxMap[tempEdge]].size() - 1; i++) {
        // auto u = paths[gEdge2PathIdxMap[tempEdge]][i];
        // auto v = paths[gEdge2PathIdxMap[tempEdge]][i + 1];
        // mstEdgeSet.insert(std::make_pair(std::min(u, v), std::max(u, v)));
      // std::cout << "  " << u << ", " << v << "\n";
      // }
      auto edge = boost::edge(u, v, g).first;
      if (weightmap[edge] != LARGEDOUBLE) {
        auto netG_u = (g[u].objPtr)->getVertexDescriptor();
        auto netG_v = (g[v].objPtr)->getVertexDescriptor();
        if (boost::edge(netG_u, netG_v, netG).second == true) {
          edge_descriptor_t routeEdge = (boost::edge(netG_u, netG_v, netG).first);
          routes.insert(netG[routeEdge].objPtr);
        }
      }
    }


  }

  double getKMBEdgeWeight(frBlockObject* start,
                          frBlockObject* end,
                          const std::shared_ptr<frCMap> &cMap) {
    //return 1;

    frCoord GCELLGRIDX = getGCELLGRIDX();
    frCoord GCELLGRIDY = getGCELLGRIDY();
    //frCoord GCELLOFFSETX = getGCELLOFFSETX();
    //frCoord GCELLOFFSETY = getGCELLOFFSETY();
    double weight = 0.0;
    double LARGEWEIGHT = 99;
    double SMALLWEIGHT = 1;
    double VIAWEIGHT = 10;
    frLayerNum startLayerNum, endLayerNum;
    // Since every terminal has to be connected, so set distance 
    // from edge to them to zero assuming each term is connected
    // to the steiner of the GCell that the terminal belongs to
    // p.s., potentially a pin can have access point on different 
    // layers, prefer access point on higher layer than lower layer
    if (start->typeId() == frcInstTerm ||
        start->typeId() == frcTerm || 
        end->typeId() == frcInstTerm || 
        end->typeId() == frcTerm) {
      weight = SMALLWEIGHT;
      return weight;
    }

    // for steiner to steiner, get the summation of track utilization 
    // along the direct edge
    frPoint startPoint, endPoint;
    if (start->typeId() == frcSteiner) {
      static_cast<frSteiner*>(start)->getPoint(startPoint);
      startLayerNum = static_cast<frSteiner*>(start)->getLayerNum();
    } else {
      // frBlockObject is none of Term, InstTerm or Steiner
      return LARGEWEIGHT;
    }
    if (end->typeId() == frcSteiner) {
      static_cast<frSteiner*>(end)->getPoint(endPoint);
      endLayerNum = static_cast<frSteiner*>(end)->getLayerNum();
    } else {
      return LARGEWEIGHT;
    }

    if (startPoint.x() != endPoint.x() && startPoint.y() != endPoint.y()) {
      return LARGEWEIGHT;
    }

    // start sum up the congestion along the line;
    // via guide
    if (startPoint.x() == endPoint.x() && startPoint.y() == endPoint.y()) {
      if (startLayerNum == endLayerNum) {
        return SMALLWEIGHT;
      } else {
        return VIAWEIGHT;
      }
    } else {
      // assuming that both startPoint and endPoint are center of gcell
      if (startPoint.x() == endPoint.x()) {
        auto startY = std::min(startPoint.y(), endPoint.y());
        auto endY = std::max(startPoint.y(), endPoint.y());
        for (auto yCoord = startY; yCoord <= endY; yCoord += GCELLGRIDY) {
          weight += getGCellCongestionScore(frPoint(startPoint.x(), yCoord), startLayerNum, cMap);
        }
      } else if (startPoint.y() == endPoint.y()) {
        auto startX = std::min(startPoint.x(), endPoint.x());
        auto endX = std::max(startPoint.x(), endPoint.x());
        for (auto xCoord = startX; xCoord <= endX; xCoord += GCELLGRIDX) {
          weight += getGCellCongestionScore(frPoint(xCoord, startPoint.y()), startLayerNum, cMap);
        }
      }
    }

    return weight;


    // return 1.0;
  }

  double getGCellCongestionScore(frPoint point, frLayerNum &layerNum, const std::shared_ptr<frCMap> &cMap) {
    double congestionScore = 0.0;

    frCoord GCELLGRIDX = getGCELLGRIDX();
    frCoord GCELLGRIDY = getGCELLGRIDY();
    frCoord GCELLOFFSETX = getGCELLOFFSETX();
    frCoord GCELLOFFSETY = getGCELLOFFSETY();

    int xIndex = (point.x() - GCELLOFFSETX) / GCELLGRIDX;
    xIndex = (xIndex > (int)cMap->getNumX()) ? cMap->getNumX() : xIndex;
    int yIndex = (point.y() - GCELLOFFSETY) / GCELLGRIDY;
    yIndex = (yIndex > (int)cMap->getNumY()) ? cMap->getNumY() : yIndex;

    auto supply = cMap->getSupply(xIndex, yIndex, layerNum);
    auto edge1Demand = cMap->getEdge1Demand(xIndex, yIndex, layerNum);
    auto edge2Demand = cMap->getEdge2Demand(xIndex, yIndex, layerNum);
    auto localDemand = cMap->getLocalDemand(xIndex, yIndex, layerNum);
    auto throughDemand = cMap->getThroughDemand(xIndex, yIndex, layerNum);

    congestionScore = 1.0 * (throughDemand + (edge1Demand + edge2Demand) / 2.0 + localDemand) / supply;

    return congestionScore;


  }

}

