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

#include "dr/FlexDR.h"
#include "drc/frDRC.h"
#include <chrono>

using namespace std;
using namespace fr;

inline frCoord FlexDRWorker::pt2boxDistSquare(const frPoint &pt, const frBox &box) {
  frCoord dx = max(max(box.left()   - pt.x(), pt.x() - box.right()), 0);
  frCoord dy = max(max(box.bottom() - pt.y(), pt.y() - box.top()),   0);
  return dx * dx + dy * dy;
}

inline frCoord FlexDRWorker::box2boxDistSquare(const frBox &box1, const frBox &box2, frCoord &dx, frCoord &dy) {
  dx = max(max(box1.left(), box2.left())     - min(box1.right(), box2.right()), 0);
  dy = max(max(box1.bottom(), box2.bottom()) - min(box1.top(), box2.top()),     0);
  return dx * dx + dy * dy;
}


/*inline*/ void FlexDRWorker::modMinSpacingCostPlaner(drNet* net, const frBox &box, frMIdx z, bool isAddPathCost) {
  auto lNum = gridGraph.getLayerNum(z);
  // obj1 = curr obj
  frCoord width1  = box.width();
  frCoord length1 = box.length();
  // obj2 = other obj
  // layer default width
  frCoord width2     = getDesign()->getTech()->getLayer(lNum)->getWidth();
  frCoord halfwidth2 = width2 / 2;
  // spacing value needed
  frCoord bloatDist = 0;
  auto con = getDesign()->getTech()->getLayer(lNum)->getMinSpacing();
  if (con) {
    if (con->typeId() == frConstraintTypeEnum::frcSpacingConstraint) {
      bloatDist = static_cast<frSpacingConstraint*>(con)->getMinSpacing();
    } else if (con->typeId() == frConstraintTypeEnum::frcSpacingTablePrlConstraint) {
      bloatDist = static_cast<frSpacingTablePrlConstraint*>(con)->find(max(width1, width2), length1);
    } else if (con->typeId() == frConstraintTypeEnum::frcSpacingTableTwConstraint) {
      bloatDist = static_cast<frSpacingTableTwConstraint*>(con)->find(width1, width2, length1);
    } else {
      cout <<"Warning: min spacing rule not supporterd" <<endl;
      return;
    }
  } else {
    cout <<"Warning: no min spacing rule" <<endl;
    return;
  }
  frCoord bloatDistSquare = bloatDist * bloatDist;

  FlexMazeIdx mIdx1;
  FlexMazeIdx mIdx2;
  // assumes width always > 2
  frBox bx(box.left()   - bloatDist - halfwidth2 + 1, box.bottom() - bloatDist - halfwidth2 + 1,
           box.right()  + bloatDist + halfwidth2 - 1, box.top()    + bloatDist + halfwidth2 - 1);
  gridGraph.getIdxBox(mIdx1, mIdx2, bx);
  //if (!isInitDR()) {
  //  cout <<" box " <<box <<" bloatDist " <<bloatDist <<" bx " <<bx <<endl;
  //  cout <<" midx1/2 (" <<mIdx1.x() <<", " <<mIdx1.y() <<") ("
  //                      <<mIdx2.x() <<", " <<mIdx2.y() <<") (" <<endl;
  //}

  frPoint pt, pt1, pt2, pt3, pt4;
  frCoord distSquare = 0;
  int cnt = 0;
  for (int i = mIdx1.x(); i <= mIdx2.x(); i++) {
    for (int j = mIdx1.y(); j <= mIdx2.y(); j++) {
      gridGraph.getPoint(pt, i, j);
      pt1.set(pt.x() + halfwidth2, pt.y() - halfwidth2);
      pt2.set(pt.x() + halfwidth2, pt.y() + halfwidth2);
      pt3.set(pt.x() - halfwidth2, pt.y() - halfwidth2);
      pt4.set(pt.x() - halfwidth2, pt.y() + halfwidth2);
      distSquare = min(pt2boxDistSquare(pt1, box), pt2boxDistSquare(pt2, box));
      distSquare = min(pt2boxDistSquare(pt3, box), distSquare);
      distSquare = min(pt2boxDistSquare(pt4, box), distSquare);
      if (distSquare < bloatDistSquare) {
        if (isAddPathCost) {
          gridGraph.addDRCCostPlanar(i, j, z); // safe access
        } else {
          gridGraph.subDRCCostPlanar(i, j, z); // safe access
        }
        if (TEST) {
          cout <<"    (" <<i <<", " <<j <<", " <<z <<") minSpc planer" <<endl;
        }
        cnt++;
        //if (!isInitDR()) {
        //  cout <<" planer find viol mIdx (" <<i <<", " <<j <<") " <<pt <<endl;
        //}
      }
    }
  }
  //cout <<"planer mod " <<cnt <<" edges" <<endl;
}


/*inline*/ void FlexDRWorker::modMinSpacingCost(drNet* net, const frBox &box, frMIdx z, bool isAddPathCost, bool isCurrPs) {
  auto lNum = gridGraph.getLayerNum(z);
  // obj1 = curr obj
  frCoord width1  = box.width();
  frCoord length1 = box.length();
  // obj2 planar = other obj
  // layer default width
  frCoord width2planar     = getDesign()->getTech()->getLayer(lNum)->getWidth();
  frCoord halfwidth2planar = width2planar / 2;
  // obj2 viaL = other obj
  frViaDef* viaDefL = (lNum > getDesign()->getTech()->getBottomLayerNum()) ? 
                      getDesign()->getTech()->getLayer(lNum-1)->getDefaultViaDef() : 
                      nullptr;
  frVia viaL(viaDefL);
  frBox viaBoxL(0,0,0,0);
  if (viaDefL) {
    viaL.getLayer2BBox(viaBoxL);
  }
  frCoord width2viaL  = viaBoxL.width();
  frCoord length2viaL = viaBoxL.length();
  // obj2 viaU = other obj
  frViaDef* viaDefU = (lNum < getDesign()->getTech()->getTopLayerNum()) ? 
                      getDesign()->getTech()->getLayer(lNum+1)->getDefaultViaDef() : 
                      nullptr;
  frVia viaU(viaDefU);
  frBox viaBoxU(0,0,0,0);
  if (viaDefU) {
    viaU.getLayer1BBox(viaBoxU);
  }
  frCoord width2viaU  = viaBoxU.width();
  frCoord length2viaU = viaBoxU.length();

  // spacing value needed
  frCoord bloatDistPlanar = 0;
  frCoord bloatDistViaL   = 0;
  frCoord bloatDistViaU   = 0;
  auto con = getDesign()->getTech()->getLayer(lNum)->getMinSpacing();
  if (con) {
    if (con->typeId() == frConstraintTypeEnum::frcSpacingConstraint) {
      bloatDistPlanar = static_cast<frSpacingConstraint*>(con)->getMinSpacing();
      bloatDistViaL   = static_cast<frSpacingConstraint*>(con)->getMinSpacing();
      bloatDistViaU   = static_cast<frSpacingConstraint*>(con)->getMinSpacing();
    } else if (con->typeId() == frConstraintTypeEnum::frcSpacingTablePrlConstraint) {
      bloatDistPlanar = static_cast<frSpacingTablePrlConstraint*>(con)->find(max(width1, width2planar), length1);
      bloatDistViaL   = static_cast<frSpacingTablePrlConstraint*>(con)->find(max(width1, width2viaL), isCurrPs ? length2viaL : min(length1, length2viaL));
      bloatDistViaU   = static_cast<frSpacingTablePrlConstraint*>(con)->find(max(width1, width2viaU), isCurrPs ? length2viaU : min(length1, length2viaU));
    } else if (con->typeId() == frConstraintTypeEnum::frcSpacingTableTwConstraint) {
      bloatDistPlanar = static_cast<frSpacingTableTwConstraint*>(con)->find(width1, width2planar, length1);
      bloatDistViaL   = static_cast<frSpacingTableTwConstraint*>(con)->find(width1, width2viaL, isCurrPs ? length2viaL : min(length1, length2viaL));
      bloatDistViaU   = static_cast<frSpacingTableTwConstraint*>(con)->find(width1, width2viaU, isCurrPs ? length2viaU : min(length1, length2viaU));
    } else {
      cout <<"Warning: min spacing rule not supporterd" <<endl;
      return;
    }
  } else {
    cout <<"Warning: no min spacing rule" <<endl;
    return;
  }

  // other obj eol spc to curr obj
  // no need to bloat eolWithin because eolWithin always < minSpacing
  frCoord bloatDistEolX = 0;
  frCoord bloatDistEolY = 0;
  for (auto con: getDesign()->getTech()->getLayer(lNum)->getEolSpacing()) {
    auto eolSpace  = con->getMinSpacing();
    auto eolWidth  = con->getEolWidth();
    // eol up and down
    if (viaDefL && viaBoxL.right() - viaBoxL.left() < eolWidth) {
      bloatDistEolY = max(bloatDistEolY, eolSpace);
    } 
    if (viaDefU && viaBoxU.right() - viaBoxU.left() < eolWidth) {
      bloatDistEolY = max(bloatDistEolY, eolSpace);
    } 
    // eol left and right
    if (viaDefL && viaBoxL.top() - viaBoxL.bottom() < eolWidth) {
      bloatDistEolX = max(bloatDistEolX, eolSpace);
    }
    if (viaDefU && viaBoxU.top() - viaBoxU.bottom() < eolWidth) {
      bloatDistEolX = max(bloatDistEolX, eolSpace);
    }
  }

  frCoord bloatDist = max(max(bloatDistPlanar, bloatDistViaL), bloatDistViaU);
  //frCoord bloatDistSquare = bloatDist * bloatDist;

  FlexMazeIdx mIdx1;
  FlexMazeIdx mIdx2;
  // assumes width always > 2
  frBox bx(box.left()   - max(bloatDist, bloatDistEolX) - max(max(halfwidth2planar, viaBoxL.right() ), viaBoxU.right() ) + 1, 
           box.bottom() - max(bloatDist, bloatDistEolY) - max(max(halfwidth2planar, viaBoxL.top()   ), viaBoxU.top()   ) + 1,
           box.right()  + max(bloatDist, bloatDistEolX) + max(max(halfwidth2planar, viaBoxL.left()  ), viaBoxU.left()  ) - 1, 
           box.top()    + max(bloatDist, bloatDistEolY) + max(max(halfwidth2planar, viaBoxL.bottom()), viaBoxU.bottom()) - 1);
  gridGraph.getIdxBox(mIdx1, mIdx2, bx);
  //if (!isInitDR()) {
  //  cout <<" box " <<box <<" bloatDist " <<bloatDist <<" bx " <<bx <<endl;
  //  cout <<" midx1/2 (" <<mIdx1.x() <<", " <<mIdx1.y() <<") ("
  //                      <<mIdx2.x() <<", " <<mIdx2.y() <<") (" <<endl;
  //}

  frPoint pt;
  frBox tmpBx;
  frCoord dx, dy, prl;
  frTransform xform;
  frCoord reqDist = 0;
  frCoord distSquare = 0;
  int cnt = 0;
  for (int i = mIdx1.x(); i <= mIdx2.x(); i++) {
    for (int j = mIdx1.y(); j <= mIdx2.y(); j++) {
      gridGraph.getPoint(pt, i, j);
      xform.set(pt);
      // planar
      tmpBx.set(pt.x() - halfwidth2planar, pt.y() - halfwidth2planar,
                pt.x() + halfwidth2planar, pt.y() + halfwidth2planar);
      distSquare = box2boxDistSquare(box, tmpBx, dx, dy);
      prl = max(dx, dy);
      if (con->typeId() == frConstraintTypeEnum::frcSpacingConstraint) {
        reqDist = static_cast<frSpacingConstraint*>(con)->getMinSpacing();
      } else if (con->typeId() == frConstraintTypeEnum::frcSpacingTablePrlConstraint) {
        reqDist = static_cast<frSpacingTablePrlConstraint*>(con)->find(max(width1, width2planar), prl > 0 ? length1 : 0);
      } else if (con->typeId() == frConstraintTypeEnum::frcSpacingTableTwConstraint) {
        reqDist = static_cast<frSpacingTableTwConstraint*>(con)->find(width1, width2planar, prl > 0 ? length1 : 0);
      }
      if (distSquare < reqDist * reqDist) {
        if (isAddPathCost) {
          gridGraph.addDRCCostPlanar(i, j, z); // safe access
        } else {
          gridGraph.subDRCCostPlanar(i, j, z); // safe access
        }
        if (TEST) {
          cout <<"    (" <<i <<", " <<j <<", " <<z <<") minSpc planer" <<endl;
        }
        cnt++;
      }
      // viaL
      if (viaDefL) {
        tmpBx.set(viaBoxL);
        tmpBx.transform(xform);
        distSquare = box2boxDistSquare(box, tmpBx, dx, dy);
        prl = max(dx, dy);
        // curr is ps
        if (isCurrPs) {
          if (dx == 0 && dy > 0) {
            prl = viaBoxL.right() - viaBoxL.left();
          } else if (dx > 0 && dy == 0) {
            prl = viaBoxL.top() - viaBoxL.bottom();
          }
        }
        if (con->typeId() == frConstraintTypeEnum::frcSpacingConstraint) {
          reqDist = static_cast<frSpacingConstraint*>(con)->getMinSpacing();
        } else if (con->typeId() == frConstraintTypeEnum::frcSpacingTablePrlConstraint) {
          reqDist = static_cast<frSpacingTablePrlConstraint*>(con)->find(max(width1, width2viaL), prl);
        } else if (con->typeId() == frConstraintTypeEnum::frcSpacingTableTwConstraint) {
          reqDist = static_cast<frSpacingTableTwConstraint*>(con)->find(width1, width2viaL, prl);
        }
        if (distSquare < reqDist * reqDist) {
          if (isAddPathCost) {
            gridGraph.addDRCCostVia(i, j, z - 1);
          } else {
            gridGraph.subDRCCostVia(i, j, z - 1);
          }
          if (TEST) {
            cout <<"    (" <<i <<", " <<j <<", " <<z - 1 <<") U minSpc via" <<endl;
          }
        } else {
          modMinSpacingCostVia_eol(box, tmpBx, isAddPathCost, false, i, j, z);
        }
        //modMinSpacingCostVia_eol(box, tmpBx, isAddPathCost, false, i, j, z);
      }
      if (viaDefU) {
        tmpBx.set(viaBoxU);
        tmpBx.transform(xform);
        distSquare = box2boxDistSquare(box, tmpBx, dx, dy);
        prl = max(dx, dy);
        // curr is ps
        if (isCurrPs) {
          if (dx == 0 && dy > 0) {
            prl = viaBoxU.right() - viaBoxU.left();
          } else if (dx > 0 && dy == 0) {
            prl = viaBoxU.top() - viaBoxU.bottom();
          }
        }
        if (con->typeId() == frConstraintTypeEnum::frcSpacingConstraint) {
          reqDist = static_cast<frSpacingConstraint*>(con)->getMinSpacing();
        } else if (con->typeId() == frConstraintTypeEnum::frcSpacingTablePrlConstraint) {
          reqDist = static_cast<frSpacingTablePrlConstraint*>(con)->find(max(width1, width2viaU), prl);
        } else if (con->typeId() == frConstraintTypeEnum::frcSpacingTableTwConstraint) {
          reqDist = static_cast<frSpacingTableTwConstraint*>(con)->find(width1, width2viaU, prl);
        }
        if (distSquare < reqDist * reqDist) {
          if (isAddPathCost) {
            gridGraph.addDRCCostVia(i, j, z);
          } else {
            gridGraph.subDRCCostVia(i, j, z);
          }
          if (TEST) {
            cout <<"    (" <<i <<", " <<j <<", " <<z <<") U minSpc via" <<endl;
          }
        } else {
          modMinSpacingCostVia_eol(box, tmpBx, isAddPathCost, true, i, j, z);
        }
        //modMinSpacingCostVia_eol(box, tmpBx, isAddPathCost, true, i, j, z);
      }
    }
  }
  //cout <<"planer mod " <<cnt <<" edges" <<endl;
}

/*inline*/ void FlexDRWorker::modMinSpacingCostVia_eol_helper(const frBox &box, const frBox &testBox, bool isAddPathCost, bool isUpperVia,
                                                          frMIdx i, frMIdx j, frMIdx z) {
  if (testBox.overlaps(box, false)) {
    if (isUpperVia) {
      if (isAddPathCost) {
        gridGraph.addDRCCostVia(i, j, z);
      } else {
        gridGraph.subDRCCostVia(i, j, z);
      }
      if (TEST) {
        cout <<"    (" <<i <<", " <<j <<", " <<z <<") U minSpc eol helper" <<endl;
      }
    } else {
      if (isAddPathCost) {
        gridGraph.addDRCCostVia(i, j, z - 1);
      } else {
        gridGraph.subDRCCostVia(i, j, z - 1);
      }
      if (TEST) {
        cout <<"    (" <<i <<", " <<j <<", " <<z - 1 <<") U minSpc eol helper" <<endl;
      }
    }
  }
}

/*inline*/ void FlexDRWorker::modMinSpacingCostVia_eol(const frBox &box, const frBox &tmpBx, bool isAddPathCost, bool isUpperVia,
                                                   frMIdx i, frMIdx j, frMIdx z) {
  auto lNum = gridGraph.getLayerNum(z);
  frBox testBox;
  if (getDesign()->getTech()->getLayer(lNum)->hasEolSpacing()) {
    for (auto eolCon: getDesign()->getTech()->getLayer(lNum)->getEolSpacing()) {
      auto eolSpace  = eolCon->getMinSpacing();
      auto eolWidth  = eolCon->getEolWidth();
      auto eolWithin = eolCon->getEolWithin();
      // eol to up and down
      if (tmpBx.right() - tmpBx.left() < eolWidth) {
        testBox.set(tmpBx.left() - eolWithin, tmpBx.top(),               tmpBx.right() + eolWithin, tmpBx.top() + eolSpace);
        modMinSpacingCostVia_eol_helper(box, testBox, isAddPathCost, isUpperVia, i, j, z);

        testBox.set(tmpBx.left() - eolWithin, tmpBx.bottom() - eolSpace, tmpBx.right() + eolWithin, tmpBx.bottom());
        modMinSpacingCostVia_eol_helper(box, testBox, isAddPathCost, isUpperVia, i, j, z);
      }
      // eol to left and right
      if (tmpBx.top() - tmpBx.bottom() < eolWidth) {
        testBox.set(tmpBx.right(),           tmpBx.bottom() - eolWithin, tmpBx.right() + eolSpace, tmpBx.top() + eolWithin);
        modMinSpacingCostVia_eol_helper(box, testBox, isAddPathCost, isUpperVia, i, j, z);

        testBox.set(tmpBx.left() - eolSpace, tmpBx.bottom() - eolWithin, tmpBx.left(),             tmpBx.top() + eolWithin);
        modMinSpacingCostVia_eol_helper(box, testBox, isAddPathCost, isUpperVia, i, j, z);
      }
    }
  }
}


void FlexDRWorker::modMinSpacingCostVia(const frBox &box, frMIdx z, bool isAddPathCost, bool isUpperVia, bool isCurrPs) {
  auto lNum = gridGraph.getLayerNum(z);
  // obj1 = curr obj
  frCoord width1  = box.width();
  frCoord length1 = box.length();
  // obj2 = other obj
  // default via dimension
  frViaDef* viaDef = nullptr;
  if (isUpperVia) {
    viaDef = (lNum < getDesign()->getTech()->getTopLayerNum()) ? 
             getDesign()->getTech()->getLayer(lNum+1)->getDefaultViaDef() : 
             nullptr;
  } else {
    viaDef = (lNum > getDesign()->getTech()->getBottomLayerNum()) ? 
             getDesign()->getTech()->getLayer(lNum-1)->getDefaultViaDef() : 
             nullptr;
  }
  if (viaDef == nullptr) {
    return;
  }
  frVia via(viaDef);
  frBox viaBox(0,0,0,0);
  if (isUpperVia) {
    via.getLayer1BBox(viaBox);
  } else {
    via.getLayer2BBox(viaBox);
  }
  frCoord width2  = viaBox.width();
  frCoord length2 = viaBox.length();

  // spacing value needed
  frCoord bloatDist = 0;
  auto con = getDesign()->getTech()->getLayer(lNum)->getMinSpacing();
  if (con) {
    if (con->typeId() == frConstraintTypeEnum::frcSpacingConstraint) {
      bloatDist = static_cast<frSpacingConstraint*>(con)->getMinSpacing();
    } else if (con->typeId() == frConstraintTypeEnum::frcSpacingTablePrlConstraint) {
      bloatDist = static_cast<frSpacingTablePrlConstraint*>(con)->find(max(width1, width2), isCurrPs ? length2 : min(length1, length2));
    } else if (con->typeId() == frConstraintTypeEnum::frcSpacingTableTwConstraint) {
      bloatDist = static_cast<frSpacingTableTwConstraint*>(con)->find(width1, width2, isCurrPs ? length2 : min(length1, length2));
    } else {
      cout <<"Warning: min spacing rule not supporterd" <<endl;
      return;
    }
  } else {
    cout <<"Warning: no min spacing rule" <<endl;
    return;
  }
  // other obj eol spc to curr obj
  // no need to blaot eolWithin because eolWithin always < minSpacing
  frCoord bloatDistEolX = 0;
  frCoord bloatDistEolY = 0;
  for (auto con: getDesign()->getTech()->getLayer(lNum)->getEolSpacing()) {
    auto eolSpace  = con->getMinSpacing();
    auto eolWidth  = con->getEolWidth();
    // eol up and down
    if (viaBox.right() - viaBox.left() < eolWidth) {
      bloatDistEolY = max(bloatDistEolY, eolSpace);
    } 
    // eol left and right
    if (viaBox.top() - viaBox.bottom() < eolWidth) {
      bloatDistEolX = max(bloatDistEolX, eolSpace);
    }
  }
  //frCoord bloatDistSquare = bloatDist * bloatDist;
  
  FlexMazeIdx mIdx1;
  FlexMazeIdx mIdx2;
  // assumes width always > 2
  frBox bx(box.left()   - max(bloatDist, bloatDistEolX) - (viaBox.right() - 0) + 1, 
             box.bottom() - max(bloatDist, bloatDistEolY) - (viaBox.top() - 0) + 1,
           box.right()  + max(bloatDist, bloatDistEolX) + (0 - viaBox.left())  - 1, 
             box.top()    + max(bloatDist, bloatDistEolY) + (0 - viaBox.bottom()) - 1);
  gridGraph.getIdxBox(mIdx1, mIdx2, bx);

  frPoint pt;
  frBox tmpBx;
  frCoord distSquare = 0;
  frCoord dx, dy, prl;
  frTransform xform;
  frCoord reqDist = 0;
  frBox sViaBox;
  for (int i = mIdx1.x(); i <= mIdx2.x(); i++) {
    for (int j = mIdx1.y(); j <= mIdx2.y(); j++) {
      gridGraph.getPoint(pt, i, j);
      xform.set(pt);
      //if (gridGraph.isSVia(i, j, isUpperVia ? z : z - 1)) {
        //auto sViaDef= apSVia[FlexMazeIdx(i, j, isUpperVia ? z : z - 1)];
        //frVia sVia(sViaDef);
        //if (isUpperVia) {
        //  sVia.getLayer1BBox(sViaBox);
        //} else {
        //  sVia.getLayer2BBox(sViaBox);
        //}
        //tmpBx.set(sViaBox);
        //continue;
      //}
      tmpBx.set(viaBox);
      tmpBx.transform(xform);
      distSquare = box2boxDistSquare(box, tmpBx, dx, dy);
      prl = max(dx, dy);
      // curr is ps
      if (isCurrPs) {
        if (dx == 0 && dy > 0) {
          prl = viaBox.right() - viaBox.left();
        } else if (dx > 0 && dy == 0) {
          prl = viaBox.top() - viaBox.bottom();
        }
      }
      if (con->typeId() == frConstraintTypeEnum::frcSpacingConstraint) {
        reqDist = static_cast<frSpacingConstraint*>(con)->getMinSpacing();
      } else if (con->typeId() == frConstraintTypeEnum::frcSpacingTablePrlConstraint) {
        reqDist = static_cast<frSpacingTablePrlConstraint*>(con)->find(max(width1, width2), prl);
      } else if (con->typeId() == frConstraintTypeEnum::frcSpacingTableTwConstraint) {
        reqDist = static_cast<frSpacingTableTwConstraint*>(con)->find(width1, width2, prl);
      }
      if (distSquare < reqDist * reqDist) {
        if (isUpperVia) {
          if (isAddPathCost) {
            gridGraph.addDRCCostVia(i, j, z);
          } else {
            gridGraph.subDRCCostVia(i, j, z);
          }
          if (TEST) {
            cout <<"    (" <<i <<", " <<j <<", " <<z <<") U minSpc via" <<endl;
          }
        } else {
          if (isAddPathCost) {
            gridGraph.addDRCCostVia(i, j, z - 1);
          } else {
            gridGraph.subDRCCostVia(i, j, z - 1);
          }
          if (TEST) {
            cout <<"    (" <<i <<", " <<j <<", " <<z - 1 <<") U minSpc via" <<endl;
          }
        }
      }
      // eol, other obj to curr obj
      modMinSpacingCostVia_eol(box, tmpBx, isAddPathCost, isUpperVia, i, j, z);
    }
  }

}


// type == 0: planer
// type == 1: down
// type == 2: up
/*inline*/ void FlexDRWorker::modEolSpacingCost_helper(const frBox &testbox, frMIdx z, bool isAddPathCost, int type) {
  auto lNum = gridGraph.getLayerNum(z);
  frBox bx;
  if (type == 0) {
    // layer default width
    frCoord width2     = getDesign()->getTech()->getLayer(lNum)->getWidth();
    frCoord halfwidth2 = width2 / 2;
    // assumes width always > 2
    bx.set(testbox.left()   - halfwidth2 + 1, testbox.bottom() - halfwidth2 + 1,
           testbox.right()  + halfwidth2 - 1, testbox.top()    + halfwidth2 - 1);
  } else {
    // default via dimension
    frViaDef* viaDef = nullptr;
    if (type == 1) {
      viaDef = (lNum > getDesign()->getTech()->getBottomLayerNum()) ? 
               getDesign()->getTech()->getLayer(lNum-1)->getDefaultViaDef() : 
               nullptr;
    } else if (type == 2) {
      viaDef = (lNum < getDesign()->getTech()->getTopLayerNum()) ? 
               getDesign()->getTech()->getLayer(lNum+1)->getDefaultViaDef() : 
               nullptr;
    }
    if (viaDef == nullptr) {
      return;
    }
    frVia via(viaDef);
    frBox viaBox(0,0,0,0);
    if (type == 2) { // upper via
      via.getLayer1BBox(viaBox);
    } else {
      via.getLayer2BBox(viaBox);
    }
    // assumes via bbox always > 2
    bx.set(testbox.left()  - (viaBox.right() - 0) + 1, testbox.bottom() - (viaBox.top() - 0) + 1,
           testbox.right() + (0 - viaBox.left())  - 1, testbox.top()    + (0 - viaBox.bottom()) - 1);
  }

  FlexMazeIdx mIdx1;
  FlexMazeIdx mIdx2;
  gridGraph.getIdxBox(mIdx1, mIdx2, bx); // >= bx
  
  for (int i = mIdx1.x(); i <= mIdx2.x(); i++) {
    for (int j = mIdx1.y(); j <= mIdx2.y(); j++) {
      if (type == 0) {
        if (isAddPathCost) {
          gridGraph.addDRCCostPlanar(i, j, z); // safe access
        } else {
          gridGraph.subDRCCostPlanar(i, j, z); // safe access
        }
        if (TEST) {
          cout <<"    (" <<i <<", " <<j <<", " <<z <<") N eolSpc" <<endl;
        }
      } else if (type == 1) {
        //if (gridGraph.isSVia(i, j, z - 1)) {
        //  continue;
        //}
        if (isAddPathCost) {
          gridGraph.addDRCCostVia(i, j, z - 1); // safe access
        } else {
          gridGraph.subDRCCostVia(i, j, z - 1); // safe access
        }
        if (TEST) {
          cout <<"    (" <<i <<", " <<j <<", " <<z - 1 <<") U eolSpc" <<endl;
        }
      } else if (type == 2) {
        //if (gridGraph.isSVia(i, j, z)) {
        //  continue;
        //}
        if (isAddPathCost) {
          gridGraph.addDRCCostVia(i, j, z); // safe access
        } else {
          gridGraph.subDRCCostVia(i, j, z); // safe access
        }
        if (TEST) {
          cout <<"    (" <<i <<", " <<j <<", " <<z <<") U eolSpc" <<endl;
        }
      }
    }
  }
}

/*inline*/ void FlexDRWorker::modEolSpacingCost(const frBox &box, frMIdx z, bool isAddPathCost) {
  auto lNum = gridGraph.getLayerNum(z);
  frBox testBox;
  if (getDesign()->getTech()->getLayer(lNum)->hasEolSpacing()) {
    for (auto con: getDesign()->getTech()->getLayer(lNum)->getEolSpacing()) {
      auto eolSpace  = con->getMinSpacing();
      auto eolWidth  = con->getEolWidth();
      auto eolWithin = con->getEolWithin();
      // curr obj to other obj eol
      //if (!isInitDR()) {
      //  cout <<"eolSpace/within = " <<eolSpace <<" " <<eolWithin <<endl;
      //}
      // eol to up and down
      if (box.right() - box.left() < eolWidth) {
        testBox.set(box.left() - eolWithin, box.top(),               box.right() + eolWithin, box.top() + eolSpace);
        //if (!isInitDR()) {
        //  cout <<"  topBox " <<testBox <<endl;
        //}
        modEolSpacingCost_helper(testBox, z, isAddPathCost, 0);
        modEolSpacingCost_helper(testBox, z, isAddPathCost, 1);
        modEolSpacingCost_helper(testBox, z, isAddPathCost, 2);
        testBox.set(box.left() - eolWithin, box.bottom() - eolSpace, box.right() + eolWithin, box.bottom());
        //if (!isInitDR()) {
        //  cout <<"  bottomBox " <<testBox <<endl;
        //}
        modEolSpacingCost_helper(testBox, z, isAddPathCost, 0);
        modEolSpacingCost_helper(testBox, z, isAddPathCost, 1);
        modEolSpacingCost_helper(testBox, z, isAddPathCost, 2);
      }
      // eol to left and right
      if (box.top() - box.bottom() < eolWidth) {
        testBox.set(box.right(),           box.bottom() - eolWithin, box.right() + eolSpace, box.top() + eolWithin);
        //if (!isInitDR()) {
        //  cout <<"  rightBox " <<testBox <<endl;
        //}
        modEolSpacingCost_helper(testBox, z, isAddPathCost, 0);
        modEolSpacingCost_helper(testBox, z, isAddPathCost, 1);
        modEolSpacingCost_helper(testBox, z, isAddPathCost, 2);
        testBox.set(box.left() - eolSpace, box.bottom() - eolWithin, box.left(),             box.top() + eolWithin);
        //if (!isInitDR()) {
        //  cout <<"  leftBox " <<testBox <<endl;
        //}
        modEolSpacingCost_helper(testBox, z, isAddPathCost, 0);
        modEolSpacingCost_helper(testBox, z, isAddPathCost, 1);
        modEolSpacingCost_helper(testBox, z, isAddPathCost, 2);
      }
      // other obj to curr obj eol
    }
  }
}

/*inline*/ void FlexDRWorker::modCutSpacingCost(const frBox &box, frMIdx z, bool isAddPathCost) {
  auto lNum = gridGraph.getLayerNum(z) + 1;
  if (!getDesign()->getTech()->getLayer(lNum)->hasCutSpacing()) {
    return;
  }
  // obj1 = curr obj
  // obj2 = other obj
  // default via dimension
  frViaDef* viaDef = getDesign()->getTech()->getLayer(lNum)->getDefaultViaDef();
  frVia via(viaDef);
  frBox viaBox(0,0,0,0);
  via.getCutBBox(viaBox);

  // spacing value needed
  frCoord bloatDist = 0;
  for (auto con: getDesign()->getTech()->getLayer(lNum)->getCutSpacing()) {
    bloatDist = max(bloatDist, con->getCutSpacing());
  }
  //frCoord bloatDistSquare = bloatDist * bloatDist;
  
  FlexMazeIdx mIdx1;
  FlexMazeIdx mIdx2;
  // assumes width always > 2
  frBox bx(box.left()   - bloatDist - (viaBox.right() - 0) + 1, 
           box.bottom() - bloatDist - (viaBox.top() - 0) + 1,
           box.right()  + bloatDist + (0 - viaBox.left())  - 1, 
           box.top()    + bloatDist + (0 - viaBox.bottom()) - 1);
  gridGraph.getIdxBox(mIdx1, mIdx2, bx);

  frPoint pt;
  frBox tmpBx;
  frCoord distSquare = 0;
  frCoord c2cSquare = 0;
  frCoord dx, dy, prl;
  frTransform xform;
  //frCoord reqDist = 0;
  frCoord reqDistSquare = 0;
  frPoint boxCenter, tmpBxCenter;
  boxCenter.set((box.left() + box.right()) / 2, (box.bottom() + box.top()) / 2);
  frCoord currDistSquare = 0;
  bool hasViol = false;
  for (int i = mIdx1.x(); i <= mIdx2.x(); i++) {
    for (int j = mIdx1.y(); j <= mIdx2.y(); j++) {
      for (auto &uFig: via.getViaDef()->getCutFigs()) {
        auto obj = static_cast<frRect*>(uFig.get());
        gridGraph.getPoint(pt, i, j);
        xform.set(pt);
        obj->getBBox(tmpBx);
        //tmpBx.set(viaBox);
        tmpBx.transform(xform);
        tmpBxCenter.set((tmpBx.left() + tmpBx.right()) / 2, (tmpBx.bottom() + tmpBx.top()) / 2);
        distSquare = box2boxDistSquare(box, tmpBx, dx, dy);
        c2cSquare = (boxCenter.x() - tmpBxCenter.x()) * (boxCenter.x() - tmpBxCenter.x()) + 
                    (boxCenter.y() - tmpBxCenter.y()) * (boxCenter.y() - tmpBxCenter.y()); 
        prl = max(dx, dy);
        for (auto con: getDesign()->getTech()->getLayer(lNum)->getCutSpacing()) {
          hasViol = false;
          //reqDist = con->getCutSpacing();
          reqDistSquare = con->getCutSpacing() * con->getCutSpacing();
          currDistSquare = con->hasCenterToCenter() ? c2cSquare : distSquare;
          if (con->hasSameNet()) {
            continue;
          }
          if (con->isLayer()) {
            ;
          } else if (con->isAdjacentCuts()) {
            if (currDistSquare < reqDistSquare) {
              hasViol = true;
              // should disable hasViol and modify this part to new grid graph
            }
          } else if (con->isParallelOverlap()) {
            if (prl > 0 && currDistSquare < reqDistSquare) {
              hasViol = true;
            }
          } else if (con->isArea()) {
            auto currArea = max(box.length() * box.width(), tmpBx.length() * tmpBx.width());
            if (currArea >= con->getCutArea() && currDistSquare < reqDistSquare) {
              hasViol = true;
            }
          } else {
            if (currDistSquare < reqDistSquare) {
              hasViol = true;
            }
          }
          if (hasViol) {
            if (isAddPathCost) {
              gridGraph.addDRCCostVia(i, j, z);
            } else {
              gridGraph.subDRCCostVia(i, j, z);
            }
            if (TEST) {
              cout <<"    (" <<i <<", " <<j <<", " <<z <<") U cutSpc" <<endl;
            }
            break;
          }
        }
      }
    }
  }
}

void FlexDRWorker::addPathCost(drConnFig *connFig) {
  modPathCost(connFig, true);
}

void FlexDRWorker::subPathCost(drConnFig *connFig) {
  modPathCost(connFig, false);
}

/*inline*/ void FlexDRWorker::modPathCost(drConnFig *connFig, bool isAddPathCost) {
  if (connFig->typeId() == drcPathSeg) {
    auto obj = static_cast<drPathSeg*>(connFig);
    auto net = obj->getNet();
    FlexMazeIdx bi, ei;
    obj->getMazeIdx(bi, ei);
    if (TEST) {
      cout <<"  ";
      if (isAddPathCost) {
        cout <<"add";
      } else {
        cout <<"sub";
      }
      cout <<"PsCost for " <<bi <<" -- " <<ei <<endl;
    }
    // new 
    frBox box;
    obj->getBBox(box);
    
    modMinSpacingCostPlaner(net, box, bi.z(), isAddPathCost);
    modMinSpacingCostVia(box, bi.z(), isAddPathCost, true,  true);
    modMinSpacingCostVia(box, bi.z(), isAddPathCost, false, true);
    //modMinSpacingCost(net, box, bi.z(), isAddPathCost, true);
    modEolSpacingCost(box, bi.z(), isAddPathCost);
  } else if (connFig->typeId() == drcPatchWire) {
    auto obj = static_cast<drPatchWire*>(connFig);
    auto net = obj->getNet();
    frMIdx zIdx = gridGraph.getMazeZIdx(obj->getLayerNum());
    // FlexMazeIdx bi, ei;
    // obj->getMazeIdx(bi, ei);
    // if (TEST) {
    //   cout <<"  ";
    //   if (isAddPathCost) {
    //     cout <<"add";
    //   } else {
    //     cout <<"sub";
    //   }
    //   cout <<"PsCost for " <<bi <<" -- " <<ei <<endl;
    // }
    // new 
    frBox box;
    obj->getBBox(box);
    
    modMinSpacingCostPlaner(net, box, zIdx, isAddPathCost);
    modMinSpacingCostVia(box, zIdx, isAddPathCost, true,  true);
    modMinSpacingCostVia(box, zIdx, isAddPathCost, false, true);
    //modMinSpacingCost(net, box, bi.z(), isAddPathCost, true);
    modEolSpacingCost(box, zIdx, isAddPathCost);
  } else if (connFig->typeId() == drcVia) {
    auto obj = static_cast<drVia*>(connFig);
    auto net = obj->getNet();
    FlexMazeIdx bi, ei;
    obj->getMazeIdx(bi, ei);
    if (TEST) {
      cout <<"  ";
      if (isAddPathCost) {
        cout <<"add";
      } else {
        cout <<"sub";
      }
      cout <<"ViaCost for " <<bi <<" -- " <<ei <<endl;
    }
    // new
    
    frBox box;
    obj->getLayer1BBox(box); // assumes enclosure for via is always rectangle

    modMinSpacingCostPlaner(net, box, bi.z(), isAddPathCost);
    modMinSpacingCostVia(box, bi.z(), isAddPathCost, true,  false);
    modMinSpacingCostVia(box, bi.z(), isAddPathCost, false, false);
    //modMinSpacingCost(net, box, bi.z(), isAddPathCost, false);
    modEolSpacingCost(box, bi.z(), isAddPathCost);
    
    obj->getLayer2BBox(box); // assumes enclosure for via is always rectangle

    modMinSpacingCostPlaner(net, box, ei.z(), isAddPathCost);
    modMinSpacingCostVia(box, ei.z(), isAddPathCost, true,  false);
    modMinSpacingCostVia(box, ei.z(), isAddPathCost, false, false);
    //modMinSpacingCost(net, box, ei.z(), isAddPathCost, false);
    modEolSpacingCost(box, ei.z(), isAddPathCost);

    //obj->getCutBBox(box);
    frTransform xform;
    frPoint pt;
    obj->getOrigin(pt);
    xform.set(pt);
    for (auto &uFig: obj->getViaDef()->getCutFigs()) {
      //if (uFig->typeId() == frcRect) {
        auto rect = static_cast<frRect*>(uFig.get());
        rect->getBBox(box);
        box.transform(xform);
        modCutSpacingCost(box, bi.z(), isAddPathCost);
      //}
    }
  }
}

void FlexDRWorker::mazeIterInit_sortRerouteNets(vector<drNet*> &rerouteNets) {
  // sort
  sort(rerouteNets.begin(), rerouteNets.end(), 
       [](drNet* const &a, drNet* const &b) {
       frBox boxA, boxB;
       a->getPinBox(boxA);
       b->getPinBox(boxB);
       auto areaA = (boxA.right() - boxA.left()) * (boxA.top() - boxA.bottom());
       auto areaB = (boxB.right() - boxB.left()) * (boxB.top() - boxB.bottom());
       return (a->getNumPinsIn() == b->getNumPinsIn() ? (areaA == areaB ? a->getId() < b->getId() : areaA < areaB) : 
                                                        a->getNumPinsIn() < b->getNumPinsIn());
       });
}

void FlexDRWorker::mazeIterInit_initDR(vector<drNet*> &rerouteNets) {
  for (auto &net: nets) {
    rerouteNets.push_back(net.get());
  }
  mazeIterInit_sortRerouteNets(rerouteNets);
  for (auto &net: rerouteNets) {
    net->setModified(true);
    net->setNumMarkers(0);
    // add via access cost when net is not routed
    if (RESERVE_VIA_ACCESS) {
      initMazeCost_via_helper(net, true);
    }
    //net->clear();
  }
}

// temporary settings to test search and repair
void FlexDRWorker::mazeIterInit_searchRepair(vector<drNet*> &rerouteNets) {
  auto &workerRegionQuery = getWorkerRegionQuery();
  int cnt = 0;
  if (getRipupMode() == 0) {
    for (auto &net: nets) {
      if (net->isRipup()) {
        rerouteNets.push_back(net.get());
      }
    }
  } else if (getRipupMode() == 1) {
    for (auto &net: nets) {
      rerouteNets.push_back(net.get());
    }
  }
  // change the drNet magical sorting bit here
  // to do
  //sort(rerouteNets.begin(), rerouteNets.end(), 
  //     [](drNet* const &a, drNet* const &b) {return *a < *b;});
  mazeIterInit_sortRerouteNets(rerouteNets);
  for (auto &net: rerouteNets) {
    net->setModified(true);
    net->setNumMarkers(0);
    for (auto &uConnFig: net->getRouteConnFigs()) {
      subPathCost(uConnFig.get());
      workerRegionQuery.remove(uConnFig.get()); // worker region query
      cnt++;
    }
    // add via access cost when net is not routed
    if (RESERVE_VIA_ACCESS) {
      initMazeCost_via_helper(net, true);
    }
    net->clear();
  }
  //cout <<"sr sub " <<cnt <<" connfig costs" <<endl;
}

void FlexDRWorker::mazeIterInit(vector<drNet*> &rerouteNets) {
  initMazeCost_marker();
  if (isInitDR()) {
    mazeIterInit_initDR(rerouteNets);
  } else {
    mazeIterInit_searchRepair(rerouteNets);
  }
}

void FlexDRWorker::mazeNetInit(drNet* net) {
  gridGraph.resetStatus();
  // sub via access cost when net is about to route
  if (RESERVE_VIA_ACCESS) {
    initMazeCost_via_helper(net, false);
  }
  initMazeCost_ap_helper(net, false);
  initMazeCost_boundary_helper(net, false);
}

void FlexDRWorker::mazeNetEnd(drNet* net) {
  initMazeCost_ap_helper(net, true);
  initMazeCost_boundary_helper(net, true);
}

void FlexDRWorker::route_drc() {
  DRCWorker drcWorker(getDesign(), fixedObjs);
  drcWorker.addDRNets(nets);
  using namespace std::chrono;
  high_resolution_clock::time_point t0 = high_resolution_clock::now();
  drcWorker.init();
  high_resolution_clock::time_point t1 = high_resolution_clock::now();
  drcWorker.setup();
  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  drcWorker.main();
  setMarkers(drcWorker.getViolations());
  high_resolution_clock::time_point t3 = high_resolution_clock::now();
  //drcWorker.report();
  
  duration<double> time_span0 = duration_cast<duration<double>>(t1 - t0);

  duration<double> time_span1 = duration_cast<duration<double>>(t2 - t1);

  duration<double> time_span2 = duration_cast<duration<double>>(t3 - t2);
  if (VERBOSE > 1) {
    stringstream ss;
    ss   <<"DRC  (INIT/SETUP/MAIN) " <<time_span0.count() <<" " 
                                    <<time_span1.count() <<" "
                                    <<time_span2.count() <<" "
                                    <<endl;
    ss <<"#viol = " <<markers.size() <<endl;
    cout <<ss.str() <<flush;
  }
}

void FlexDRWorker::route_postRouteViaSwap() {
  auto &workerRegionQuery = getWorkerRegionQuery();
  set<FlexMazeIdx> modifiedViaIdx;
  frBox box;
  vector<drConnFig*> results;
  frPoint bp;
  FlexMazeIdx bi, ei;
  bool flag = false;
  for (auto &marker: getMarkers()) {
    results.clear();
    marker.getBBox(box);
    auto lNum = marker.getLayerNum();
    workerRegionQuery.query(box, lNum, results);
    for (auto &connFig: results) {
      if (connFig->typeId() == drcVia) {
        auto obj = static_cast<drVia*>(connFig);
        obj->getMazeIdx(bi, ei);
        auto it = apSVia.find(bi);
        if (modifiedViaIdx.find(bi) == modifiedViaIdx.end() && it != apSVia.end()) {
          auto ap = it->second;
          if (ap->nextAccessViaDef()) {
            auto newViaDef = ap->getAccessViaDef();
            workerRegionQuery.remove(obj);
            obj->setViaDef(newViaDef);
            workerRegionQuery.add(obj);
            modifiedViaIdx.insert(bi);
            flag = true;
          }
        }
      }
    }
  }
  if (flag) {
    route_drc();
  }
}

void FlexDRWorker::route() {
  //bool enableOutput = true;
  bool enableOutput = false;
  if (enableOutput) {
    cout << "start Maze route #nets = " <<nets.size() <<endl;
  }
  if (isEnableDRC() && getRipupMode() == 0 && getInitNumMarkers() == 0) {
    return;
  }
  if (DRCTEST) {
    DRCWorker drcWorker(getDesign(), fixedObjs);
    drcWorker.addDRNets(nets);
    using namespace std::chrono;
    high_resolution_clock::time_point t0 = high_resolution_clock::now();
    drcWorker.init();
    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    drcWorker.setup();
    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    drcWorker.main();
    high_resolution_clock::time_point t3 = high_resolution_clock::now();
    drcWorker.report();
    
    duration<double> time_span0 = duration_cast<duration<double>>(t1 - t0);

    duration<double> time_span1 = duration_cast<duration<double>>(t2 - t1);

    duration<double> time_span2 = duration_cast<duration<double>>(t3 - t2);
    stringstream ss;
    ss   <<"time (INIT/SETUP/MAIN) " <<time_span0.count() <<" " 
                                     <<time_span1.count() <<" "
                                     <<time_span2.count() <<" "
                                     <<endl;
    cout <<ss.str() <<flush;


  } else {
    for (int i = 0; i < mazeEndIter; ++i) {
      vector<drNet*> rerouteNets;
      mazeIterInit(rerouteNets);
      minAreaVios.clear();
      //if (i == 0) {
      //  workerDRCCost    = DRCCOST;
      //  workerMarkerCost = MARKERCOST;
      //} else {
      //  workerDRCCost *= 2;
      //  workerMarkerCost *= 2;
      //}
      for (auto net: rerouteNets) {
        //for (auto &pin: net->getPins()) {
        //  for (auto &ap: )
        mazeNetInit(net);
        bool isRouted = routeNet(net);
        if (isRouted == false) {
          // TODO: output maze area
          cout << "Fatal error: Maze Route cannot find path. Connectivity Changed.\n";
          if (OUT_MAZE_FILE != string("")) {
            gridGraph.print();
          }
          exit(1);
        }
        mazeNetEnd(net);
      }
      // drc worker here
      if (isEnableDRC()) {
        route_drc();
        route_postRouteViaSwap();
      }

      // save to best drc
      if (i == 0 || (isEnableDRC() && getMarkers().size() < getBestMarkers().size())) {
        for (auto &net: nets) {
          net->setBestRouteConnFigs();
        }
        setBestMarkers();
      }

      // quick drc
      int violNum = getNumQuickMarkers();
      if (VERBOSE > 1) {
        cout <<"#quick viol = " <<getNumQuickMarkers() <<endl;
      }
      if (isEnableDRC() && getMarkers().empty()) {
        break;
      } else if (violNum == 0) {
        break;
      }
    }
  }
}

void FlexDRWorker::routeNet_prep(drNet* net, set<drPin*, frBlockObjectComp> &unConnPins, 
                                 map<FlexMazeIdx, set<drPin*, frBlockObjectComp> > &mazeIdx2unConnPins/*,
                                 map<FlexMazeIdx, frViaDef*> &apSVia*/) {
  for (auto &pin: net->getPins()) {
    unConnPins.insert(pin.get());
    for (auto &ap: pin->getAccessPatterns()) {
      FlexMazeIdx mi;
      ap->getMazeIdx(mi);
      mazeIdx2unConnPins[mi].insert(pin.get());
      gridGraph.setDst(mi);
    }
  }
}

void FlexDRWorker::routeNet_setSrc(set<drPin*, frBlockObjectComp> &unConnPins, 
                                   map<FlexMazeIdx, set<drPin*, frBlockObjectComp> > &mazeIdx2unConnPins,
                                   vector<FlexMazeIdx> &connComps, 
                                   FlexMazeIdx &ccMazeIdx1, FlexMazeIdx &ccMazeIdx2) {
  frMIdx xDim, yDim, zDim;
  gridGraph.getDim(xDim, yDim, zDim);
  ccMazeIdx1.set(xDim - 1, yDim - 1, zDim - 1);
  ccMazeIdx2.set(0, 0, 0);
  // first pin selection algorithm goes here
  // choose the center pin
  int totAPCnt = 0;
  frCoord totX = 0;
  frCoord totY = 0;
  frCoord totZ = 0;
  FlexMazeIdx mi;
  frPoint bp;
  for (auto &pin: unConnPins) {
    for (auto &ap: pin->getAccessPatterns()) {
      ap->getMazeIdx(mi);
      ap->getPoint(bp);
      totX += bp.x();
      totY += bp.y();
      totZ += gridGraph.getZHeight(mi.z());
      totAPCnt++;
    }
  }
  totX /= totAPCnt;
  totY /= totAPCnt;
  totZ /= totAPCnt;

  frCoord currDist = std::numeric_limits<frCoord>::max();
  drPin* currPin = nullptr;
  for (auto &pin: unConnPins) {
    for (auto &ap: pin->getAccessPatterns()) {
      ap->getMazeIdx(mi);
      ap->getPoint(bp);
      frCoord dist = abs(totX - bp.x()) + abs(totY - bp.y()) + abs(totZ - gridGraph.getZHeight(mi.z()));
      if (dist < currDist) {
        currDist = dist;
        currPin  = pin;
      }
    }
  }
  unConnPins.erase(currPin);

  //auto currPin = *(unConnPins.begin());
  //unConnPins.erase(unConnPins.begin());
  // first pin selection algorithm ends here
  for (auto &ap: currPin->getAccessPatterns()) {
    ap->getMazeIdx(mi);
    connComps.push_back(mi);
    ccMazeIdx1.set(min(ccMazeIdx1.x(), mi.x()),
                   min(ccMazeIdx1.y(), mi.y()),
                   min(ccMazeIdx1.z(), mi.z()));
    ccMazeIdx2.set(max(ccMazeIdx2.x(), mi.x()),
                   max(ccMazeIdx2.y(), mi.y()),
                   max(ccMazeIdx2.z(), mi.z()));
    auto it = mazeIdx2unConnPins.find(mi);
    if (it == mazeIdx2unConnPins.end()) {
      continue;
    }
    auto it2 = it->second.find(currPin);
    if (it2 == it->second.end()) {
      continue;
    }
    it->second.erase(it2);

    gridGraph.setSrc(mi);
    // remove dst label only when no other pins share the same loc
    if (it->second.empty()) {
      mazeIdx2unConnPins.erase(it);
      gridGraph.resetDst(mi);
    }
  }

}

drPin* FlexDRWorker::routeNet_getNextDst(FlexMazeIdx &ccMazeIdx1, FlexMazeIdx &ccMazeIdx2, 
                                         map<FlexMazeIdx, set<drPin*, frBlockObjectComp> > &mazeIdx2unConnPins) {
  frPoint pt;
  frPoint ll, ur;
  gridGraph.getPoint(ll, ccMazeIdx1.x(), ccMazeIdx1.y());
  gridGraph.getPoint(ur, ccMazeIdx2.x(), ccMazeIdx2.y());
  frCoord currDist = std::numeric_limits<frCoord>::max();
  drPin* nextDst = nullptr;
  for (auto &[mazeIdx, setS]: mazeIdx2unConnPins) {
    gridGraph.getPoint(pt, mazeIdx.x(), mazeIdx.y());
    frCoord dx = max(max(ll.x() - pt.x(), pt.x() - ur.x()), 0);
    frCoord dy = max(max(ll.y() - pt.y(), pt.y() - ur.y()), 0);
    frCoord dz = max(max(gridGraph.getZHeight(ccMazeIdx1.z()) - gridGraph.getZHeight(mazeIdx.z()), 
                         gridGraph.getZHeight(mazeIdx.z()) - gridGraph.getZHeight(ccMazeIdx2.z())), 0);
    if (dx + dy + dz < currDist) {
      currDist = dx + dy + dz;
      nextDst = *(setS.begin());
    }
    if (currDist == 0) {
      break;
    }
  }
  return nextDst;
}

void FlexDRWorker::mazePinInit() {
  gridGraph.resetPrevNodeDir();
}

void FlexDRWorker::routeNet_postAstarUpdate(vector<FlexMazeIdx> &path, vector<FlexMazeIdx> &connComps,
                                            set<drPin*, frBlockObjectComp> &unConnPins, 
                                            map<FlexMazeIdx, set<drPin*, frBlockObjectComp> > &mazeIdx2unConnPins,
                                            bool isFirstConn) {
  // first point is dst
  set<FlexMazeIdx> localConnComps;
  if (!path.empty()) {
    auto mi = path[0];
    vector<drPin*> tmpPins;
    for (auto pin: mazeIdx2unConnPins[mi]) {
      //unConnPins.erase(pin);
      tmpPins.push_back(pin);
    }
    for (auto pin: tmpPins) {
      unConnPins.erase(pin);
      for (auto &ap: pin->getAccessPatterns()) {
        FlexMazeIdx mi;
        ap->getMazeIdx(mi);
        auto it = mazeIdx2unConnPins.find(mi);
        if (it == mazeIdx2unConnPins.end()) {
          continue;
        }
        auto it2 = it->second.find(pin);
        if (it2 == it->second.end()) {
          continue;
        }
        it->second.erase(it2);
        if (it->second.empty()) {
          mazeIdx2unConnPins.erase(it);
          gridGraph.resetDst(mi);
        }
        if (ALLOW_PIN_AS_FEEDTHROUGH) {
          localConnComps.insert(mi);
          gridGraph.setSrc(mi);
        }
      }
    }
  } else {
    cout <<"Error: routeNet_postAstarUpdate path is empty" <<endl;
  }
  // must be before comment line ABC so that the used actual src is set in gridgraph
  if (isFirstConn && (!ALLOW_PIN_AS_FEEDTHROUGH)) {
    for (auto &mi: connComps) {
      gridGraph.resetSrc(mi);
    }
    connComps.clear();
    if ((int)path.size() == 1) {
      connComps.push_back(path[0]);
      gridGraph.setSrc(path[0]);
    }
  }
  // line ABC
  // must have >0 length
  for (int i = 0; i < (int)path.size() - 1; ++i) {
    auto start = path[i];
    auto end = path[i + 1];
    auto startX = start.x(), startY = start.y(), startZ = start.z();
    auto endX = end.x(), endY = end.y(), endZ = end.z();
    // horizontal wire
    if (startX != endX && startY == endY && startZ == endZ) {
      for (auto currX = std::min(startX, endX); currX <= std::max(startX, endX); ++currX) {
        localConnComps.insert(FlexMazeIdx(currX, startY, startZ));
        gridGraph.setSrc(currX, startY, startZ);
        //gridGraph.resetDst(currX, startY, startZ);
      }
    // vertical wire
    } else if (startX == endX && startY != endY && startZ == endZ) {
      for (auto currY = std::min(startY, endY); currY <= std::max(startY, endY); ++currY) {
        localConnComps.insert(FlexMazeIdx(startX, currY, startZ));
        gridGraph.setSrc(startX, currY, startZ);
        //gridGraph.resetDst(startX, currY, startZ);
      }
    // via
    } else if (startX == endX && startY == endY && startZ != endZ) {
      for (auto currZ = std::min(startZ, endZ); currZ <= std::max(startZ, endZ); ++currZ) {
        localConnComps.insert(FlexMazeIdx(startX, startY, currZ));
        gridGraph.setSrc(startX, startY, currZ);
        //gridGraph.resetDst(startX, startY, currZ);
      }
    // zero length
    } else if (startX == endX && startY == endY && startZ == endZ) {
      //root.addPinGrid(startX, startY, startZ);
      std::cout << "Warning: zero-length path in updateFlexPin\n";
    } else {
      std::cout << "Error: non-colinear path in updateFlexPin\n";
    }
  }
  for (auto &mi: localConnComps) {
    if (isFirstConn && !ALLOW_PIN_AS_FEEDTHROUGH) {
      connComps.push_back(mi);
    } else {
      if (!(mi == *(path.cbegin()))) {
        connComps.push_back(mi);
      }
    }
  }
}

void FlexDRWorker::routeNet_postAstarWritePath(drNet* net, vector<FlexMazeIdx> &points/*, 
                                               const map<FlexMazeIdx, frViaDef*> &apSVia*/) {
  //bool enableOutput = true;
  bool enableOutput = false;
  if (points.empty()) {
    if (enableOutput) {
      std::cout << "Warning: path is empty in writeMazePath\n";
    }
    return;
  }
  if (TEST && points.size()) {
    cout <<"path";
    for (auto &mIdx: points) {
      cout <<" (" <<mIdx.x() <<", " <<mIdx.y() <<", " <<mIdx.z() <<")";
    }
    cout <<endl;
  }
  auto &workerRegionQuery = getWorkerRegionQuery();
  for (int i = 0; i < (int)points.size() - 1; ++i) {
    FlexMazeIdx start, end;
    if (points[i + 1] < points[i]) {
      start = points[i + 1];
      end = points[i];
    } else {
      start = points[i];
      end = points[i + 1];
    }
    auto startX = start.x(), startY = start.y(), startZ = start.z();
    auto endX = end.x(), endY = end.y(), endZ = end.z();
    // horizontal wire
    if (startX != endX && startY == endY && startZ == endZ) {
      frPoint startLoc, endLoc;
      frLayerNum currLayerNum = gridGraph.getLayerNum(startZ);
      gridGraph.getPoint(startLoc, startX, startY);
      gridGraph.getPoint(endLoc, endX, endY);
      auto currPathSeg = make_unique<drPathSeg>();
      currPathSeg->setPoints(startLoc, endLoc);
      currPathSeg->setLayerNum(currLayerNum);
      currPathSeg->addToNet(net);
      currPathSeg->setStyle(getTech()->getLayer(currLayerNum)->getDefaultSegStyle());
      currPathSeg->setMazeIdx(start, end);
      unique_ptr<drConnFig> tmp(std::move(currPathSeg));
      workerRegionQuery.add(tmp.get());
      net->addRoute(tmp);
      // quick drc cnt
      bool prevHasCost = false;
      for (int i = startX; i < endX; i++) {
        if (gridGraph.hasDRCCost(i, startY, startZ, frDirEnum::E)) {
          if (!prevHasCost) {
            net->addMarker();
            prevHasCost = true;
          }
          if (TEST) {
            cout <<" pass marker @(" <<i <<", " <<startY <<", " <<startZ <<") E" <<endl;
          }
        } else {
          prevHasCost = false;
        }
      }
      if (enableOutput) {
        cout <<" write horz pathseg (" 
             <<startLoc.x() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() <<", " 
             <<startLoc.y() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() <<") (" 
             <<endLoc.x()   * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() <<", " 
             <<endLoc.y()   * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() <<") " 
             <<getTech()->getLayer(currLayerNum)->getName() <<endl;
      }
    // vertical wire
    } else if (startX == endX && startY != endY && startZ == endZ) {
      frPoint startLoc, endLoc;
      frLayerNum currLayerNum = gridGraph.getLayerNum(startZ);
      gridGraph.getPoint(startLoc, startX, startY);
      gridGraph.getPoint(endLoc, endX, endY);
      auto currPathSeg = make_unique<drPathSeg>();
      currPathSeg->setPoints(startLoc, endLoc);
      currPathSeg->setLayerNum(currLayerNum);
      currPathSeg->addToNet(net);
      currPathSeg->setStyle(getTech()->getLayer(currLayerNum)->getDefaultSegStyle());
      currPathSeg->setMazeIdx(start, end);
      unique_ptr<drConnFig> tmp(std::move(currPathSeg));
      workerRegionQuery.add(tmp.get());
      net->addRoute(tmp);
      // quick drc cnt
      bool prevHasCost = false;
      for (int i = startY; i < endY; i++) {
        if (gridGraph.hasDRCCost(startX, i, startZ, frDirEnum::E)) {
          if (!prevHasCost) {
            net->addMarker();
            prevHasCost = true;
          }
          if (TEST) {
            cout <<" pass marker @(" <<startX <<", " <<i <<", " <<startZ <<") N" <<endl;
          }
        } else {
          prevHasCost = false;
        }
      }
      if (enableOutput) {
        cout <<" write vert pathseg (" 
             <<startLoc.x() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() <<", " 
             <<startLoc.y() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() <<") (" 
             <<endLoc.x()   * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() <<", " 
             <<endLoc.y()   * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() <<") " 
             <<getTech()->getLayer(currLayerNum)->getName() <<endl;
      }
    // via
    } else if (startX == endX && startY == endY && startZ != endZ) {
      for (auto currZ = startZ; currZ < endZ; ++currZ) {
        frPoint loc;
        frLayerNum startLayerNum = gridGraph.getLayerNum(currZ);
        //frLayerNum endLayerNum = gridGraph.getLayerNum(currZ + 1);
        gridGraph.getPoint(loc, startX, startY);
        FlexMazeIdx mi(startX, startY, currZ); 
        auto cutLayerDefaultVia = getTech()->getLayer(startLayerNum + 1)->getDefaultViaDef();
        if (gridGraph.isSVia(startX, startY, currZ)) {
          cutLayerDefaultVia = apSVia.find(mi)->second->getAccessViaDef();
        }
        auto currVia = make_unique<drVia>(cutLayerDefaultVia);
        currVia->setOrigin(loc);
        currVia->setMazeIdx(FlexMazeIdx(startX, startY, currZ), FlexMazeIdx(startX, startY, currZ+1));
        unique_ptr<drConnFig> tmp(std::move(currVia));
        workerRegionQuery.add(tmp.get());
        net->addRoute(tmp);
        if (gridGraph.hasDRCCost(startX, startY, currZ, frDirEnum::U)) {
          net->addMarker();
          if (TEST) {
            cout <<" pass marker @(" <<startX <<", " <<startY <<", " <<currZ <<") U" <<endl;
          }
        }
        if (enableOutput) {
          cout <<" write via (" 
               <<loc.x() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() <<", " 
               <<loc.y() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() <<") " 
               <<cutLayerDefaultVia->getName() <<endl;
        }
      }
    // zero length
    } else if (startX == endX && startY == endY && startZ == endZ) {
      std::cout << "Warning: zero-length path in updateFlexPin\n";
    } else {
      std::cout << "Error: non-colinear path in updateFlexPin\n";
    }
  }
}

void FlexDRWorker::routeNet_postRouteAddPathCost(drNet* net) {
  int cnt = 0;
  for (auto &connFig: net->getRouteConnFigs()) {
    addPathCost(connFig.get());
    cnt++;
  }
  //cout <<"updated " <<cnt <<" connfig costs" <<endl;
}

bool FlexDRWorker::routeNet(drNet* net) {
  if (net->getPins().size() <= 1) {
    return true;
  }
  
  if (TEST) {
    cout <<"route " <<net->getFrNet()->getName() <<endl;
  }

  set<drPin*, frBlockObjectComp> unConnPins;
  map<FlexMazeIdx, set<drPin*, frBlockObjectComp> > mazeIdx2unConnPins;
  //map<FlexMazeIdx, frViaDef*> apSVia;
  routeNet_prep(net, unConnPins, mazeIdx2unConnPins/*, apSVia*/);

  FlexMazeIdx ccMazeIdx1, ccMazeIdx2; // connComps ll, ur flexmazeidx
  vector<FlexMazeIdx> connComps;
  routeNet_setSrc(unConnPins, mazeIdx2unConnPins, connComps, ccMazeIdx1, ccMazeIdx2);

  vector<FlexMazeIdx> path; // astar must return with >= 1 idx
  bool isFirstConn = true;
  while(!unConnPins.empty()) {
    mazePinInit();
    auto nextPin = routeNet_getNextDst(ccMazeIdx1, ccMazeIdx2, mazeIdx2unConnPins);
    path.clear();
    if (gridGraph.search(connComps, nextPin, path, ccMazeIdx1, ccMazeIdx2)) {
      routeNet_postAstarUpdate(path, connComps, unConnPins, mazeIdx2unConnPins, isFirstConn);
      routeNet_postAstarWritePath(net, path/*, apSVia*/);
      routeNet_postAstarPatchMinAreaVio(net, path);
      isFirstConn = false;
    }
  }
  routeNet_postRouteAddPathCost(net);
  return true;
}

void FlexDRWorker::routeNet_postAstarPatchMinAreaVio(drNet* net, const vector<FlexMazeIdx> &path) {
  if (path.empty()) {
    return;
  }
  // get path with separated (stacked vias)
  vector<FlexMazeIdx> points;
  for (int i = 0; i < (int)path.size() - 1; ++i) {
    auto currIdx = path[i];
    auto nextIdx = path[i+1];
    if (currIdx.z() == nextIdx.z()) {
      points.push_back(currIdx);
    } else {
      if (currIdx.z() < nextIdx.z()) {
        for (auto z = currIdx.z(); z < nextIdx.z(); ++z) {
          FlexMazeIdx tmpIdx(currIdx.x(), currIdx.y(), z);
          points.push_back(tmpIdx);
        }
      } else {
        for (auto z = currIdx.z(); z > nextIdx.z(); --z) {
          FlexMazeIdx tmpIdx(currIdx.x(), currIdx.y(), z);
          points.push_back(tmpIdx);
        }
      }
    }
  }
  points.push_back(path.back());


  auto layerNum = gridGraph.getLayerNum(points.front().z());
  auto minAreaConstraint = getDesign()->getTech()->getLayer(layerNum)->getAreaConstraint();
  frCoord currArea = (minAreaConstraint) ? minAreaConstraint->getMinArea() : 0;
  frCoord startViaHalfEncArea = 0, endViaHalfEncArea = 0;
  FlexMazeIdx prevIdx = points[0], currIdx;
  for (int i = 1; i < (int)points.size(); ++i) {
    currIdx = points[i];
    // check minAreaViolation when change layer
    if (currIdx.z() != prevIdx.z()) {
      auto layerNum = gridGraph.getLayerNum(prevIdx.z());
      auto minAreaConstraint = getDesign()->getTech()->getLayer(layerNum)->getAreaConstraint();
      frCoord reqArea = (minAreaConstraint) ? minAreaConstraint->getMinArea() : 0;
      // add next via enclosure
      if (currIdx.z() < prevIdx.z()) {
        currArea += gridGraph.getHalfViaEncArea(prevIdx.z() - 1, false);
        endViaHalfEncArea = gridGraph.getHalfViaEncArea(prevIdx.z() - 1, false);
      } else {
        currArea += gridGraph.getHalfViaEncArea(prevIdx.z(), true);
        endViaHalfEncArea = gridGraph.getHalfViaEncArea(prevIdx.z(), true);
      }
      // push to minArea violation
      if (currArea < reqArea) {
        FlexMazeIdx bp, ep;
        frCoord gapArea = reqArea - (currArea - startViaHalfEncArea - endViaHalfEncArea) - std::min(startViaHalfEncArea, endViaHalfEncArea);
        // bp = std::min(prevIdx, currIdx);
        // ep = std::max(prevIdx, currIdx);
        if (points[i-1].z() == points[i-2].z()) {
          bp = std::min(points[i-1], points[i-2]);
          ep = std::max(points[i-1], points[i-2]);
        } else {
          bp = points[i-1];
          ep = points[i-1];
        }


        auto patchWidth = getDesign()->getTech()->getLayer(layerNum)->getWidth();
        // FlexDRMinAreaVio minAreaVio(net, bp, ep, reqArea - (currArea - startViaHalfEncArea) - std::min(startViaHalfEncArea, endViaHalfEncArea));
        // minAreaVios.push_back(minAreaVio);
        routeNet_postAstarAddPatchMetal(net, bp, ep, gapArea, patchWidth);
      }
      // init for next path
      if (currIdx.z() < prevIdx.z()) {
        currArea = gridGraph.getHalfViaEncArea(prevIdx.z() - 1, true);
        startViaHalfEncArea = gridGraph.getHalfViaEncArea(prevIdx.z() - 1, true);
      } else {
        currArea = gridGraph.getHalfViaEncArea(prevIdx.z(), false);
        startViaHalfEncArea = gridGraph.getHalfViaEncArea(prevIdx.z(), false);
      }
    } 
    // add the wire area
    else {
      auto layerNum = gridGraph.getLayerNum(prevIdx.z());
      auto pathWidth = getDesign()->getTech()->getLayer(layerNum)->getWidth();
      frPoint bp, ep;
      gridGraph.getPoint(bp, prevIdx.x(), prevIdx.y());
      gridGraph.getPoint(ep, currIdx.x(), currIdx.y());
      frCoord pathLength = abs(bp.x() - ep.x()) + abs(bp.y() - ep.y());
      currArea += pathLength * pathWidth;
    }
    prevIdx = currIdx;
  }
}

// void FlexDRWorker::routeNet_postRouteAddPatchMetalCost(drNet* net) {
//   for (auto &patch: net->getRoutePatchConnFigs()) {
//     addPathCost(patch.get());
//   }
// }

void FlexDRWorker::routeNet_postAstarAddPatchMetal(drNet* net, 
                                                   const FlexMazeIdx bpIdx,
                                                   const FlexMazeIdx epIdx,
                                                   const frCoord gapArea,
                                                   const frCoord patchWidth) {
  bool isPatchHorz;
  bool isLeftClean = true;
  frLayerNum layerNum = gridGraph.getLayerNum(bpIdx.z());
  frCoord patchLength = gapArea / patchWidth;
  if (gapArea % patchWidth != 0) {
    ++patchLength;
  }
  frPoint origin;
  auto &workerRegionQuery = getWorkerRegionQuery();
  // stacked via
  if (bpIdx.x() == epIdx.x() && bpIdx.y() == epIdx.y()) {
    if (getDesign()->getTech()->getLayer(layerNum)->getDir() == frcHorzPrefRoutingDir) {
      isPatchHorz = true;
    } else {
      isPatchHorz = false;
    }
  }
  // vertical patch 
  else if (bpIdx.x() == epIdx.x()) {
    isPatchHorz = false;
  }
  // horizontal patch
  else {
    isPatchHorz = true;
  }

  // try bottom / left option
  if (isPatchHorz) {
    gridGraph.getPoint(origin, bpIdx.x(), bpIdx.y());
    frPoint patchEnd(origin.x() - patchLength, origin.y());
    if (!getRouteBox().contains(patchEnd)) {
      isLeftClean = false;
    } else {
      frPoint patchLL(origin.x() - patchLength, origin.y() - patchWidth / 2);
      frPoint patchUR(origin.x(), origin.y() + patchWidth / 2);
      FlexMazeIdx startIdx, endIdx;
      gridGraph.getMazeIdx(startIdx, patchLL, layerNum);
      gridGraph.getMazeIdx(endIdx, patchUR, layerNum);
      for (auto xIdx = startIdx.x(); xIdx < endIdx.x(); ++xIdx) {
        for (auto yIdx = startIdx.y(); yIdx < endIdx.y(); ++yIdx) {
          if (gridGraph.hasDRCCost(xIdx, yIdx, bpIdx.z(), frDirEnum::E) ||
              gridGraph.hasDRCCost(xIdx, yIdx, bpIdx.z(), frDirEnum::N)) {
            isLeftClean = false;
            break;
          }
        }
      }
    }
    // add patch if clean
    if (isLeftClean) {
      frPoint patchLL(origin.x() - patchLength, origin.y() - patchWidth / 2);
      frPoint patchUR(origin.x(), origin.y() + patchWidth / 2);
      auto tmpPatch = make_unique<drPatchWire>();
      tmpPatch->setLayerNum(layerNum);
      tmpPatch->setOrigin(origin);
      // tmpPatch->setBBox(frBox(patchLL, patchUR));
      tmpPatch->setOffsetBox(frBox(-patchLength, -patchWidth / 2, 0, patchWidth / 2));
      tmpPatch->addToNet(net);
      unique_ptr<drConnFig> tmp(std::move(tmpPatch));
      workerRegionQuery.add(tmp.get());
      net->addRoute(tmp);
    }
  } else {
    gridGraph.getPoint(origin, bpIdx.x(), bpIdx.y());
    frPoint patchEnd(origin.x(), origin.y() - patchLength);
    if (!getRouteBox().contains(patchEnd)) {
      isLeftClean = false;
    } else {
      frPoint patchLL(origin.x() - patchWidth / 2, origin.y() - patchLength);
      frPoint patchUR(origin.x() + patchWidth / 2, origin.y());
      FlexMazeIdx startIdx, endIdx;
      gridGraph.getMazeIdx(startIdx, patchLL, layerNum);
      gridGraph.getMazeIdx(endIdx, patchUR, layerNum);
      for (auto xIdx = startIdx.x(); xIdx < endIdx.x(); ++xIdx) {
        for (auto yIdx = startIdx.y(); yIdx < endIdx.y(); ++yIdx) {
          if (gridGraph.hasDRCCost(xIdx, yIdx, bpIdx.z(), frDirEnum::E) ||
              gridGraph.hasDRCCost(xIdx, yIdx, bpIdx.z(), frDirEnum::N)) {
            isLeftClean = false;
            break;
          }
        }
      }
    }
    // add patch if clean
    if (isLeftClean) {
      frPoint patchLL(origin.x() - patchWidth / 2, origin.y() - patchLength);
      frPoint patchUR(origin.x() + patchWidth / 2, origin.y());
      auto tmpPatch = make_unique<drPatchWire>();
      tmpPatch->setLayerNum(layerNum);
      tmpPatch->setOrigin(origin);
      // tmpPatch->setBBox(frBox(patchLL, patchUR));
      tmpPatch->setOffsetBox(frBox(-patchWidth / 2, -patchLength, patchWidth / 2, 0));
      tmpPatch->addToNet(net);
      unique_ptr<drConnFig> tmp(std::move(tmpPatch));
      workerRegionQuery.add(tmp.get());
      net->addRoute(tmp);
    }
  }
  // use top / right option if bottom / left is not usable
  if (!isLeftClean) {
    gridGraph.getPoint(origin, epIdx.x(), epIdx.y());
    if (isPatchHorz) {
      frPoint patchLL(origin.x(), origin.y() - patchWidth / 2);
      frPoint patchUR(origin.x() + patchLength, origin.y() + patchWidth / 2);

      auto tmpPatch = make_unique<drPatchWire>();
      tmpPatch->setLayerNum(layerNum);
      tmpPatch->setOrigin(origin);
      // tmpPatch->setBBox(frBox(patchLL, patchUR));
      tmpPatch->setOffsetBox(frBox(0, -patchWidth / 2, patchLength, patchWidth / 2));
      tmpPatch->addToNet(net);
      unique_ptr<drConnFig> tmp(std::move(tmpPatch));
      workerRegionQuery.add(tmp.get());
      net->addRoute(tmp);
    } else {
      frPoint patchLL(origin.x() - patchWidth / 2, origin.y());
      frPoint patchUR(origin.x() + patchWidth / 2, origin.y() + patchLength);

      auto tmpPatch = make_unique<drPatchWire>();
      tmpPatch->setLayerNum(layerNum);
      tmpPatch->setOrigin(origin);
      // tmpPatch->setBBox(frBox(patchLL, patchUR));
      tmpPatch->setOffsetBox(frBox(-patchWidth / 2, 0, patchWidth / 2, patchLength));
      tmpPatch->addToNet(net);
      unique_ptr<drConnFig> tmp(std::move(tmpPatch));
      workerRegionQuery.add(tmp.get());
      net->addRoute(tmp);
    }
  }

}
