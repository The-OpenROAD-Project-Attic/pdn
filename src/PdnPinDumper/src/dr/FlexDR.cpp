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

#include <chrono>
#include <fstream>
#include <boost/io/ios_state.hpp>
//#include <taskflow/taskflow.hpp>
#include "dr/FlexDR.h"
#include "io/io.h"
#include "db/infra/frTime.h"
//#include <omp.h>

using namespace std;
using namespace fr;

// std::chrono::duration<double> time_span_init(0);
// std::chrono::duration<double> time_span_init0(0);
// std::chrono::duration<double> time_span_init1(0);
// std::chrono::duration<double> time_span_init2(0);
// std::chrono::duration<double> time_span_init3(0);
// std::chrono::duration<double> time_span_route(0);
// std::chrono::duration<double> time_span_end(0);

int FlexDRWorker::main() {
  using namespace std::chrono;
  high_resolution_clock::time_point t0 = high_resolution_clock::now();
  if (VERBOSE > 1) {
    frBox scaledBox;
    stringstream ss;
    ss <<endl <<"start DR worker (BOX) "
                <<"( " <<routeBox.left()   * 1.0 / getTech()->getDBUPerUU() <<" "
                <<routeBox.bottom() * 1.0 / getTech()->getDBUPerUU() <<" ) ( "
                <<routeBox.right()  * 1.0 / getTech()->getDBUPerUU() <<" "
                <<routeBox.top()    * 1.0 / getTech()->getDBUPerUU() <<" )" <<endl;
    cout <<ss.str() <<flush;
  }
  //return 0;

  init();
  //FlexMazeRoute mazeInst(getDesign(), routeBox, extBox);
  //mazeInst.setInitDR(isInitDR());
  //mazeInst.init();
  high_resolution_clock::time_point t1 = high_resolution_clock::now();
  //mazeInst.route();
  route();
  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  end();
  //mazeInst.end();
  high_resolution_clock::time_point t3 = high_resolution_clock::now();

  duration<double> time_span0 = duration_cast<duration<double>>(t1 - t0);
  //time_span_init  += time_span0;
  duration<double> time_span1 = duration_cast<duration<double>>(t2 - t1);
  //time_span_route += time_span1;
  duration<double> time_span2 = duration_cast<duration<double>>(t3 - t2);
  //time_span_end   += time_span2;

  if (VERBOSE > 1) {
    stringstream ss;
    ss   <<"time (INIT/ROUTE/POST) " <<time_span0.count() <<" " 
                                     <<time_span1.count() <<" "
                                     <<time_span2.count() <<" "
                                     <<endl;
    cout <<ss.str() <<flush;
  }
  return 0;
}

void FlexDR::initFromTA() {
  bool enableOutput = false;
  // initialize lists
  int cnt = 0;
  for (auto &net: getDesign()->getTopBlock()->getNets()) {
    for (auto &guide: net->getGuides()) {
      for (auto &connFig: guide->getRoutes()) {
        if (connFig->typeId() == frcPathSeg) {
          unique_ptr<frShape> ps = make_unique<frPathSeg>(*(static_cast<frPathSeg*>(connFig.get())));
          frPoint bp, ep;
          static_cast<frPathSeg*>(ps.get())->getPoints(bp, ep);
          if (ep.x() - bp.x() + ep.y() - bp.y() == 1) {
            ; // skip TA dummy segment
          } else {
            net->addShape(ps);
          }
        } else {
          cout <<"Error: initFromTA unsupported shape" <<endl;
        }
      }
    }
    //net->clearGuides(); // should not clear guide because of initGCellBoundary
    cnt++;
    //if (cnt < 100000) {
    //  if (cnt % 10000 == 0) {
    //    cout <<"  initFromTA complete " <<cnt <<" nets" <<endl;
    //  }
    //} else {
    //  if (cnt % 100000 == 0) {
    //    cout <<"  initFromTA complete " <<cnt <<" nets" <<endl;
    //  }
    //}
  }
  //getRegionQuery()->clearGuides(); // should not clear guide because of initGCellBoundary

  if (enableOutput) {
    for (auto &net: getDesign()->getTopBlock()->getNets()) {
      cout <<"net " <<net->getName() <<" has " <<net->getShapes().size() <<" shape" <<endl;
    }
  }
}

void FlexDR::initGCell2BoundaryPin() {
  bool enableOutput = false;
  // initiailize size
  frBox dieBox;
  getDesign()->getTopBlock()->getBoundaryBBox(dieBox);
  auto gCellPatterns = getDesign()->getTopBlock()->getGCellPatterns();
  auto &xgp = gCellPatterns.at(0);
  auto &ygp = gCellPatterns.at(1);
  //frCoord GCELLGRIDX   = xgp.getSpacing();
  //frCoord GCELLGRIDY   = ygp.getSpacing();
  //frCoord GCELLOFFSETX = xgp.getStartCoord();
  //frCoord GCELLOFFSETY = ygp.getStartCoord();
  //frCoord GCELLCNTX    = xgp.getCount();
  //frCoord GCELLCNTY    = ygp.getCount();
  auto tmpVec = vector<map<frNet*, set<pair<frPoint, frLayerNum> >, frBlockObjectComp> >((int)ygp.getCount());
  gcell2BoundaryPin = vector<vector<map<frNet*, set<pair<frPoint, frLayerNum> >, frBlockObjectComp> > >((int)xgp.getCount(), tmpVec);
  for (auto &net: getDesign()->getTopBlock()->getNets()) {
    auto netPtr = net.get();
    for (auto &guide: net->getGuides()) {
      for (auto &connFig: guide->getRoutes()) {
        if (connFig->typeId() == frcPathSeg) {
          auto ps = static_cast<frPathSeg*>(connFig.get());
          frLayerNum layerNum;
          frPoint bp, ep;
          ps->getPoints(bp, ep);
          layerNum = ps->getLayerNum();
          // skip TA dummy segment
          if (ep.x() - bp.x() + ep.y() - bp.y() == 1 || ep.x() - bp.x() + ep.y() - bp.y() == 0) {
            continue; 
          }
          frPoint idx1, idx2;
          getDesign()->getTopBlock()->getGCellIdx(bp, idx1);
          getDesign()->getTopBlock()->getGCellIdx(ep, idx2);
          //frBox gcellBox1, gcellBox2;
          //getDesign()->getTopBlock()->getGCellBox(idx1, gcellBox1);
          //getDesign()->getTopBlock()->getGCellBox(idx2, gcellBox2);
          // update gcell2BoundaryPin
          // horizontal
          if (bp.y() == ep.y()) {
            //int x1 = (bp.x() - xgp.getStartCoord()) / xgp.getSpacing();
            //int x2 = (ep.x() - xgp.getStartCoord()) / xgp.getSpacing();
            //int y  = (bp.y() - ygp.getStartCoord()) / ygp.getSpacing();
            int x1 = idx1.x();
            int x2 = idx2.x();
            int y  = idx1.y();
            //if (x1 < 0 || x2 < 0 || y < 0) {
            //  cout <<"Warning: initGCell2BoundaryPin < 0";
            //  exit(1);
            //}
            //if (x1 >= (int)GCELLCNTX) {
            //  x1 = (int)GCELLCNTX - 1;
            //}
            //if (x2 >= (int)GCELLCNTX) {
            //  x2 = (int)GCELLCNTX - 1;
            //}
            //if (y >= (int)GCELLCNTY) {
            //  y = (int)GCELLCNTY - 1;
            //}
            for (auto x = x1; x <= x2; ++x) {
              frBox gcellBox;
              getDesign()->getTopBlock()->getGCellBox(frPoint(x, y), gcellBox);
              //frCoord leftBound  = x * xgp.getSpacing() + xgp.getStartCoord();
              //frCoord rightBound = (x == (int)GCELLCNTX - 1) ? dieBox.right() : (x + 1) * xgp.getSpacing() + xgp.getStartCoord();
              frCoord leftBound  = gcellBox.left();
              frCoord rightBound = gcellBox.right();
              bool hasLeftBound  = true;
              bool hasRightBound = true;
              if (bp.x() < leftBound) {
                hasLeftBound = true;
              } else {
                hasLeftBound = false;
              }
              if (ep.x() >= rightBound) {
                hasRightBound = true;
              } else {
                hasRightBound = false;
              }
              if (hasLeftBound) {
                frPoint boundaryPt(leftBound, bp.y());
                gcell2BoundaryPin[x][y][netPtr].insert(make_pair(boundaryPt, layerNum));
                if (enableOutput) {
                  cout << "init left boundary pin (" 
                       << boundaryPt.x() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() << ", " 
                       << boundaryPt.y() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() << ") at ("
                       << x <<", " <<y <<") " 
                       << getTech()->getLayer(layerNum)->getName() <<" "
                       << string((net == nullptr) ? "null" : net->getName()) <<"\n";
                }
              }
              if (hasRightBound) {
                frPoint boundaryPt(rightBound, ep.y());
                gcell2BoundaryPin[x][y][netPtr].insert(make_pair(boundaryPt, layerNum));
                if (enableOutput) {
                  cout << "init right boundary pin (" 
                       << boundaryPt.x() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() << ", " 
                       << boundaryPt.y() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() << ") at ("
                       << x <<", " <<y <<") " 
                       << getTech()->getLayer(layerNum)->getName() <<" "
                       << string((net == nullptr) ? "null" : net->getName()) <<"\n";
                }
              }
            }
          } else if (bp.x() == ep.x()) {
            //int x  = (bp.x() - xgp.getStartCoord()) / xgp.getSpacing();
            //int y1 = (bp.y() - ygp.getStartCoord()) / ygp.getSpacing();
            //int y2 = (ep.y() - ygp.getStartCoord()) / ygp.getSpacing();
            int x  = idx1.x();
            int y1 = idx1.y();
            int y2 = idx2.y();
            //if (y1 < 0 || y2 < 0 || x < 0) {
            //  cout <<"Warning: initGCell2BoundaryPin < 0";
            //  exit(1);
            //}
            //if (x >= (int)GCELLCNTX) {
            //  x = (int)GCELLCNTX - 1;
            //}
            //if (y1 >= (int)GCELLCNTY) {
            //  y1 = (int)GCELLCNTY - 1;
            //}
            //if (y2 >= (int)GCELLCNTY) {
            //  y2 = (int)GCELLCNTY - 1;
            //}
            for (auto y = y1; y <= y2; ++y) {
              frBox gcellBox;
              getDesign()->getTopBlock()->getGCellBox(frPoint(x, y), gcellBox);
              //frCoord bottomBound = y * ygp.getSpacing() + ygp.getStartCoord();
              //frCoord topBound    = (y == (int)GCELLCNTY - 1) ? dieBox.top() : (y + 1) * ygp.getSpacing() + ygp.getStartCoord();
              frCoord bottomBound = gcellBox.bottom();
              frCoord topBound    = gcellBox.top();
              bool hasBottomBound = true;
              bool hasTopBound    = true;
              if (bp.y() < bottomBound) {
                hasBottomBound = true;
              } else {
                hasBottomBound = false;
              }
              if (ep.y() >= topBound) {
                hasTopBound = true;
              } else {
                hasTopBound = false;
              }
              if (hasBottomBound) {
                frPoint boundaryPt(bp.x(), bottomBound);
                gcell2BoundaryPin[x][y][netPtr].insert(make_pair(boundaryPt, layerNum));
                if (enableOutput) {
                  cout << "init bottom boundary pin (" 
                       << boundaryPt.x() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() << ", " 
                       << boundaryPt.y() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() << ") at ("
                       << x <<", " <<y <<") " 
                       << getTech()->getLayer(layerNum)->getName() <<" "
                       << string((net == nullptr) ? "null" : net->getName()) <<"\n";
                }
              }
              if (hasTopBound) {
                frPoint boundaryPt(ep.x(), topBound);
                gcell2BoundaryPin[x][y][netPtr].insert(make_pair(boundaryPt, layerNum));
                if (enableOutput) {
                  cout << "init top boundary pin (" 
                       << boundaryPt.x() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() << ", " 
                       << boundaryPt.y() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() << ") at ("
                       << x <<", " <<y <<") " 
                       << getTech()->getLayer(layerNum)->getName() <<" "
                       << string((net == nullptr) ? "null" : net->getName()) <<"\n";
                }
              }
            }
          } else {
            cout << "Error: non-orthogonal pathseg in initGCell2BoundryPin\n";
          }
        }
      }
    }
  }
}

void FlexDR::init() {
  frTime t;
  if (VERBOSE > 0) {
    cout <<endl <<"start routing data preparation" <<endl;
  }
  //initFromTA();
  initGCell2BoundaryPin();
  //if (VERBOSE > 0) {
  //  cout <<endl <<"init dr objs ..." <<endl;
  //}
  getRegionQuery()->initDRObj(getTech()->getLayers().size());
  //getRegionQuery()->printDRObj();
  if (VERBOSE > 0) {
    t.print();
  }
}

void FlexDR::removeGCell2BoundaryPin() {
  gcell2BoundaryPin.clear();
  gcell2BoundaryPin.shrink_to_fit();
}

map<frNet*, set<pair<frPoint, frLayerNum> >, frBlockObjectComp> FlexDR::initDR_mergeBoundaryPin(int startX, int startY, int size, const frBox &routeBox) {
  map<frNet*, set<pair<frPoint, frLayerNum> >, frBlockObjectComp> bp;
  auto gCellPatterns = getDesign()->getTopBlock()->getGCellPatterns();
  auto &xgp = gCellPatterns.at(0);
  auto &ygp = gCellPatterns.at(1);
  for (int i = startX; i < (int)xgp.getCount() && i < startX + size; i++) {
    for (int j = startY; j < (int)ygp.getCount() && j < startY + size; j++) {
      auto &currBp = gcell2BoundaryPin[i][j];
      for (auto &[net, s]: currBp) {
        for (auto &[pt, lNum]: s) {
          if (pt.x() == routeBox.left()   || pt.x() == routeBox.right() ||
              pt.y() == routeBox.bottom() || pt.y() == routeBox.top()) {
            bp[net].insert(make_pair(pt, lNum));
          }
        }
      }
    }
  }
  return bp;
}

void FlexDR::initDR(int size, bool enableDRC) {
  bool TEST = false;
  //bool TEST = true;
  //cout <<"sizeof listiter   = " <<sizeof(frListIter<frPathSeg>) <<endl;
  //cout <<"sizeof raw ptr    = " <<sizeof(frPathSeg*) <<endl;
  //cout <<"sizeof unique ptr = " <<sizeof(unique_ptr<frPathSeg>) <<endl;
  //cout <<"sizeof shared_ptr = " <<sizeof(shared_ptr<frPathSeg>) <<endl;
  //exit(0);

  //FlexGridGraph gg(getTech(), getDesign());
  ////frBox testBBox(225000, 228100, 228000, 231100); // net1702 in ispd19_test1
  //frBox testBBox(0, 0, 2000, 2000); // net1702 in ispd19_test1
  ////gg.setBBox(testBBox);
  //gg.init(testBBox);
  //gg.print();
  //exit(1);

  frTime t;

  if (VERBOSE > 0) {
    cout <<endl <<"start initial detail routing ..." <<endl;
  }
  frBox dieBox;
  getDesign()->getTopBlock()->getBoundaryBBox(dieBox);

  auto gCellPatterns = getDesign()->getTopBlock()->getGCellPatterns();
  auto &xgp = gCellPatterns.at(0);
  auto &ygp = gCellPatterns.at(1);

  int numQuickMarkers = 0;
  if (TEST) {
    FlexDRWorker worker(getDesign());
    //frBox routeBox;
    //routeBox.set(0*2000, 0*2000, 1*2000, 1*2000);
    frCoord xl = 0 * 2000;
    frCoord yl = 45.6 * 2000;
    //frCoord xh = 129 * 2000;
    //frCoord yh = 94.05 * 2000;
    frPoint idx;
    getDesign()->getTopBlock()->getGCellIdx(frPoint(xl, yl), idx);
    if (VERBOSE > 1) {
      cout <<"(i,j) = (" <<idx.x() <<", " <<idx.y() <<")" <<endl;
    }
    //getDesign()->getTopBlock()->getGCellBox(idx, routeBox);
    frBox routeBox1;
    getDesign()->getTopBlock()->getGCellBox(frPoint(idx.x(), idx.y()), routeBox1);
    frBox routeBox2;
    getDesign()->getTopBlock()->getGCellBox(frPoint(min((int)xgp.getCount() - 1, idx.x() + size-1), 
                                                    min((int)ygp.getCount(), idx.y() + size-1)), routeBox2);
    frBox routeBox(routeBox1.left(), routeBox1.bottom(), routeBox2.right(), routeBox2.top());
    auto bp = initDR_mergeBoundaryPin(idx.x(), idx.y(), size, routeBox);
    //routeBox.set(129*2000, 94.05*2000, 132*2000, 96.9*2000);
    worker.setRouteBox(routeBox);
    frBox extBox;
    routeBox.bloat(1000, extBox);
    worker.setRouteBox(routeBox);
    worker.setExtBox(extBox);
    //int i = (129   * 2000 - xgp.getStartCoord()) / xgp.getSpacing();
    //int j = (94.05 * 2000 - ygp.getStartCoord()) / ygp.getSpacing();
    //worker.setDRIter(0, gcell2BoundaryPin[idx.x()][idx.y()]);
    worker.setDRIter(0, bp);
    worker.setEnableDRC(enableDRC);
    worker.main();
    cout <<"done"  <<endl <<flush;
  } else {
    //vector<FlexDRWorker> workers;
    int cnt = 0;
    //int tot = (int)xgp.getCount() * (int)ygp.getCount();
    int tot = (((int)xgp.getCount() - 1) / size + 1) * (((int)ygp.getCount() - 1) / size + 1);
    int prev_perc = 0;
    bool isExceed = false;
    for (int i = 0; i < (int)xgp.getCount(); i += size) {
      for (int j = 0; j < (int)ygp.getCount(); j += size) {
    //for (int i = 310; i < 330; i++) {
    //  for (int j = 300; j < 320; j++) {
        FlexDRWorker worker(getDesign());
        frBox routeBox1;
        getDesign()->getTopBlock()->getGCellBox(frPoint(i, j), routeBox1);
        frBox routeBox2;
        getDesign()->getTopBlock()->getGCellBox(frPoint(min((int)xgp.getCount() - 1, i + size-1), 
                                                        min((int)ygp.getCount(), j + size-1)), routeBox2);
        //frBox routeBox;
        frBox routeBox(routeBox1.left(), routeBox1.bottom(), routeBox2.right(), routeBox2.top());
        //getDesign()->getTopBlock()->getGCellBox(frPoint(i, j), routeBox);
        //if (!(routeBox.left()   >= 504 * 2000  && routeBox.right() <= 525*2000 &&
        //    routeBox.bottom() >= 538.65*2000 && routeBox.top()   <= 558.6*2000)) {
        //  continue;
        //}
        frBox extBox;
        routeBox.bloat(1000, extBox);
        worker.setRouteBox(routeBox);
        worker.setExtBox(extBox);
        //workers.push_back(worker);
        //worker.setDRIter(0, &gcell2BoundaryPin[i][j]);
        auto bp = initDR_mergeBoundaryPin(i, j, size, routeBox);
        worker.setDRIter(0, bp);
        // set boundary pin
        worker.setEnableDRC(enableDRC);
        worker.main();
        numQuickMarkers += worker.getNumQuickMarkers();
        //cout <<"i/j = " <<i <<", " <<j <<endl;
        cnt++;
        if (VERBOSE > 0) {
          if (cnt * 1.0 / tot >= prev_perc / 100.0 + 0.1) {
            if (prev_perc == 0 && t.isExceed(2)) {
              isExceed = true;
            }
            prev_perc += 10;
            //if (true) {
            if (isExceed) {
              if (enableDRC) {
                cout <<"    completing " <<prev_perc <<"% with " <<getDesign()->getTopBlock()->getNumMarkers() <<" violations" <<endl;
              } else {
                cout <<"    completing " <<prev_perc <<"% with " <<numQuickMarkers <<" quick violations" <<endl;
              }
              cout <<"    " <<t <<endl <<flush;
            }
          }
        }
        //if (cnt % 10000 == 0) {
        //  cout <<"    complete " <<cnt <<"/" <<tot <<endl;
        //  cout <<"    " <<t <<endl;
        //  //cout <<"  completing XX% with YY violations" <<endl;
        //  //cout <<"  elapsed time = 00:04:55, memory = ZZZZ.ZZ (MB)" <<endl;
        //}
      }
    }
  }

  //cout <<"  number of violations = " <<numMarkers <<endl;
  removeGCell2BoundaryPin();
  checkConnectivity();

  if (VERBOSE > 0) {
    if (enableDRC) {
      cout <<"  number of violations = "       <<getDesign()->getTopBlock()->getNumMarkers() <<endl;
    } else {
      cout <<"  number of quick violations = " <<numQuickMarkers <<endl;
    }
    //cout <<"    by layer and type :" <<endl;
    //cout <<"           MetSpc EOLSpc Loop CutSpc AdjCut CorSpc Others Totals" <<endl;
    //cout <<"    Metal1      0      0    0      0      0      0      0      0" <<endl;
    //cout <<"    Totals      0      0    0      0      0      0      0      0" <<endl;
    //cout <<"cpu time = 00:00:00, elapsed time = 00:00:00, memory = 0000.00 (MB), peak = 0000.00 (MB)" <<endl;
    t.print();
    cout <<flush;
  }
}

void FlexDR::searchRepair(int iter, int size, int offset, int mazeEndIter, 
                          frUInt4 workerDRCCost, frUInt4 workerMarkerCost,
                          bool enableDRC, int ripupMode, bool TEST) {
  frTime t;
  //bool TEST = false;
  //bool TEST = true;
  if (VERBOSE > 0) {
    cout <<endl <<"start " <<iter;
    string suffix;
    if (iter == 1 || (iter > 20 && iter % 10 == 1)) {
      suffix = "st";
    } else if (iter == 2 || (iter > 20 && iter % 10 == 2)) {
      suffix = "nd";
    } else if (iter == 3 || (iter > 20 && iter % 10 == 3)) {
      suffix = "rd";
    } else {
      suffix = "th";
    }
    cout <<suffix <<" optimization iteration ..." <<endl;
  }
  frBox dieBox;
  getDesign()->getTopBlock()->getBoundaryBBox(dieBox);
  //getRegionQuery()->initDRObj(getTech()->getLayers().size());
  //getRegionQuery()->printDRObj();
  auto gCellPatterns = getDesign()->getTopBlock()->getGCellPatterns();
  auto &xgp = gCellPatterns.at(0);
  auto &ygp = gCellPatterns.at(1);
  int numQuickMarkers = 0;
  if (TEST) {
    cout <<"search and repair test mode" <<endl <<flush;
    FlexDRWorker worker(getDesign());
    frBox routeBox;
    //frCoord xl = 148.5 * 2000;
    //frCoord yl = 570 * 2000;
    //frPoint idx;
    //getDesign()->getTopBlock()->getGCellIdx(frPoint(xl, yl), idx);
    //if (VERBOSE > 1) {
    //  cout <<"(i,j) = (" <<idx.x() <<", " <<idx.y() <<")" <<endl;
    //}
    //getDesign()->getTopBlock()->getGCellBox(idx, routeBox);
    //routeBox.set(156*2000, 108.3*2000, 177*2000, 128.25*2000);
    // routeBox.set(175*2000, 3.5*2000, 185*2000, 13.5*2000);
    routeBox.set(0*2000, 0*2000, 200*2000, 200*2000);
    worker.setRouteBox(routeBox);
    frBox extBox;
    routeBox.bloat(1000, extBox);
    worker.setRouteBox(routeBox);
    worker.setExtBox(extBox);
    worker.setMazeEndIter(mazeEndIter);
    // worker.setTest(true);
    worker.setTest(false);
    worker.setDRCTest(true);
    worker.setDRIter(iter);
    worker.setEnableDRC(enableDRC);
    worker.setRipupMode(ripupMode);
    worker.setCost(workerDRCCost, workerMarkerCost);
    worker.main();
    numQuickMarkers += worker.getNumQuickMarkers();
    cout <<"done"  <<endl <<flush;
  } else {
    //vector<FlexDRWorker> workers;
    int clipSize = size;
    int cnt = 0;
    int tot = (((int)xgp.getCount() - 1 - offset) / clipSize + 1) * (((int)ygp.getCount() - 1 - offset) / clipSize + 1);
    int prev_perc = 0;
    bool isExceed = false;
    for (int i = offset; i < (int)xgp.getCount(); i += clipSize) {
      for (int j = offset; j < (int)ygp.getCount(); j += clipSize) {
    //for (int i = 312; i < 330; i += 3) {
    //  for (int j = 300; j < 318; j += 3) {
        FlexDRWorker worker(getDesign());
        frBox routeBox1;
        getDesign()->getTopBlock()->getGCellBox(frPoint(i, j), routeBox1);
        frBox routeBox2;
        getDesign()->getTopBlock()->getGCellBox(frPoint(min((int)xgp.getCount() - 1, i + clipSize-1), 
                                                        min((int)ygp.getCount(), j + clipSize-1)), routeBox2);
        frBox routeBox(routeBox1.left(), routeBox1.bottom(), routeBox2.right(), routeBox2.top());
        frBox extBox;
        routeBox.bloat(1000, extBox);
        worker.setRouteBox(routeBox);
        worker.setExtBox(extBox);
        worker.setMazeEndIter(mazeEndIter);
        worker.setDRIter(iter);
        worker.setEnableDRC(enableDRC);
        worker.setRipupMode(ripupMode);
        worker.setCost(workerDRCCost, workerMarkerCost);
        //workers.push_back(worker);
        worker.main();
        numQuickMarkers += worker.getNumQuickMarkers();
        cnt++;
        if (VERBOSE > 0) {
          if (cnt * 1.0 / tot >= prev_perc / 100.0 + 0.1) {
            if (prev_perc == 0 && t.isExceed(2)) {
              isExceed = true;
            }
            prev_perc += 10;
            //if (true) {
            if (isExceed) {
              if (enableDRC) {
                cout <<"    completing " <<prev_perc <<"% with " <<getDesign()->getTopBlock()->getNumMarkers() <<" violations" <<endl;
              } else {
                cout <<"    completing " <<prev_perc <<"% with " <<numQuickMarkers <<" quick violations" <<endl;
              }
              cout <<"    " <<t <<endl <<flush;
            }
          }
        }
        //if (cnt > 10000 && ) {
        //  cout <<"    completing " <<cnt <<"/" <<tot <<endl;
        //  cout <<"    " <<t <<endl;
        //  //cout <<"  completing XX% with YY violations" <<endl;
        //  //cout <<"  elapsed time = 00:04:55, memory = ZZZZ.ZZ (MB)" <<endl;
        //}
      }
    }
  }
  //cout <<"  number of violations = " <<numMarkers <<endl;
  checkConnectivity();
  if (VERBOSE > 0) {
    if (enableDRC) {
      cout <<"  number of violations = " <<getDesign()->getTopBlock()->getNumMarkers() <<endl;
    } else {
      cout <<"  number of quick violations = " <<numQuickMarkers <<endl;
    }
    //cout <<"    by layer and type :" <<endl;
    //cout <<"           MetSpc EOLSpc Loop CutSpc AdjCut CorSpc Others Totals" <<endl;
    //cout <<"    Metal1      0      0    0      0      0      0      0      0" <<endl;
    //cout <<"    Totals      0      0    0      0      0      0      0      0" <<endl;
    //cout <<"cpu time = 00:00:00, elapsed time = 00:00:00, memory = 0000.00 (MB), peak = 0000.00 (MB)" <<endl;
    t.print();
    cout <<flush;
  }
}

void FlexDR::end() {
  vector<unsigned long long> wlen(getTech()->getLayers().size(), 0);
  vector<unsigned long long> sCut(getTech()->getLayers().size(), 0);
  vector<unsigned long long> mCut(getTech()->getLayers().size(), 0);
  unsigned long long totWlen = 0;
  unsigned long long totSCut = 0;
  unsigned long long totMCut = 0;
  frPoint bp, ep;
  for (auto &net: getDesign()->getTopBlock()->getNets()) {
    for (auto &shape: net->getShapes()) {
      if (shape->typeId() == frcPathSeg) {
        auto obj = static_cast<frPathSeg*>(shape.get());
        obj->getPoints(bp, ep);
        auto lNum = obj->getLayerNum();
        frCoord psLen = ep.x() - bp.x() + ep.y() - bp.y();
        wlen[lNum] += psLen;
        totWlen += psLen;
      }
    }
    for (auto &via: net->getVias()) {
      auto lNum = via->getViaDef()->getCutLayerNum();
      if (via->getViaDef()->isMultiCut()) {
        ++mCut[lNum];
        ++totMCut;
      } else {
        ++sCut[lNum];
        ++totSCut;
      }
    }
  }
  if (VERBOSE > 0) {
    boost::io::ios_all_saver guard(std::cout);
    cout <<endl <<"total wire length = " <<totWlen / getDesign()->getTopBlock()->getDBUPerUU() <<" um" <<endl;
    for (int i = getTech()->getBottomLayerNum(); i <= getTech()->getTopLayerNum(); i++) {
      if (getTech()->getLayer(i)->getType() == frLayerTypeEnum::ROUTING) {
        cout <<"total wire length on LAYER " <<getTech()->getLayer(i)->getName() <<" = " 
             <<wlen[i] / getDesign()->getTopBlock()->getDBUPerUU() <<" um" <<endl;
      }
    }
    cout <<"total number of vias = " <<totSCut + totMCut <<endl;
    if (totMCut > 0) {
      cout <<"total number of multi-cut vias = " <<totMCut 
           << " (" <<setw(5) <<fixed <<setprecision(1) <<totMCut * 100.0 / (totSCut + totMCut) <<"%)" <<endl;
      cout <<"total number of single-cut vias = " <<totSCut 
           << " (" <<setw(5) <<fixed <<setprecision(1) <<totSCut * 100.0 / (totSCut + totMCut) <<"%)" <<endl;
    }
    cout <<"up-via summary (total " <<totSCut + totMCut <<"):" <<endl;
    int nameLen = 0;
    for (int i = getTech()->getBottomLayerNum(); i <= getTech()->getTopLayerNum(); i++) {
      if (getTech()->getLayer(i)->getType() == frLayerTypeEnum::CUT) {
        nameLen = max(nameLen, (int)getTech()->getLayer(i-1)->getName().size());
      }
    }
    int maxL = 1 + nameLen + 4 + (int)to_string(totSCut).length();
    if (totMCut) {
      maxL += 9 + 4 + (int)to_string(totMCut).length() + 9 + 4 + (int)to_string(totSCut + totMCut).length();
    }
    if (totMCut) {
      cout <<" " <<setw(nameLen + 4 + (int)to_string(totSCut).length() + 9) <<"single-cut";
      cout <<setw(4 + (int)to_string(totMCut).length() + 9) <<"multi-cut" 
           <<setw(4 + (int)to_string(totSCut + totMCut).length()) <<"total";
    }
    cout <<endl;
    for (int i = 0; i < maxL; i++) {
      cout <<"-";
    }
    cout <<endl;
    for (int i = getTech()->getBottomLayerNum(); i <= getTech()->getTopLayerNum(); i++) {
      if (getTech()->getLayer(i)->getType() == frLayerTypeEnum::CUT) {
        cout <<" "    <<setw(nameLen) <<getTech()->getLayer(i-1)->getName() 
             <<"    " <<setw((int)to_string(totSCut).length()) <<sCut[i];
        if (totMCut) {
          cout <<" ("   <<setw(5) <<(double)((sCut[i] + mCut[i]) ? sCut[i] * 100.0 / (sCut[i] + mCut[i]) : 0.0) <<"%)";
          cout <<"    " <<setw((int)to_string(totMCut).length()) <<mCut[i] 
               <<" ("   <<setw(5) <<(double)((sCut[i] + mCut[i]) ? mCut[i] * 100.0 / (sCut[i] + mCut[i]) : 0.0) <<"%)"
               <<"    " <<setw((int)to_string(totSCut + totMCut).length()) <<sCut[i] + mCut[i];
        }
        cout <<endl;
      }
    }
    for (int i = 0; i < maxL; i++) {
      cout <<"-";
    }
    cout <<endl;
    cout <<" "    <<setw(nameLen) <<""
         <<"    " <<setw((int)to_string(totSCut).length()) <<totSCut;
    if (totMCut) {
      cout <<" ("   <<setw(5) <<(double)((totSCut + totMCut) ? totSCut * 100.0 / (totSCut + totMCut) : 0.0) <<"%)";
      cout <<"    " <<setw((int)to_string(totMCut).length()) <<totMCut 
           <<" ("   <<setw(5) <<(double)((totSCut + totMCut) ? totMCut * 100.0 / (totSCut + totMCut) : 0.0) <<"%)"
           <<"    " <<setw((int)to_string(totSCut + totMCut).length()) <<totSCut + totMCut;
    }
    cout <<endl <<endl <<flush;
    guard.restore();
  }
}

void FlexDR::reportDRC() {
  double dbu = design->getTech()->getDBUPerUU();
  cout << DRC_RPT_FILE << "\n";
  if (DRC_RPT_FILE != string("")) {
    ofstream drcRpt(DRC_RPT_FILE.c_str());
    if (drcRpt.is_open()) {
      for (auto &marker: getDesign()->getTopBlock()->getMarkers()) {
        drcRpt << "  violation type: " << int(marker->getConstraint()->typeId()) << "\n";
        // get source(s) of violation
        drcRpt << "    srcs: ";
        for (auto src: marker->getSrcs()) {
          if (src) {
            switch (src->typeId()) {
              case frcNet:
                drcRpt << (static_cast<frNet*>(src))->getName() << " ";
                break;
              case frcInstTerm: {
                frInstTerm* instTerm = (static_cast<frInstTerm*>(src));
                // drcRpt << instTerm->getInst()->getName() 
                       // << "/" << instTerm->getTerm()->getName() << " ";
                drcRpt << "Pin of Cell " << instTerm->getInst()->getName() << " ";
                break;
              }
              case frcTerm: {
                frTerm* term = (static_cast<frTerm*>(src));
                drcRpt << term->getName() << " ";
                break;
              }
              default:
                std::cout << "Error: unexpected src type in marker\n";
            }
          }
        }
        drcRpt << "\n";
        // get violation bbox
        frBox bbox;
        marker->getBBox(bbox);
        drcRpt << "    bbox = (" << bbox.left() / dbu << ", " << bbox.bottom() / dbu << ") - ("
               << bbox.right() / dbu << ", " << bbox.top() / dbu << ")\n";
      }
    } else {
      cout << "Error: Fail to open DRC report file\n";
    }
  } else {
    cout << "Error: DRC report file is not specified\n";
  }
}


int FlexDR::main() {
  init();
  frTime t;
  if (VERBOSE > 0) {
    cout <<endl <<endl <<"start detail routing ...";
  }
  // initDR: enableDRC
  initDR(7, true);
  //cout   <<endl
  //       <<"time (INIT/ROUTE/POST) " <<time_span_init.count() <<" " 
  //                                   <<time_span_route.count() <<" "
  //                                   <<time_span_end.count() <<" "
  //                                   <<endl;
  //io::Writer writer(getDesign());
  //writer.writeFromDR("_init");
  // search and repair: iter, size, offset, mazeEndIter, workerDRCCost, workerMarkerCost, enableDRC, ripupMode, TEST
  // assume only mazeEndIter > 1 if enableDRC and ripupMode == 0 (partial ripup)
  end();
  searchRepair(1, 7, 3, 1, DRCCOST, 0, true, 1); // func as fully rerouting iter, no marker cost
  //end();
  //searchRepair(2, 7, 0, 1, DRCCOST, 0, true, 1); // func as rerouting iter, no marker cost
  //end();
  //searchRepair(3, 7, 0, 1, DRCCOST, 0, true, 1); // func as rerouting iter, no marker cost
  //end();
  //searchRepair(4, 7, 3, 1, DRCCOST, 0, true, 1); // func as rerouting iter, no marker cost
  end();
  searchRepair(2, 7, 0, 1, DRCCOST, MARKERCOST, true, 0); // true search and repair
  end();
  searchRepair(3, 7, 3, 1, DRCCOST, MARKERCOST, true, 0); // true search and repair
  end();
  searchRepair(4, 7, 0, 1, DRCCOST, MARKERCOST*2, true, 0); // true search and repair
  end();
  searchRepair(5, 7, 3, 1, DRCCOST, MARKERCOST*2, true, 0); // true search and repair
  //reportDRC();
  //end();
  //searchRepair(7, 7, 0, 1, DRCCOST, MARKERCOST*32, true, 0);
  //end();
  //searchRepair(8, 7, 3, 1, DRCCOST, MARKERCOST*32, true, 0);
  //end();
  //searchRepair(4, 11, 5, 1, true);
  //end();
  //searchRepair(5, 13, 0, 1, true);
  //end();
  //searchRepair(6, 13, 7, 1, true);
  //end();
  //searchRepair(7, 17, 0, 1, true);
  //end();
  //searchRepair(8, 17, 9, 1, true);
  //end();
  //searchRepair(3, 7, 1, 4);
  //end();
  //searchRepair(4, 11, 0, 6);
  //end();
  //BOTTOM_ROUTING_LAYER = 2;
  //searchRepair(3, 7, 0, 1, true);
  //end();
  //searchRepair(5, 11, 5, 6);
  //end();
  //searchRepair(4, 17, 0);
  //end();
  //searchRepair(2, 16, 1);
  //end();
  //searchRepair(3, 16, 2);
  //end();
  //searchRepair(4, 16, 3);
  //end();
  //searchRepair(5, 16, 4);
  //for (int i = 1; i <= END_ITERATION; i++) {
  //  end();
  //  searchRepair(i, 5, 0);
  //}
  //omp_set_num_threads(40);
  //#pragma omp parallel for schedule(dynamic)
  //for (int i = 0; i < 999999999; i++) {
  //  auto j = i*i;
  //  if (VERBOSE > 100) {
  //    cout <<j <<endl;
  //  }
  //}
  //time_span_init  = std::chrono::duration<double>(0);
  //time_span_route = std::chrono::duration<double>(0);
  //time_span_end   = std::chrono::duration<double>(0);
  //cout   <<endl 
  //       <<"time (INIT/ROUTE/POST) " <<time_span_init.count() <<" " 
  //                                   <<time_span_route.count() <<" "
  //                                   <<time_span_end.count() <<" "
  //                                   <<endl;

  if (DRC_RPT_FILE != std::string("")) {
    double dbu = getDesign()->getTech()->getDBUPerUU();
    std::ofstream drcRpt(DRC_RPT_FILE.c_str());
    if (drcRpt.is_open()) {
      for (auto &umarker: getDesign()->getTopBlock()->getMarkers()) {
        auto &marker = *umarker;
        drcRpt << "  violation type: " << int(marker.getConstraint()->typeId()) << "\n";
        // get source(s) of violation
        drcRpt << "    srcs: ";
        for (auto src: marker.getSrcs()) {
          if (src) {
            switch (src->typeId()) {
              case frcNet:
                drcRpt << (static_cast<frNet*>(src))->getName() << " ";
                break;
              case frcInstTerm: {
                frInstTerm* instTerm = (static_cast<frInstTerm*>(src));
                // drcRpt << instTerm->getInst()->getName() 
                       // << "/" << instTerm->getTerm()->getName() << " ";
                drcRpt << "Pin of Cell " << instTerm->getInst()->getName() << " ";
                break;
              }
              case frcTerm: {
                frTerm* term = (static_cast<frTerm*>(src));
                drcRpt << term->getName() << " ";
                break;
              }
              default:
                std::cout << "Error: unexpected src type in marker\n";
            }
          }
        }
        drcRpt << "\n";
        // get violation bbox
        frBox bbox;
        marker.getBBox(bbox);
        drcRpt << "    bbox = (" << bbox.left() / dbu << ", " << bbox.bottom() / dbu << ") - ("
               << bbox.right() / dbu << ", " << bbox.top() / dbu << ")\n";
      }
    } else {
      std::cout << "Error: Fail to open DRC report file\n";
    }
  }


  if (VERBOSE > 0) {
    cout <<endl <<"complete detail routing";
    end();
  }
  if (VERBOSE > 0) {
    t.print();
    cout <<endl;
  }
  return 0;
}

