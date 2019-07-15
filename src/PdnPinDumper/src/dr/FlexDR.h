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

#ifndef _FR_FLEXDR_H_
#define _FR_FLEXDR_H_

#include <memory>
#include "frDesign.h"
#include "db/drObj/drNet.h"
#include "db/drObj/drMarker.h"
#include "dr/FlexGridGraph.h"
#include "dr/FlexWavefront.h"

namespace fr {

  class FlexDR {
  public:
    // constructors
    FlexDR(frDesign* designIn): design(designIn) {}
    // getters
    frTechObject* getTech() const {
      return design->getTech();
    }
    frDesign* getDesign() const {
      return design;
    }
    frRegionQuery* getRegionQuery() const {
      return design->getRegionQuery();
    }
    // others
    int main();
  protected:
    frDesign*          design;
    std::vector<std::vector<std::map<frNet*, std::set<std::pair<frPoint, frLayerNum> >, frBlockObjectComp> > > gcell2BoundaryPin;
    // others
    void init();
    void initFromTA();
    void initGCell2BoundaryPin();
    void removeGCell2BoundaryPin();
    void checkConnectivity();
    void checkConnectivity_initDRObjs(frNet* net, std::vector<frConnFig*> &netDRObjs);
    void checkConnectivity_pin2epMap(frNet* net, std::vector<frConnFig*> &netDRObjs,
                                     std::map<frBlockObject*, std::set<std::pair<frPoint, frLayerNum> >, frBlockObjectComp> &pin2epMap);
    void checkConnectivity_pin2epMap_helper(frNet* net, const frPoint &bp, frLayerNum lNum, 
                                            std::map<frBlockObject*, std::set<std::pair<frPoint, frLayerNum> >, frBlockObjectComp> &pin2epMap);
    void checkConnectivity_nodeMap(frNet* net, 
                                   std::vector<frConnFig*> &netDRObjs,
                                   std::vector<frBlockObject*> &netPins,
                                   std::map<frBlockObject*, std::set<std::pair<frPoint, frLayerNum> >, frBlockObjectComp> &pin2epMap,
                                   std::map<std::pair<frPoint, frLayerNum>, std::set<int> > &nodeMap);
    void checkConnectivity_nodeMap_routeObjEnd(frNet* net, std::vector<frConnFig*> &netRouteObjs,
                                               std::map<std::pair<frPoint, frLayerNum>, std::set<int> > &nodeMap);
    void checkConnectivity_nodeMap_routeObjSplit(frNet* net, std::vector<frConnFig*> &netRouteObjs,
                                                 std::map<std::pair<frPoint, frLayerNum>, std::set<int> > &nodeMap);
    void checkConnectivity_nodeMap_routeObjSplit_helper(const frPoint &crossPt, 
                   frCoord trackCoord, frCoord splitCoord, frLayerNum lNum, 
                   std::vector<std::map<frCoord, std::map<frCoord, std::pair<frCoord, int> > > > &mergeHelper,
                   std::map<std::pair<frPoint, frLayerNum>, std::set<int> > &nodeMap);
    void checkConnectivity_nodeMap_pin(frNet* net, 
                                       std::vector<frConnFig*> &netRouteObjs,
                                       std::vector<frBlockObject*> &netPins,
                                       std::map<frBlockObject*, std::set<std::pair<frPoint, frLayerNum> >, frBlockObjectComp> &pin2epMap,
                                       std::map<std::pair<frPoint, frLayerNum>, std::set<int> > &nodeMap);
    bool checkConnectivity_astar(frNet* net, std::vector<bool> &adjVisited, std::vector<int> &adjPrevIdx, 
                                 std::map<std::pair<frPoint, frLayerNum>, std::set<int> > &nodeMap, int &gCnt, int &nCnt);
    void checkConnectivity_final(frNet *net, std::vector<frConnFig*> &netRouteObjs, std::vector<frBlockObject*> &netPins,
                                 std::vector<bool> &adjVisited, int gCnt, int nCnt,
                                 std::map<std::pair<frPoint, frLayerNum>, std::set<int> > &nodeMap);
    void initDR(int size, bool enableDRC = false);
    std::map<frNet*, std::set<std::pair<frPoint, frLayerNum> >, frBlockObjectComp> initDR_mergeBoundaryPin(int i, int j, int size, const frBox &routeBox);
    void searchRepair(int iter, int size, int offset, int mazeEndIter = 1, frUInt4 workerDRCCost = DRCCOST, frUInt4 workerMarkerCost = MARKERCOST, 
                      bool enableDRC = false, int ripupMode = 1, bool TEST = false);
    void end();
    void reportDRC();
  };

  class FlexDRWorker;
  class FlexDRWorkerRegionQuery {
  public:
      FlexDRWorkerRegionQuery(FlexDRWorker* in): drWorker(in) {}
      frDesign* getDesign() const;
      FlexDRWorker* getDRWorker() const {
        return drWorker;
      }
      void add(drConnFig* connFig);
      void add(drConnFig* connFig, std::vector<std::vector<rq_rptr_value_t<drConnFig> > > &allShapes);
      void remove(drConnFig* connFig);
      void query(const frBox &box, frLayerNum layerNum, std::vector<drConnFig*> &result);
      void init();
  protected:
      FlexDRWorker* drWorker;
      std::vector<bgi::rtree<rq_rptr_value_t<drConnFig>, bgi::quadratic<16> > > shapes; // only for drXXX in dr worker
  };

  class FlexDRMinAreaVio {
  public:
    // constructors
    FlexDRMinAreaVio() {}
    FlexDRMinAreaVio(drNet* netIn, FlexMazeIdx bpIn, FlexMazeIdx epIn, frCoord gapAreaIn): net(netIn), 
                                                                                           bp(bpIn),
                                                                                           ep(epIn),
                                                                                           gapArea(gapAreaIn) {}
    // setters
    void setDRNet(drNet *netIn) {
      net = netIn;
    }
    void setPoints(FlexMazeIdx bpIn, FlexMazeIdx epIn) {
      bp = bpIn;
      ep = epIn;
    }
    void setGapArea(frCoord gapAreaIn) {
      gapArea = gapAreaIn;
    }

    // getters
    drNet* getNet() const {
      return net;
    }
    void getPoints(FlexMazeIdx &bpIn, FlexMazeIdx &epIn) const {
      bpIn = bp;
      epIn = ep;
    }
    frCoord getGapArea() const {
      return gapArea;
    }

  protected:
    drNet *net;
    FlexMazeIdx bp, ep;
    frCoord gapArea;
  };

  class FlexDRWorker {
  public:
    // constructors
    FlexDRWorker(frDesign* designIn): design(designIn), routeBox(), extBox(), drIter(0), mazeEndIter(1), 
                                      TEST(false), DRCTEST(false), enableDRC(true), ripupMode(1), workerDRCCost(DRCCOST), 
                                      workerMarkerCost(MARKERCOST), boundaryPin(), pinCnt(0), initNumMarkers(0),
                                      apSVia(), fixedObjs(), planarHistoryMarkers(), viaHistoryMarkers(), 
                                      nets(), gridGraph(designIn), markers(), rq(this) {}
    // setters
    void setRouteBox(const frBox &boxIn) {
      routeBox.set(boxIn);
    }
    void setExtBox(const frBox &boxIn) {
      extBox.set(boxIn);
    }
    void setDRIter(int in) {
      drIter = in;
    }
    void setDRIter(int in, std::map<frNet*, std::set<std::pair<frPoint, frLayerNum> >, frBlockObjectComp> &bp) {
      drIter = in;
      boundaryPin = std::move(bp);
    }
    void setMazeEndIter(int in) {
      mazeEndIter = in;
    }
    void setTest(bool in) {
      TEST = in;
    }
    void setDRCTest(bool in) {
      DRCTEST = in;
    }
    void setEnableDRC(bool in) {
      enableDRC = in;
    }
    void setRipupMode(int in) {
      ripupMode = in;
    }
    void setCost(frUInt4 drcCostIn, frUInt4 markerCostIn) {
      workerDRCCost = drcCostIn;
      workerMarkerCost = markerCostIn;
    }
    //void addMarker(std::unique_ptr<frMarker> &in) {
    //  auto rptr = in.get();
    //  markers.push_back(std::move(in));
    //  rptr->setIter(--(markers.end()));
    //}
    //void removeMarker(frMarker* in) {
    //  markers.erase(in->getIter());
    //}
    void setMarkers(std::vector<frMarker> &in) {
      markers.clear();
      frBox box;
      for (auto &marker: in) {
        marker.getBBox(box);
        if (getRouteBox().overlaps(box)) {
          markers.push_back(marker);
        }
      }
    }
    void setBestMarkers() {
      bestMarkers = markers;
    }
    void clearMarkers() {
      markers.clear();
    }
    void setInitNumMarkers(int in) {
      initNumMarkers = in;
    }

    // getters
    frTechObject* getTech() const {
      return design->getTech();
    }
    frDesign* getDesign() const {
      return design;
    }
    void getRouteBox(frBox &boxIn) const {
      boxIn.set(routeBox);
    }
    const frBox& getRouteBox() const {
      return routeBox;
    }
    frBox& getRouteBox() {
      return routeBox;
    }
    void getExtBox(frBox &boxIn) const {
      boxIn.set(extBox);
    }
    const frBox& getExtBox() const {
      return extBox;
    }
    frBox& getExtBox() {
      return extBox;
    }
    frRegionQuery* getRegionQuery() const {
      return design->getRegionQuery();
    }
    bool isInitDR() const {
      return (drIter == 0);
    }
    int getDRIter() const {
      return drIter;
    }
    bool isEnableDRC() const {
      return enableDRC;
    }
    int getRipupMode() const {
      return ripupMode;
    }
    //const std::vector<std::unique_ptr<frMarker> >& getMarkers() const {
    //  return markers;
    //}
    //std::vector<std::unique_ptr<frMarker> >& getMarkers() {
    //  return markers;
    //}
    const std::vector<std::unique_ptr<drNet> >& getNets() const {
      return nets;
    }
    std::vector<std::unique_ptr<drNet> >& getNets() {
      return nets;
    }
    const std::vector<frMarker>& getMarkers() const {
      return markers;
    }
    std::vector<frMarker>& getMarkers() {
      return markers;
    }
    const std::vector<frMarker>& getBestMarkers() const {
      return bestMarkers;
    }
    std::vector<frMarker>& getBestMarkers() {
      return bestMarkers;
    }
    const FlexDRWorkerRegionQuery& getWorkerRegionQuery() const {
      return rq;
    }
    FlexDRWorkerRegionQuery& getWorkerRegionQuery() {
      return rq;
    }
    int getInitNumMarkers() const {
      return initNumMarkers;
    }
    int getNumMarkers() const {
      return markers.size();
    }

    // others
    int main();
    // others
    int getNumQuickMarkers();
    
  protected:
    frDesign* design;
    frBox     routeBox;
    frBox     extBox;
    int       drIter;
    int       mazeEndIter;
    bool      TEST, DRCTEST;
    bool      enableDRC;
    int       ripupMode;
    frUInt4   workerDRCCost, workerMarkerCost;
    std::map<frNet*, std::set<std::pair<frPoint, frLayerNum> >, frBlockObjectComp> boundaryPin;
    int       pinCnt;
    int       initNumMarkers;
    std::map<FlexMazeIdx, drAccessPattern*> apSVia;
    std::vector<frBlockObject*>             fixedObjs;
    std::set<FlexMazeIdx>                   planarHistoryMarkers;
    std::set<FlexMazeIdx>                   viaHistoryMarkers;
    std::vector<FlexDRMinAreaVio>           minAreaVios;

    // local storage
    std::vector<std::unique_ptr<drNet> >    nets;
    FlexGridGraph                           gridGraph;
    //std::vector<std::unique_ptr<frMarker> > markers;
    std::vector<frMarker>                   markers;
    std::vector<frMarker>                   bestMarkers;
    FlexDRWorkerRegionQuery                 rq;

    // init
    void init();
    void initNets();
    void initNetObjs(std::set<frNet*, frBlockObjectComp> &nets, 
                     std::map<frNet*, std::vector<std::unique_ptr<drConnFig> >, frBlockObjectComp> &netRouteObjs,
                     std::map<frNet*, std::vector<std::unique_ptr<drConnFig> >, frBlockObjectComp> &netExtObjs);
    void initNetObjs_pathSeg(frPathSeg* pathSeg,
                             std::set<frNet*, frBlockObjectComp> &nets, 
                             std::map<frNet*, std::vector<std::unique_ptr<drConnFig> >, frBlockObjectComp> &netRouteObjs,
                             std::map<frNet*, std::vector<std::unique_ptr<drConnFig> >, frBlockObjectComp> &netExtObjs);
    void initNetObjs_via(frVia* via,
                         std::set<frNet*, frBlockObjectComp> &nets, 
                         std::map<frNet*, std::vector<std::unique_ptr<drConnFig> >, frBlockObjectComp> &netRouteObjs,
                         std::map<frNet*, std::vector<std::unique_ptr<drConnFig> >, frBlockObjectComp> &netExtObjs);
    void initNetObjs_patchWire(frPatchWire* pwire,
                               std::set<frNet*, frBlockObjectComp> &nets, 
                               std::map<frNet*, std::vector<std::unique_ptr<drConnFig> >, frBlockObjectComp> &netRouteObjs,
                               std::map<frNet*, std::vector<std::unique_ptr<drConnFig> >, frBlockObjectComp> &netExtObjs);
    void initNets_initDR(std::set<frNet*, frBlockObjectComp> &nets, 
                         std::map<frNet*, std::vector<std::unique_ptr<drConnFig> >, frBlockObjectComp> &netRouteObjs,
                         std::map<frNet*, std::vector<std::unique_ptr<drConnFig> >, frBlockObjectComp> &netExtObjs);
    void initNets_searchRepair(std::set<frNet*, frBlockObjectComp> &nets, 
                         std::map<frNet*, std::vector<std::unique_ptr<drConnFig> >, frBlockObjectComp> &netRouteObjs,
                         std::map<frNet*, std::vector<std::unique_ptr<drConnFig> >, frBlockObjectComp> &netExtObjs);
    void initNets_searchRepair_pin2epMap(frNet* net, 
                                         std::vector<std::unique_ptr<drConnFig> > &netRouteObjs,
                                         /*std::vector<std::unique_ptr<drConnFig> > &netExtObjs,
                                         std::vector<frBlockObject*> &netPins,*/
                                         std::map<frBlockObject*, std::set<std::pair<frPoint, frLayerNum> >, frBlockObjectComp> &pin2epMap/*,
                                         std::map<std::pair<frPoint, frLayerNum>, std::set<int> > &nodeMap*/);

    void initNets_searchRepair_pin2epMap_helper(frNet* net, const frPoint &bp, frLayerNum lNum, 
                                                std::map<frBlockObject*, std::set<std::pair<frPoint, frLayerNum> >, frBlockObjectComp> &pin2epMap);
    void initNets_searchRepair_nodeMap(frNet* net, 
                                       std::vector<std::unique_ptr<drConnFig> > &netRouteObjs,
                                       std::vector<frBlockObject*> &netPins,
                                       std::map<frBlockObject*, std::set<std::pair<frPoint, frLayerNum> >, frBlockObjectComp> &pin2epMap,
                                       std::map<std::pair<frPoint, frLayerNum>, std::set<int> > &nodeMap);

    void initNets_searchRepair_nodeMap_routeObjEnd(frNet* net, std::vector<std::unique_ptr<drConnFig> > &netRouteObjs,
                                                   std::map<std::pair<frPoint, frLayerNum>, std::set<int> > &nodeMap);
    void initNets_searchRepair_nodeMap_routeObjSplit(frNet* net, std::vector<std::unique_ptr<drConnFig> > &netRouteObjs,
                                                     std::map<std::pair<frPoint, frLayerNum>, std::set<int> > &nodeMap);
    void initNets_searchRepair_nodeMap_routeObjSplit_helper(const frPoint &crossPt, 
                   frCoord trackCoord, frCoord splitCoord, frLayerNum lNum, 
                   std::vector<std::map<frCoord, std::map<frCoord, std::pair<frCoord, int> > > > &mergeHelper,
                   std::map<std::pair<frPoint, frLayerNum>, std::set<int> > &nodeMap);
    void initNets_searchRepair_nodeMap_pin(frNet* net, 
                                           std::vector<std::unique_ptr<drConnFig> > &netRouteObjs,
                                           std::vector<frBlockObject*> &netPins,
                                           std::map<frBlockObject*, std::set<std::pair<frPoint, frLayerNum> >, frBlockObjectComp> &pin2epMap,
                                           std::map<std::pair<frPoint, frLayerNum>, std::set<int> > &nodeMap);
    void initNets_searchRepair_connComp(frNet* net, 
                                        std::map<std::pair<frPoint, frLayerNum>, std::set<int> > &nodeMap,
                                        std::vector<int> &compIdx);



    void initNet(frNet* net, 
                 std::vector<std::unique_ptr<drConnFig> > &routeObjs,
                 std::vector<std::unique_ptr<drConnFig> > &extObjs,
                 std::vector<frBlockObject*> &terms);
    void initNet_term(drNet* dNet, std::vector<frBlockObject*> &terms);
    void initNet_termGenAp(drPin* dPin);
    void getTrackLocs(bool isHorzTracks, frLayerNum currLayerNum, frCoord low, frCoord high, std::set<frCoord> &trackLocs);
    void initNet_boundary(drNet* net, std::vector<std::unique_ptr<drConnFig> > &extObjs);
    void initNets_regionQuery();
    void initNets_numPinsIn();

    void initGridGraph();
    void initTrackCoords(std::map<frCoord, std::map<frLayerNum, frTrackPattern*> > &xMap,
                         std::map<frCoord, std::map<frLayerNum, frTrackPattern*> > &yMap);
    void initTrackCoords_route(drNet* net, 
                               std::map<frCoord, std::map<frLayerNum, frTrackPattern*> > &xMap,
                               std::map<frCoord, std::map<frLayerNum, frTrackPattern*> > &yMap);
    void initTrackCoords_pin(drNet* net, 
                             std::map<frCoord, std::map<frLayerNum, frTrackPattern*> > &xMap,
                             std::map<frCoord, std::map<frLayerNum, frTrackPattern*> > &yMap);
    void initMazeIdx();
    void initMazeIdx_connFig(drConnFig *connFig);
    void initMazeIdx_ap(drAccessPattern *ap);
    void initMazeCost();
    void initMazeCost_connFig();
    void initMazeCost_pin();
    void initMazeCost_pin_helper(const frBox &box, frCoord bloatDist, frMIdx zIdx);
    void initMazeCost_ap(); // disable maze edge
    void initMazeCost_marker();
    //void initMazeCost_via();
    void initMazeCost_via_helper(drNet* net, bool isAddPathCost);
    void initMazeCost_ap_helper(drNet* net, bool isAddPathCost);
    void initMazeCost_boundary_helper(drNet* net, bool isAddPathCost);
    void addToCostGrids(const Rectangle &region, std::set<std::pair<frMIdx, frMIdx> > &costGrids);

    // DRC
    void initFixedObjs();
    void initMarkers();

    // route
    void route();
    void addPathCost(drConnFig *connFig);
    void subPathCost(drConnFig *connFig);
    /*inline*/ void modPathCost(drConnFig *connFig, bool isAddPathCost);
    // minSpc
    /*inline*/ void modMinSpacingCost(drNet* net, const frBox &box, frMIdx z, bool isAddPathCost, bool isCurrPs);
    /*inline*/ void modMinSpacingCostPlaner(drNet* net, const frBox &box, frMIdx z, bool isAddPathCost);
    /*inline*/ void modMinSpacingCostVia(const frBox &box, frMIdx z, bool isAddPathCost, bool isUpperVia, bool isCurrPs);
    inline frCoord pt2boxDistSquare(const frPoint &pt, const frBox &box);
    inline frCoord box2boxDistSquare(const frBox &box1, const frBox &box2, frCoord &dx, frCoord &dy);
    /*inline*/ void modMinSpacingCostVia_eol(const frBox &box, const frBox &tmpBx, bool isAddPathCost, bool isUpperVia, frMIdx i, frMIdx j, frMIdx z);
    /*inline*/ void modMinSpacingCostVia_eol_helper(const frBox &box, const frBox &testBox, bool isAddPathCost, bool isUpperVia, frMIdx i, frMIdx j, frMIdx z);
    // eolSpc
    /*inline*/ void modEolSpacingCost_helper(const frBox &testbox, frMIdx z, bool isAddPathCost, int type);
    /*inline*/ void modEolSpacingCost(const frBox &box, frMIdx z, bool isAddPathCost);
    // cutSpc
    /*inline*/ void modCutSpacingCost(const frBox &box, frMIdx z, bool isAddPathCost);

    void mazeIterInit(std::vector<drNet*> &rerouteNets);
    void mazeIterInit_initDR(std::vector<drNet*> &rerouteNets);
    void mazeIterInit_sortRerouteNets(std::vector<drNet*> &rerouteNets);
    void mazeIterInit_searchRepair(std::vector<drNet*> &rerouteNets);

    void mazeNetInit(drNet* net);
    void mazeNetEnd(drNet* net);
    bool routeNet(drNet* net);
    void routeNet_prep(drNet* net, std::set<drPin*, frBlockObjectComp> &pins, 
                       std::map<FlexMazeIdx, std::set<drPin*, frBlockObjectComp> > &mazeIdx2unConnPins);
    void routeNet_setSrc(std::set<drPin*, frBlockObjectComp> &unConnPins, 
                         std::map<FlexMazeIdx, std::set<drPin*, frBlockObjectComp> > &mazeIdx2unConnPins,
                         std::vector<FlexMazeIdx> &connComps, FlexMazeIdx &ccMazeIdx1, FlexMazeIdx &ccMazeIdx2);
    void mazePinInit();
    drPin* routeNet_getNextDst(FlexMazeIdx &ccMazeIdx1, FlexMazeIdx &ccMazeIdx2, 
                               std::map<FlexMazeIdx, std::set<drPin*, frBlockObjectComp> > &mazeIdx2unConnPins);
    void        routeNet_postAstarUpdate(std::vector<FlexMazeIdx> &path, std::vector<FlexMazeIdx> &connComps,
                                         std::set<drPin*, frBlockObjectComp> &unConnPins, 
                                         std::map<FlexMazeIdx, std::set<drPin*, frBlockObjectComp> > &mazeIdx2unConnPins,
                                         bool isFirstConn);
    void        routeNet_postAstarWritePath(drNet* net, std::vector<FlexMazeIdx> &points);
    void        routeNet_postAstarPatchMinAreaVio(drNet* net, const std::vector<FlexMazeIdx> &path);
    void        routeNet_postAstarAddPatchMetal(drNet* net, const FlexMazeIdx bpIdx, const FlexMazeIdx epIdx, const frCoord gapArea, const frCoord patchWidth);
    void        routeNet_postRouteAddPathCost(drNet* net);
    void        routeNet_postRouteAddPatchMetalCost(drNet* net);

    // drc
    void route_drc();
    void route_postRouteViaSwap();

    // end
    void end();
    void endGetModNets(std::set<frNet*, frBlockObjectComp> &modNets);
    void endRemoveNets(std::set<frNet*, frBlockObjectComp> &modNets, 
                       std::map<frNet*, std::set<std::pair<frPoint, frLayerNum> >, frBlockObjectComp> &boundPts);
    void endRemoveNets_pathSeg(frPathSeg* pathSeg, std::set<std::pair<frPoint, frLayerNum> > &boundPts);
    //void endRemoveNets(std::set<frNet*> &modNets);
    //void endRemoveNets_pathSeg(frPathSeg* pathSeg);
    void endRemoveNets_via(frVia* via);
    void endRemoveNets_patchWire(frPatchWire* pwire);
    void endAddNets(std::map<frNet*, std::set<std::pair<frPoint, frLayerNum> >, frBlockObjectComp> &boundPts);
    void endAddNets_pathSeg(drPathSeg* pathSeg);
    //void endAddNets();
    //void endAddNets_pathSeg(drPathSeg* pathSeg);
    void endAddNets_via(drVia* via);
    void endAddNets_patchWire(drPatchWire* pwire);
    void endAddNets_merge(frNet* net, std::set<std::pair<frPoint, frLayerNum> > &boundPts);

    void endRemoveMarkers();
    void endAddMarkers();

  };

}

#endif
