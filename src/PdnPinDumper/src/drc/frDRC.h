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

#ifndef _FR_DRC_H_
#define _FR_DRC_H_

#include "frBaseTypes.h"
#include "frDesign.h"
#include "db/tech/frConstraint.h"
#include "db/obj/frMarker.h"
#include <boost/icl/interval_set.hpp>
#include "db/drObj/drNet.h"

namespace fr {
  enum class drcEdgeTypeEnum {
    UNKNOWN = 0,
    FIXED = 1,
    ROUTE = 2
  };

  enum class drcSpecialNetIdEnum {
    OBS = 0,
    FLOATPOWER = 1,
    FLOATGROUND = 2,
    FLOATSIGNAL = 3
  };

  class DRCNet;
  class DRCEdge;
  // class drConnFig;
  // class drNet;
  // class drVia;
  // class drPathSeg;

  class DRCWorker {
  public:
    // constructor
    DRCWorker() {}
    DRCWorker(frDesign* designIn, frVector<frBlockObject*> objsIn): design(designIn), objs(objsIn), target(nullptr) {}
    // getters
    frVector<frMarker>& getViolations() {
      return markers;
    }
    // setters

    // functions
    void init();
    void addDRNets(const std::vector<std::unique_ptr<drNet> > &nets);
    void setTarget(frBlockObject* in) {
      target = in;
    }
    void setup();
    void main();
    void report();

  protected:
    frDesign* design;
    // constraints
    std::map<frLayerNum, frVector<frConstraint*> > layer2Constraints;
    // objects
    frVector<frBlockObject*> objs; // all objects that the DRCWorker sees intiially. Sort later
    frBlockObject* target;
    frVector<drConnFig*> drConnFigs;
    frVector<frBlockObject*> terms; // including terms and instTerms
    frVector<frConnFig*> connFigs; // including wire and via
    frVector<frBlockObject*> blockages; 
    // internal information
    int netCnt = 4;
    int targetNetId = -1;
    std::map<frNet*, int> net2Id; // OBS's netId == 0, floating power net Id == 1, 
                                  // floating ground net Id == 2, floating signal net == 3
    std::map<frTerm*, int> term2Id; 
    frVector<std::unique_ptr<DRCNet> > drcNets;
    std::map<frLayerNum, bgi::rtree<std::pair<segment_t, DRCEdge*>, bgi::quadratic<16> > > layer2EdgeRTree; // segment to Edge
    std::map<frLayerNum, bgi::rtree<std::pair<box_t, int>, bgi::quadratic<16> > > layer2FixedRTree, layer2MergedRTree; // max rectangle to netId
    
    // output
    frVector<frMarker> markers;

    // functions
    void populateLayerConstraints();
    void initNetRects();
    void initSpecialNets();
    void initNetRects_terms();
    void initNetRects_connFigs();
    void initNetRects_blockages();
    void initNetRects_drConnFigs();
    void sortObj();
    void setupEdgeRTree();
    void setupEdgeRTree_net(DRCNet* net);
    void setupMaxRectRTree();
    void setupMaxRectRTree_net(DRCNet* net);
    void updateMaxRectRtree(int netId, 
                            const std::map<frLayerNum, PolygonSet> &layer2PolySet, 
                            std::map<frLayerNum, bgi::rtree<std::pair<box_t, int>, bgi::quadratic<16> > > &maxRectRTree);
    

    // rule check related
    void checkMetalShort();
    void checkMetalSpacingRules();
    void checkMetalSpacingTablePrlRule(const bgi::rtree<std::pair<segment_t, DRCEdge*>, bgi::quadratic<16> > &edgeRTree,  
                                       frSpacingTablePrlConstraint* constraint);
    void checkMetalSpacingEndOfLineRule(const bgi::rtree<std::pair<segment_t, DRCEdge*>, bgi::quadratic<16> > &edgeRTree,  
                                        frSpacingEndOfLineConstraint* constraint);
    void checkCutSpacingRules();
    void checkCutSpacingRule(const bgi::rtree<std::pair<segment_t, DRCEdge*>, bgi::quadratic<16> > &edgeRTree,
                             frCutSpacingConstraint* constraint);
    void checkMinWidthRule();
    void checkMinWidthRule_net(DRCNet* drcNet);
    void checkMetalMinAreaRule();
    void checkMetalMinAreaRule_net(DRCNet* drcNet);
    void checkMetalMinStepRule();
    void checkMetalMinStepRule_net(DRCNet* drcNet);
    

    // utility
    void addDRConnFig(drConnFig* in);
    void getSpacingTableTestBox(const segment_t &seg, DRCEdge* prlEdge, frCoord maxSpacingVal, box_t &testBox);
    bool isOppositeDir(frDirEnum dir1, frDirEnum dir2);
    void getEdgeSpanBox(const segment_t &seg, const frDirEnum &dir, const frCoord &span, box_t &edgeSpanBox);
    void getEdgeSpanRect(const segment_t &seg, const frDirEnum &dir, const frCoord &span, Rectangle &edgeSpanRect);
    void getEOLTestBox(const segment_t &seg, DRCEdge* eolEdge, const frCoord &eolSpace, const frCoord &eolWithin, box_t &testBox);
    void getParallelEdgeBoxs(const segment_t &seg,
                              DRCEdge *eolEdge, 
                              const frCoord &parSpace,
                              const frCoord &parWithin,
                              const frCoord &eolWithin,
                              box_t &parallelEdgeBoxLeft,
                              box_t &parallelEdgeBoxRight);
    bool isEolEdge(DRCEdge* edge);
    frDirEnum leftEdgeDir(const frDirEnum &dirIn);
    frDirEnum rightEdgeDir(const frDirEnum &dirIn);
    void printEdgeRTree();
    bool covered_by(const boost::icl::discrete_interval<frCoord, std::less> &intv,
                    const boost::icl::interval_set<frCoord> &intvSet);
    frCoord rectDiagLength(const Rectangle &rect);

  };
  
  class DRCNet {
  public:
    // constructor
    DRCNet() {}
    DRCNet(int netIdIn): id(netIdIn), src(nullptr) {}
    // getters
    int getId() {
      return id;
    }
    // frNet* getNet() {
    //   return dbNet;
    // }
    frBlockObject* getSrc() {
      return src;
    }
    frVector<std::pair<frLayerNum, Rectangle> >& getFixedRects() {
      return fixedRects;
    }
    frVector<std::pair<frLayerNum, Rectangle> >& getRouteRects() {
      return routeRects;
    }
    std::map<frLayerNum, PolygonSet>& getLayer2FixedPolySet() {
      return layer2FixedPolySet;
    }
    std::map<frLayerNum, PolygonSet>& getLayer2RoutePolySet() {
      return layer2RoutePolySet;
    }
    std::map<frLayerNum, std::map<Segment, DRCEdge*> >& getLayerSegEdgeMap() {
      return layer2Seg2Edge;
    }

    // setters
    void setId(int idIn) {
      id = idIn;
    }
    // void setNet(frNet* netIn) {
    //   dbNet = netIn;
    // }
    void setSrc(frBlockObject *srcIn) {
      src = srcIn;
    }
    void addFixedRect(const std::pair<frLayerNum, Rectangle> &layerRectIn) {
      fixedRects.push_back(layerRectIn);
    }
    void addRouteRect(const std::pair<frLayerNum, Rectangle> &layerRectIn) {
      routeRects.push_back(layerRectIn);
    }
    void addFixedLayerPoly(const frLayerNum &layerNumIn, const Polygon &polyIn) {
      using namespace boost::polygon::operators;
      layer2FixedPolySet[layerNumIn] += polyIn;
    }
    void addFixedLayerRect(const frLayerNum &layerNumIn, const Rectangle &rectIn) {
      using namespace boost::polygon::operators;
      layer2FixedPolySet[layerNumIn] += rectIn;
    }
    void addRouteLayerPoly(const frLayerNum &layerNumIn, const Polygon &polyIn) {
      using namespace boost::polygon::operators;
      layer2RoutePolySet[layerNumIn] += polyIn;
    }
    void addRouteLayerRect(const frLayerNum &layerNumIn, const Rectangle &rectIn) {
      using namespace boost::polygon::operators;
      layer2RoutePolySet[layerNumIn] += rectIn;
    }
    DRCEdge* addDRCEdge() {
      drcEdges.push_back(std::make_unique<DRCEdge>());
      return (drcEdges.back().get());
    }
    void addToLayerSegEdgeMap(const frLayerNum &layerNumIn, const Segment &segIn, DRCEdge* edgeIn) {
      layer2Seg2Edge[layerNumIn][segIn] = edgeIn;
    }

  protected:
    int id;
    // frNet* dbNet;
    // frInstTerm *dbInstTerm;
    // frTerm *dbTerm;
    frBlockObject *src;
    frVector<std::pair<frLayerNum, Rectangle> > fixedRects, routeRects;
    std::map<frLayerNum, PolygonSet> layer2FixedPolySet, layer2RoutePolySet;
    std::vector<std::unique_ptr<DRCEdge> > drcEdges;
    std::map<frLayerNum, std::map<Segment, DRCEdge*> > layer2Seg2Edge;
  };

  class DRCEdge {
  public:
    // constructor
    DRCEdge() : netId(-1), type(drcEdgeTypeEnum::UNKNOWN), dir(frDirEnum::UNKNOWN), prev(nullptr), next(nullptr), span(-1), width(-1) {}
    // getters
    int getNetId() {
      return netId;
    }
    frLayerNum getLayerNum() {
      return layerNum;
    }
    Segment getSegment() {
      return seg;
    }
    drcEdgeTypeEnum getType() {
      return type;
    }
    frDirEnum getDir() {
      return dir;
    }
    DRCEdge* getPrevEdge() {
      return prev;
    }
    DRCEdge* getNextEdge() {
      return next;
    }
    frCoord getSpan() {
      return span;
    }
    frCoord getWidth() {
      return width;
    }
    boost::polygon::direction_1d getWinding() {
      return winding;
    }

    // setters
    void setId(const int &netIdIn) {
      netId = netIdIn;
    }
    void setLayerNum(const frLayerNum &layerNumIn) {
      layerNum = layerNumIn;
    }
    void setSegment(const Segment &segIn) {
      seg = segIn;
    }
    void setType(const drcEdgeTypeEnum &typeIn) {
      type = typeIn;
    }
    void setDir(const frDirEnum &dirIn) {
      dir = dirIn;
    }
    void setPrev(DRCEdge* prevIn) {
      prev = prevIn;
    }
    void setNext(DRCEdge* nextIn) {
      next = nextIn;
    }
    void setSpan(const frCoord &spanIn) {
      span = spanIn;
    }
    void setWidth(const frCoord &widthIn) {
      width = widthIn;
    }
    void setWinding(const boost::polygon::direction_1d &in) {
      winding = in;
    }
    
    
  protected:
    int netId;
    Segment seg;
    frLayerNum layerNum;
    drcEdgeTypeEnum type;
    frDirEnum dir; // the direction perpendicular to the edge and towards outside of polygon
    DRCEdge *prev, *next; // clockwise linked list of edges for minstep checking, eol checking
    frCoord span, width;
    boost::polygon::direction_1d winding;
  };

}



#endif
