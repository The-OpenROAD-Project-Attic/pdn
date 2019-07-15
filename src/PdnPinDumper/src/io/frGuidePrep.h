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

#ifndef _FR_GUIDE_PREP_H_
#define _FR_GUIDE_PREP_H_

#include "frBaseTypes.h"
#include <map>
#include <utility>
#include <tuple>
#include "db/tech/frLayer.h"
#include "db/obj/frBlock.h"
//#include "frMacro.h"
#include <memory>
#include <string>
#include <set>
#include <boost/icl/interval_set.hpp>
#include <boost/icl/interval_map.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/pending/disjoint_sets.hpp>
#include <boost/icl/split_interval_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/prim_minimum_spanning_tree.hpp>


namespace fr {


  frCoord getGCELLGRIDX();
  frCoord getGCELLGRIDY();
  frCoord getGCELLOFFSETX();
  frCoord getGCELLOFFSETY();
  
  void setGCELLGRIDX(frCoord &);
  void setGCELLGRIDY(frCoord &);
  void setGCELLOFFSETX(frCoord &);
  void setGCELLOFFSETY(frCoord &);

  struct guideSteinerCmp {
    bool operator()(const std::pair<std::pair<frPoint, frLayerNum>, std::pair<frPoint, frLayerNum> > &lhs,
                    const std::pair<std::pair<frPoint, frLayerNum>, std::pair<frPoint, frLayerNum> > &rhs) {
      if (lhs.first == rhs.first) {
        return (lhs.second > rhs.second);
      } else {
        return (lhs.first > rhs.first);
      }
    }
  };

  void getGCellParam(std::map<std::string, fr::frCollection<fr::frRect> > &rawGuide, std::vector<std::shared_ptr<fr::frLayer> > &vlayers);
  void getGCellSizeParam(std::map<std::string, fr::frCollection<fr::frRect> > &rawGuide, std::vector<std::shared_ptr<fr::frLayer> > &vlayers);
  void getGCellOffsetParam(std::map<std::string, fr::frCollection<fr::frRect> > &rawGuide, std::vector<std::shared_ptr<fr::frLayer> > &vlayers);
  void printGCellInfo();

  void updateXform(frTransform &xform, frPoint &size);
  frBox point2GCellBox(frPoint &pointIn);
  void genGCell2PinMap(std::map<std::string, std::shared_ptr<fr::frBlock> > &macros,
                       std::map<std::string, std::shared_ptr<fr::frNet> > &nets,
                       std::map<std::string, std::map<frPoint, std::set<std::pair<frBlockObject*, frLayerNum> > > > &gCell2PinMap);
  // update GCell2PinMap based on whether the term is intersecting with any GCell
  void updateGCell2PinMap(frBlockObject* blockObj,
                          frTerm* term,
                          std::string netName,
                          std::map<std::string, std::map<frPoint, std::set<std::pair<frBlockObject*, frLayerNum> > > > &gCell2PinMap);
  // void getStandardGuide(frCollection<frRect> &guidesIn,
  //                       frCollection<frRect> &guidesOut,
  //                       std::vector<std::shared_ptr<fr::frLayer> > layers);
  void getStandardGuide(std::map<std::string, fr::frCollection<fr::frRect> > &tmpGuides,
                        std::vector<std::shared_ptr<fr::frLayer> > &vlayers,
                        std::map<std::string, std::shared_ptr<fr::frNet> > &nets,
                        std::map<std::string, std::map<frPoint, std::set<std::pair<frBlockObject*, fr::frLayerNum> > > > &gCell2PinMap);
  void getStandardGuide(frCollection<frRect> &guideRects,
                        std::vector<std::shared_ptr<fr::frLayer> > &vlayers,
                        std::shared_ptr<fr::frNet> &net,
                        std::map<frPoint, std::set<std::pair<frBlockObject*, fr::frLayerNum> > > &gCell2PinMap);
  // frCollection<std::set<int> > getGuideCluster(frCollection<frRect> &guideBoxes);
  frPoint getGCellCenter(frPoint pointIn);
  frCollection<frLayerNum> getPinLayerNum(std::shared_ptr<frBlockObject> &termIn);
  
  void getGSegSteiner(boostSegment &currGSeg,
                          frPrefRoutingDir &currDir,
                          std::map<frCoord, boost::icl::interval_set<frCoord> > &neighborGCell2RowMap,
                          std::set<frPoint> &steiners);
  void getInterLayerSteinerEdge(frPoint steiner, 
                                frLayerNum lowerLayerNum, 
                                frLayerNum upperLayerNum,
                                std::set<std::pair<frPoint, frLayerNum> > &steinerSet,
                                // std::set<std::pair<std::pair<frPoint, frLayerNum>, std::pair<frPoint, frLayerNum> >, guideSteinerCmp> &steinerEdgeSet);
                                std::set<std::pair<std::pair<frPoint, frLayerNum>, std::pair<frPoint, frLayerNum> > > &steinerEdgeSet);
  
  void checkGRConnectivity(std::map<std::string, std::shared_ptr<fr::frNet> > &nets);

  void appendTouchingGuideIntv(std::map<frLayerNum, std::map<frCoord, boost::icl::interval_set<frCoord> > > &layerGCell2IntvSet,
                               std::vector<std::shared_ptr<fr::frLayer> > &vlayers);
  void getLocalNetStats(const std::map<std::string, std::map<frPoint, std::set<std::pair<std::shared_ptr<frBlockObject>, frLayerNum> > > > &gCell2PinMap);
  
  void KMBTermSteinerTree(frNet &net,
                          graph_t &netG,
                          const std::shared_ptr<frCMap> &cMap,
                          std::set<std::shared_ptr<frConnFig> > &routes);
  double getKMBEdgeWeight(frBlockObject* start,
                          frBlockObject* end,
                          const std::shared_ptr<frCMap> &cMap);
  double getGCellCongestionScore(frPoint point, frLayerNum &layerNum, const std::shared_ptr<frCMap> &cMap);
}

#endif
