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

#ifndef _DR_NET_H_
#define _DR_NET_H_

#include <memory>
#include "db/drObj/drBlockObject.h"
#include "db/drObj/drPin.h"
#include "db/drObj/drShape.h"
#include "db/drObj/drVia.h"

namespace fr {
  class frNet;
  class drNet: public drBlockObject {
  public:
    // constructors
    drNet(): drBlockObject(), pins(), extConnFigs(), routeConnFigs(), bestRouteConnFigs(),
             fNet(nullptr), modified(false), numMarkers(0), numPinsIn(0), pinBox(), ripup(false) {}
    // getters
    const std::vector<std::unique_ptr<drPin> >& getPins() const {
      return pins;
    }
    std::vector<std::unique_ptr<drPin> >& getPins() {
      return pins;
    }
    const std::vector<std::unique_ptr<drConnFig> >& getExtConnFigs() const {
      return extConnFigs;
    }
    std::vector<std::unique_ptr<drConnFig> >& getExtConnFigs() {
      return extConnFigs;
    }
    const std::vector<std::unique_ptr<drConnFig> >& getRouteConnFigs() const {
      return routeConnFigs;
    }
    std::vector<std::unique_ptr<drConnFig> >& getRouteConnFigs() {
      return routeConnFigs;
    }
    const std::vector<std::unique_ptr<drConnFig> >& getBestRouteConnFigs() const {
      return bestRouteConnFigs;
    }
    std::vector<std::unique_ptr<drConnFig> >& getBestRouteConnFigs() {
      return bestRouteConnFigs;
    }
    frNet* getFrNet() const {
      return fNet;
    }
    bool isModified() const {
      return modified;
    }
    int getNumMarkers() const {
      return numMarkers;
    }
    int getNumPinsIn() const {
      return numPinsIn;
    }
    void getPinBox(frBox &in) {
      in.set(pinBox);
    }
    bool isRipup() const {
      return ripup;
    }

    // setters
    void addPin(std::unique_ptr<drPin> &pinIn) {
      pinIn->setNet(this);
      //pinIn->setId(pins.size());
      pins.push_back(std::move(pinIn));
    }
    void addRoute(std::unique_ptr<drConnFig> &in, bool isExt = false) {
      in->addToNet(this);
      if (isExt) {
        extConnFigs.push_back(std::move(in));
      } else {
        routeConnFigs.push_back(std::move(in));
      }
    }
    void setBestRouteConnFigs() {
      for (auto &uConnFig: routeConnFigs) {
        if (uConnFig->typeId() == drcPathSeg) {
          std::unique_ptr<drConnFig> uPtr = std::make_unique<drPathSeg>(*static_cast<drPathSeg*>(uConnFig.get()));
          bestRouteConnFigs.push_back(std::move(uPtr));
        } else if (uConnFig->typeId() == drcVia) {
          std::unique_ptr<drConnFig> uPtr = std::make_unique<drVia>(*static_cast<drVia*>(uConnFig.get()));
          bestRouteConnFigs.push_back(std::move(uPtr));
        } else if (uConnFig->typeId() == drcPatchWire) {
          std::unique_ptr<drConnFig> uPtr = std::make_unique<drPatchWire>(*static_cast<drPatchWire*>(uConnFig.get()));
          bestRouteConnFigs.push_back(std::move(uPtr));
        }
      }
    }
    void clear() {
      routeConnFigs.clear();
    }
    void setFrNet(frNet* in) {
      fNet = in;
    }
    void setModified(bool in) {
      modified = in;
    }

    void setNumMarkers(int in) {
      numMarkers = in;
    }
    void addMarker() {
      numMarkers++;
    }
    void setNumPinsIn(int in) {
      numPinsIn = in;
    }
    void setPinBox(const frBox &in) {
      pinBox.set(in);
    }
    void setRipup() {
      ripup = true;
    }
    void resetRipup() {
      ripup = false;
    }

    // others
    frBlockObjectEnum typeId() const override {
      return drcNet;
    }

    bool operator< (const drNet &b) const {
      return (numMarkers == b.numMarkers) ? (id < b.id) : (numMarkers > b.numMarkers);
    }

  protected:
    std::vector<std::unique_ptr<drPin> >         pins;
    std::vector<std::unique_ptr<drConnFig> >     extConnFigs;
    std::vector<std::unique_ptr<drConnFig> >     routeConnFigs;
    std::vector<std::unique_ptr<drConnFig> >     bestRouteConnFigs;
    frNet*                                       fNet;
    bool                                         modified;
    int                                          numMarkers;
    int                                          numPinsIn;
    frBox                                        pinBox;
    bool                                         ripup;
  };
}




#endif
