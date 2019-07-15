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

/**************************************************************************
 * Copyright(c) 2018 Regents of the University of California
 *              Andrew B. Kahng, Lutong Wang and Bangqi Xu
 * Contact      abk@cs.ucsd.edu, luw002@ucsd.edu and bax002@ucsd.edu
 * Affiliation: Computer Science and Engineering Department, UC San Diego,
 *              La Jolla, CA 92093-0404, USA
 *
 *************************************************************************/

#ifndef _GLOBAL_H_
#define _GLOBAL_H_

#include <iostream>
#include <memory>
#include <string>
#include "frDesign.h"
#include "db/obj/frBlock.h"

extern std::string DEF_FILE;
extern std::string GUIDE_FILE;
extern std::string OUTGUIDE_FILE;
extern std::string LEF_FILE;
extern std::string OUTTA_FILE;
extern std::string OUT_FILE;
extern std::string DBPROCESSNODE;
extern std::string OUT_MAZE_FILE;
extern std::string DRC_RPT_FILE;
extern int MAX_THREADS ;
extern int VERBOSE     ;
extern int BOTTOM_ROUTING_LAYER;
extern bool ALLOW_PIN_AS_FEEDTHROUGH;
extern bool USENONPREFTRACKS;
//extern int TEST;

extern int END_ITERATION;

extern fr::frUInt4 TAVIACOST;
extern fr::frUInt4 TAPINCOST;
extern fr::frUInt4 TAALIGNCOST;
extern fr::frUInt4 TADRCCOST;
extern float       TASHAPEBLOATWIDTH;

extern fr::frUInt4 VIACOST;
extern bool USEMINSPACING_OBS;
extern bool RESERVE_VIA_ACCESS;

extern fr::frUInt4 GRIDCOST;
extern fr::frUInt4 SHAPECOST;
extern fr::frUInt4 DRCCOST;
extern fr::frUInt4 MARKERCOST;
extern float       MARKERDECAY;
extern float       SHAPEBLOATWIDTH;

#define DIRBITSIZE 3
#define WAVEFRONTBUFFERSIZE 2
#define WAVEFRONTBITSIZE (WAVEFRONTBUFFERSIZE * DIRBITSIZE)
#define WAVEFRONTBUFFERHIGHMASK (111 << ((WAVEFRONTBUFFERSIZE - 1) * DIRBITSIZE))

namespace fr {
  extern frCoord getGCELLGRIDX();
  extern frCoord getGCELLGRIDY();
  extern frCoord getGCELLOFFSETX();
  extern frCoord getGCELLOFFSETY();
}

extern std::ostream& operator<< (std::ostream& os, const fr::frViaDef &viaDefIn);
extern std::ostream& operator<< (std::ostream& os, const fr::frBlock &blockIn);
extern std::ostream& operator<< (std::ostream& os, const fr::frInst &instIn);
extern std::ostream& operator<< (std::ostream& os, const fr::frInstTerm &instTermIn);
extern std::ostream& operator<< (std::ostream& os, const fr::frTerm &termIn);
extern std::ostream& operator<< (std::ostream& os, const fr::frPin &pinIn);
extern std::ostream& operator<< (std::ostream& os, const fr::frRect &pinFig);
extern std::ostream& operator<< (std::ostream& os, const fr::frPolygon &pinFig);
extern std::ostream& operator<< (std::ostream& os, const fr::frNet &net);
extern std::ostream& operator<< (std::ostream& os, const fr::frLayerBlockage &blk);

extern std::ostream& operator<< (std::ostream& os, const fr::frPoint &pIn);
extern std::ostream& operator<< (std::ostream& os, const fr::frBox &box);

//extern size_t getPeakRSS();
//extern size_t getCurrentRSS();

//extern void printAllMacros(const std::shared_ptr<fr::frDesign> &design);

#endif
