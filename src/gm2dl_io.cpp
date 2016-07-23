// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G.Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "gm2dl_io.h"

  const int Gm2dlIO::ID_LASERPOSE  = numeric_limits<int>::max();
  const int Gm2dlIO::ID_ODOMCALIB  = numeric_limits<int>::max() - 1;

  int Gm2dlIO::readRobotOdom(const std::string& filename, DataQueue& queue)
  {
    ifstream is(filename.c_str());
    if (! is.good())
      return false;

    int cnt = 0;

    // parse the GM2DL file and extract the vertices, edges, and the laser data
    stringstream currentLine;
    while(1) {
      int bytesRead = readLine(is, currentLine);
      if (bytesRead == -1)
        break;
      RobotOdom* rl2 = new RobotOdom;
      rl2->read(currentLine);
      queue.add(rl2);
      cnt++;
    }
    return cnt;
  }

