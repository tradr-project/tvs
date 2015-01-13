/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */

#include "OMPLTVSEnvironment.h"
#include "OMPLTVSStateSpace.h"
#include <boost/lexical_cast.hpp>

ompl::control::OMPLTVSEnvironment::OMPLTVSEnvironment(Environment *env)
: stepSize_(0.5), maxControlSteps_(5), minControlSteps_(1), env_(env) {
}

ompl::control::OMPLTVSEnvironment::~OMPLTVSEnvironment() {
}

unsigned int ompl::control::OMPLTVSEnvironment::getControlDimension(void) const {
    return 2;
}

void ompl::control::OMPLTVSEnvironment::getControlBounds(std::vector<double> &lower, std::vector<double> &upper) const {
    static double maxVel = 5.0;
    lower.resize(2);
    lower[0] = -maxVel;
    lower[1] = -maxVel;
    
    upper.resize(2);
    upper[0] = maxVel;
    upper[1] = maxVel;
}

void ompl::control::OMPLTVSEnvironment::applyControl(const double *control) const {
    env_->v->setTrackVelocities(control[0], control[1]);
}

void ompl::control::OMPLTVSEnvironment::addToSearchTree(const ompl::base::State *s1, const ompl::base::State *s2)
{
    searchTree.resize(searchTree.size() + 1);
    memcpy(searchTree.back().a, s1->as<ompl::control::OMPLTVSStateSpace::StateType>()->getBodyPosition(0), sizeof(dReal) * 3);
    memcpy(searchTree.back().b, s2->as<ompl::control::OMPLTVSStateSpace::StateType>()->getBodyPosition(0), sizeof(dReal) * 3);
}
