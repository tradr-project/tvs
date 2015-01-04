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

#include "OMPLTVSStateSpace.h"
#include <ompl/util/Console.h>
#include <boost/lexical_cast.hpp>
#include <limits>
#include <queue>

ompl::control::OMPLTVSStateSpace::OMPLTVSStateSpace(const OMPLTVSEnvironmentPtr &env,
                                                  double positionWeight, double linVelWeight, double angVelWeight, double orientationWeight) :
    base::CompoundStateSpace(), env_(env)
{
    setName("OMPLTVS" + getName());
    type_ = base::STATE_SPACE_TYPE_COUNT + 1;

    std::string body = ""; // unused

    addSubspace(base::StateSpacePtr(new base::RealVectorStateSpace(3)), positionWeight); // position
    components_.back()->setName(components_.back()->getName() + body + ":position");
    
    addSubspace(base::StateSpacePtr(new base::RealVectorStateSpace(3)), linVelWeight);   // linear velocity
    components_.back()->setName(components_.back()->getName() + body + ":linvel");
    
    addSubspace(base::StateSpacePtr(new base::RealVectorStateSpace(3)), angVelWeight);   // angular velocity
    components_.back()->setName(components_.back()->getName() + body + ":angvel");
    
    addSubspace(base::StateSpacePtr(new base::SO3StateSpace()), orientationWeight);      // orientation
    components_.back()->setName(components_.back()->getName() + body + ":orientation");

    lock(); // prevents modification of subspaces
    setDefaultBounds();
}

void ompl::control::OMPLTVSStateSpace::setDefaultBounds()
{
    base::RealVectorBounds bounds1(3);

    // find the bounding box that contains all geoms included in the collision spaces
    double mX, mY, mZ, MX, MY, MZ;
    mX = mY = mZ = std::numeric_limits<double>::infinity();
    MX = MY = MZ = -std::numeric_limits<double>::infinity();
    bool found = false;

    std::queue<dSpaceID> spaces;
    spaces.push(env_->env_->space);

    while (!spaces.empty())
    {
        dSpaceID space = spaces.front();
        spaces.pop();

        int n = dSpaceGetNumGeoms(space);

        for (int j = 0 ; j < n ; ++j)
        {
            dGeomID geom = dSpaceGetGeom(space, j);
            if (dGeomIsSpace(geom))
                spaces.push((dSpaceID)geom);
            else
            {
                bool valid = true;
                dReal aabb[6];
                dGeomGetAABB(geom, aabb);

                // things like planes are infinite; we want to ignore those
                for (int k = 0 ; k < 6 ; ++k)
                    if (fabs(aabb[k]) >= std::numeric_limits<dReal>::max())
                    {
                        valid = false;
                        break;
                    }
                if (valid)
                {
                    found = true;
                    if (aabb[0] < mX) mX = aabb[0];
                    if (aabb[1] > MX) MX = aabb[1];
                    if (aabb[2] < mY) mY = aabb[2];
                    if (aabb[3] > MY) MY = aabb[3];
                    if (aabb[4] < mZ) mZ = aabb[4];
                    if (aabb[5] > MZ) MZ = aabb[5];
                }
            }
        }
    }

    if (found)
    {
        double dx = MX - mX;
        double dy = MY - mY;
        double dz = MZ - mZ;
        double dM = std::max(dx, std::max(dy, dz));

        // add 10% in each dimension + 1% of the max dimension
        dx = dx * 1.10 + dM * 0.01;
        dy = dy * 1.10 + dM * 0.01;
        dz = dz * 1.10 + dM * 0.01;

        bounds1.low[0] = mX - dx;
        bounds1.high[0] = MX + dx;
        bounds1.low[1] = mY - dy;
        bounds1.high[1] = MY + dy;
        bounds1.low[2] = mZ - dz;
        bounds1.high[2] = MZ + dz;

        setVolumeBounds(bounds1);
    }
    
    // limit all velocities to 1 m/s, 1 rad/s, respectively
    bounds1.setLow(-1);
    bounds1.setHigh(1);
    setLinearVelocityBounds(bounds1);
    setAngularVelocityBounds(bounds1);
}

void ompl::control::OMPLTVSStateSpace::copyState(base::State *destination, const base::State *source) const
{
    CompoundStateSpace::copyState(destination, source);
    destination->as<StateType>()->collision = source->as<StateType>()->collision;
}

bool ompl::control::OMPLTVSStateSpace::evaluateCollision(const base::State *state) const
{
    if (state->as<StateType>()->collision & (1 << STATE_COLLISION_KNOWN_BIT))
        return state->as<StateType>()->collision & (1 << STATE_COLLISION_VALUE_BIT);
    env_->mutex_.lock();
    writeState(state);
    bool collision = env_->env_->evaluateCollision();
    env_->mutex_.unlock();
    if (collision)
        state->as<StateType>()->collision &= (1 << STATE_COLLISION_VALUE_BIT);
    state->as<StateType>()->collision &= (1 << STATE_COLLISION_KNOWN_BIT);
    return collision;
}

bool ompl::control::OMPLTVSStateSpace::satisfiesBoundsExceptRotation(const StateType *state) const
{
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
        if (i % 4 != 3)
            if (!components_[i]->satisfiesBounds(state->components[i]))
                return false;
    return true;
}

void ompl::control::OMPLTVSStateSpace::setVolumeBounds(const base::RealVectorBounds &bounds) {
    components_[0 /* position */]->as<base::RealVectorStateSpace>()->setBounds(bounds);
}

void ompl::control::OMPLTVSStateSpace::setLinearVelocityBounds(const base::RealVectorBounds &bounds) {
    components_[1 /* lin vel */]->as<base::RealVectorStateSpace>()->setBounds(bounds);
}

void ompl::control::OMPLTVSStateSpace::setAngularVelocityBounds(const base::RealVectorBounds &bounds) {
    components_[2 /* ang vel */]->as<base::RealVectorStateSpace>()->setBounds(bounds);
}

ompl::base::State* ompl::control::OMPLTVSStateSpace::allocState() const
{
    StateType *state = new StateType();
    allocStateComponents(state);
    return state;
}

void ompl::control::OMPLTVSStateSpace::freeState(base::State *state) const
{
    CompoundStateSpace::freeState(state);
}

// this function should most likely not be used with OMPLTVS propagations, but just in case it is called, we need to make sure the collision information
// is cleared from the resulting state
void ompl::control::OMPLTVSStateSpace::interpolate(const base::State *from, const base::State *to, const double t, base::State *state) const
{
    CompoundStateSpace::interpolate(from, to, t, state);
    state->as<StateType>()->collision = 0;
}

/// @cond IGNORE
namespace ompl
{
    namespace control
    {
        // we need to make sure any collision information is cleared when states are sampled (just in case this ever happens)
        class WrapperForOMPLTVSSampler : public ompl::base::StateSampler
        {
        public:
            WrapperForOMPLTVSSampler(const base::StateSpace *space, const base::StateSamplerPtr &wrapped) : base::StateSampler(space), wrapped_(wrapped)
            {
            }

            virtual void sampleUniform(ompl::base::State *state)
            {
                wrapped_->sampleUniform(state);
                state->as<OMPLTVSStateSpace::StateType>()->collision = 0;
            }

            virtual void sampleUniformNear(base::State *state, const base::State *near, const double distance)
            {
                wrapped_->sampleUniformNear(state, near, distance);
                state->as<OMPLTVSStateSpace::StateType>()->collision = 0;
            }

            virtual void sampleGaussian(base::State *state, const base::State *mean, const double stdDev)
            {
                wrapped_->sampleGaussian(state, mean, stdDev);
                state->as<OMPLTVSStateSpace::StateType>()->collision = 0;
            }
        private:
            base::StateSamplerPtr wrapped_;
        };
    }
}
/// @endcond

ompl::base::StateSamplerPtr ompl::control::OMPLTVSStateSpace::allocDefaultStateSampler() const
{
    base::StateSamplerPtr sampler = base::CompoundStateSpace::allocDefaultStateSampler();
    return base::StateSamplerPtr(new WrapperForOMPLTVSSampler(this, sampler));
}

ompl::base::StateSamplerPtr ompl::control::OMPLTVSStateSpace::allocStateSampler() const
{
    base::StateSamplerPtr sampler = base::CompoundStateSpace::allocStateSampler();
    if (dynamic_cast<WrapperForOMPLTVSSampler*>(sampler.get()))
        return sampler;
    else
        return base::StateSamplerPtr(new WrapperForOMPLTVSSampler(this, sampler));
}

void ompl::control::OMPLTVSStateSpace::readState(base::State *state) const
{
    StateType *s = state->as<StateType>();

    const dReal *pos = env_->env_->v->getPosition();
    const dReal *vel = env_->env_->v->getLinearVel();
    const dReal *ang = env_->env_->v->getAngularVel();
    const dReal *rot = env_->env_->v->getQuaternion();
    double *s_pos = s->as<base::RealVectorStateSpace::StateType>(0)->values;
    double *s_vel = s->as<base::RealVectorStateSpace::StateType>(1)->values;
    double *s_ang = s->as<base::RealVectorStateSpace::StateType>(2)->values;
    base::SO3StateSpace::StateType &s_rot = *s->as<base::SO3StateSpace::StateType>(3);

    for (int j = 0; j < 3; ++j)
    {
        s_pos[j] = pos[j];
        s_vel[j] = vel[j];
        s_ang[j] = ang[j];
    }

    s_rot.w = rot[0];
    s_rot.x = rot[1];
    s_rot.y = rot[2];
    s_rot.z = rot[3];

    s->collision = 0;
}

void ompl::control::OMPLTVSStateSpace::writeState(const base::State *state) const
{
    const StateType *s = state->as<StateType>();

    double *s_pos = s->as<base::RealVectorStateSpace::StateType>(0)->values;
    double *s_vel = s->as<base::RealVectorStateSpace::StateType>(1)->values;
    double *s_ang = s->as<base::RealVectorStateSpace::StateType>(2)->values;
    const base::SO3StateSpace::StateType &s_rot = *s->as<base::SO3StateSpace::StateType>(3);
    
    dQuaternion q;
    q[0] = s_rot.w;
    q[1] = s_rot.x;
    q[2] = s_rot.y;
    q[3] = s_rot.z;
    
    env_->env_->v->setPosition(s_pos);
    env_->env_->v->setVel(s_vel, s_ang);
    env_->env_->v->setQuaternion(q);
}
