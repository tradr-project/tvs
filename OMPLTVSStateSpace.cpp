//
//  OMPLTVSStateSpace.cpp
//  tvs
//
//  Created by Federico Ferri on 03/01/2015.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "OMPLTVSStateSpace.h"
#include <ompl/util/Console.h>
#include <boost/lexical_cast.hpp>
#include <limits>
#include <queue>

OMPLTVSStateProjectionEvaluator::OMPLTVSStateProjectionEvaluator(const ompl::base::StateSpace *space) : ompl::base::ProjectionEvaluator(space) {
}

unsigned int OMPLTVSStateProjectionEvaluator::getDimension(void) const {
    return 3;
}

void OMPLTVSStateProjectionEvaluator::defaultCellSizes(void) {
    cellSizes_.resize(3);
    cellSizes_[0] = 0.2;
    cellSizes_[1] = 0.2;
    cellSizes_[2] = 0.1;
}

void OMPLTVSStateProjectionEvaluator::project(const ompl::base::State *state, ompl::base::EuclideanProjection &projection) const {
    const double *pos = state->as<OMPLTVSStateSpace::StateType>()->getBodyPosition(0);
    projection[0] = pos[0];
    projection[1] = pos[1];
    projection[2] = pos[2];
}


OMPLTVSStateSpace::OMPLTVSStateSpace(const OMPLTVSEnvironmentPtr &env, double positionWeight, double linVelWeight, double angVelWeight, double orientationWeight)
: ompl::base::CompoundStateSpace(), env_(env) {
    setName("OMPLTVS" + getName());
    type_ = ompl::base::STATE_SPACE_TYPE_COUNT + 1;

    std::string body = ""; // unused

    for(size_t i = 0; i < dRigidBodyArraySize(env_->env_->v->bodyArray); i++) {
        std::string body = ":B" + boost::lexical_cast<std::string>(i);

        addSubspace(ompl::base::StateSpacePtr(new ompl::base::RealVectorStateSpace(3)), positionWeight); // position
        components_.back()->setName(components_.back()->getName() + body + ":position");
        
        addSubspace(ompl::base::StateSpacePtr(new ompl::base::RealVectorStateSpace(3)), linVelWeight);   // linear velocity
        components_.back()->setName(components_.back()->getName() + body + ":linvel");
        
        addSubspace(ompl::base::StateSpacePtr(new ompl::base::RealVectorStateSpace(3)), angVelWeight);   // angular velocity
        components_.back()->setName(components_.back()->getName() + body + ":angvel");
        
        addSubspace(ompl::base::StateSpacePtr(new ompl::base::SO3StateSpace()), orientationWeight);      // orientation
        components_.back()->setName(components_.back()->getName() + body + ":orientation");
    }

    lock(); // prevents modification of subspaces
    setDefaultBounds();
}

void OMPLTVSStateSpace::setDefaultBounds() {
    ompl::base::RealVectorBounds bounds1(3);

    // find the bounding box that contains all geoms included in the collision spaces
    double mX, mY, mZ, MX, MY, MZ;
    mX = mY = mZ = std::numeric_limits<double>::infinity();
    MX = MY = MZ = -std::numeric_limits<double>::infinity();
    bool found = false;

    std::queue<dSpaceID> spaces;
    spaces.push(env_->env_->space);

    while(!spaces.empty()) {
        dSpaceID space = spaces.front();
        spaces.pop();

        int n = dSpaceGetNumGeoms(space);

        for(int j = 0 ; j < n ; ++j) {
            dGeomID geom = dSpaceGetGeom(space, j);
            if(dGeomIsSpace(geom)) {
                spaces.push((dSpaceID)geom);
            } else {
                bool valid = true;
                dReal aabb[6];
                dGeomGetAABB(geom, aabb);

                // things like planes are infinite; we want to ignore those
                for(int k = 0 ; k < 6 ; ++k)
                    if(fabs(aabb[k]) >= std::numeric_limits<dReal>::max()) {
                        valid = false;
                        break;
                    }
                if(valid) {
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

    if(found) {
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
    
    bounds1.setLow(-2);
    bounds1.setHigh(2);
    setLinearVelocityBounds(bounds1);
    
    bounds1.setLow(-8.0*M_PI);
    bounds1.setHigh(8.0*M_PI);
    setAngularVelocityBounds(bounds1);
}

void OMPLTVSStateSpace::copyState(ompl::base::State *destination, const ompl::base::State *source) const {
    CompoundStateSpace::copyState(destination, source);
    destination->as<StateType>()->collision = source->as<StateType>()->collision;
}

bool OMPLTVSStateSpace::evaluateCollision(const ompl::base::State *state) const {
    if(state->as<StateType>()->collision & (1 << STATE_COLLISION_KNOWN_BIT))
        return state->as<StateType>()->collision & (1 << STATE_COLLISION_VALUE_BIT);
    env_->mutex_.lock();
    writeState(state);
    bool collision = env_->env_->evaluateCollision();
    env_->mutex_.unlock();
    if(collision)
        state->as<StateType>()->collision &= (1 << STATE_COLLISION_VALUE_BIT);
    state->as<StateType>()->collision &= (1 << STATE_COLLISION_KNOWN_BIT);
    return collision;
}

bool OMPLTVSStateSpace::satisfiesBoundsExceptRotation(const StateType *state) const {
    static const char *comp_name[] = {"position", "linear_vel", "angular_vel", "rotation"};
    for(unsigned int i = 0 ; i < componentCount_ ; ++i) {
        int body = i / NUM_COMPONENTS;
        int comp = i % NUM_COMPONENTS;
        if(comp == COMPONENT_ROTATION) continue;
        if(!components_[i]->satisfiesBounds(state->components[i])) {
            std::cout << "body #" << body << " does not satisfy " << comp_name[comp] << " bounds" << std::endl;
            return false;
        }
    }
    return true;
}

void OMPLTVSStateSpace::setVolumeBounds(const ompl::base::RealVectorBounds &bounds) {
    for(size_t i = 0; i < dRigidBodyArraySize(env_->env_->v->bodyArray); i++) {
        components_[i * NUM_COMPONENTS + COMPONENT_POSITION]->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
    }
}

void OMPLTVSStateSpace::setLinearVelocityBounds(const ompl::base::RealVectorBounds &bounds) {
    for(size_t i = 0; i < dRigidBodyArraySize(env_->env_->v->bodyArray); i++) {
        components_[i * NUM_COMPONENTS + COMPONENT_LINEAR_VELOCITY]->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
    }
}

void OMPLTVSStateSpace::setAngularVelocityBounds(const ompl::base::RealVectorBounds &bounds) {
    for(size_t i = 0; i < dRigidBodyArraySize(env_->env_->v->bodyArray); i++) {
        components_[i * NUM_COMPONENTS + COMPONENT_ANGULAR_VELOCITY]->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
    }
}

ompl::base::State* OMPLTVSStateSpace::allocState() const {
    StateType *state = new StateType();
    allocStateComponents(state);
    return state;
}

void OMPLTVSStateSpace::freeState(ompl::base::State *state) const {
    CompoundStateSpace::freeState(state);
}

// this function should most likely not be used with OMPLTVS propagations, but just in case it is called, we need to make sure the collision information
// is cleared from the resulting state
void OMPLTVSStateSpace::interpolate(const ompl::base::State *from, const ompl::base::State *to, const double t, ompl::base::State *state) const {
    CompoundStateSpace::interpolate(from, to, t, state);
    state->as<StateType>()->collision = 0;
}

// we need to make sure any collision information is cleared when states are sampled (just in case this ever happens)
class WrapperForOMPLTVSSampler : public ompl::base::StateSampler {
public:
    WrapperForOMPLTVSSampler(const ompl::base::StateSpace *space, const ompl::base::StateSamplerPtr &wrapped) : ompl::base::StateSampler(space), wrapped_(wrapped) {}

    virtual void sampleUniform(ompl::base::State *state) {
        wrapped_->sampleUniform(state);
        state->as<OMPLTVSStateSpace::StateType>()->collision = 0;
    }

    virtual void sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, const double distance) {
        wrapped_->sampleUniformNear(state, near, distance);
        state->as<OMPLTVSStateSpace::StateType>()->collision = 0;
    }

    virtual void sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, const double stdDev) {
        wrapped_->sampleGaussian(state, mean, stdDev);
        state->as<OMPLTVSStateSpace::StateType>()->collision = 0;
    }

private:
    ompl::base::StateSamplerPtr wrapped_;
};

ompl::base::StateSamplerPtr OMPLTVSStateSpace::allocDefaultStateSampler() const {
    ompl::base::StateSamplerPtr sampler = ompl::base::CompoundStateSpace::allocDefaultStateSampler();
    return ompl::base::StateSamplerPtr(new WrapperForOMPLTVSSampler(this, sampler));
}

ompl::base::StateSamplerPtr OMPLTVSStateSpace::allocStateSampler() const {
    ompl::base::StateSamplerPtr sampler = ompl::base::CompoundStateSpace::allocStateSampler();
    if (dynamic_cast<WrapperForOMPLTVSSampler*>(sampler.get()))
        return sampler;
    else
        return ompl::base::StateSamplerPtr(new WrapperForOMPLTVSSampler(this, sampler));
}

void OMPLTVSStateSpace::readState(ompl::base::State *state) const {
    StateType *s = state->as<StateType>();
    
    for(size_t i = dRigidBodyArraySize(env_->env_->v->bodyArray) - 1; i >= 0; i--) {
        size_t j = i * NUM_COMPONENTS;
        dBodyID body = dRigidBodyArrayGet(env_->env_->v->bodyArray, i);

        const dReal *pos = dBodyGetPosition(body);
        const dReal *vel = dBodyGetLinearVel(body);
        const dReal *ang = dBodyGetAngularVel(body);
        const dReal *rot = dBodyGetQuaternion(body);

        double *s_pos = s->as<ompl::base::RealVectorStateSpace::StateType>(j + COMPONENT_POSITION)->values;
        double *s_vel = s->as<ompl::base::RealVectorStateSpace::StateType>(j + COMPONENT_LINEAR_VELOCITY)->values;
        double *s_ang = s->as<ompl::base::RealVectorStateSpace::StateType>(j + COMPONENT_ANGULAR_VELOCITY)->values;
        ompl::base::SO3StateSpace::StateType &s_rot = *s->as<ompl::base::SO3StateSpace::StateType>(j + COMPONENT_ROTATION);

        for (int k = 0; k < 3; ++k) {
            s_pos[k] = pos[k];
            s_vel[k] = vel[k];
            s_ang[k] = ang[k];
        }

        s_rot.w = rot[0];
        s_rot.x = rot[1];
        s_rot.y = rot[2];
        s_rot.z = rot[3];
    }
    s->collision = 0;
}

void OMPLTVSStateSpace::writeState(const ompl::base::State *state) const {
    const StateType *s = state->as<StateType>();

    for(size_t i = dRigidBodyArraySize(env_->env_->v->bodyArray) - 1; i >= 0; i--) {
        size_t j = i * NUM_COMPONENTS;
        dBodyID body = dRigidBodyArrayGet(env_->env_->v->bodyArray, i);

        double *s_pos = s->as<ompl::base::RealVectorStateSpace::StateType>(j + COMPONENT_POSITION)->values;
        double *s_vel = s->as<ompl::base::RealVectorStateSpace::StateType>(j + COMPONENT_LINEAR_VELOCITY)->values;
        double *s_ang = s->as<ompl::base::RealVectorStateSpace::StateType>(j + COMPONENT_ANGULAR_VELOCITY)->values;
        const ompl::base::SO3StateSpace::StateType &s_rot = *s->as<ompl::base::SO3StateSpace::StateType>(j + COMPONENT_ROTATION);
        
        dQuaternion q;
        q[0] = s_rot.w;
        q[1] = s_rot.x;
        q[2] = s_rot.y;
        q[3] = s_rot.z;
        
        dBodySetPosition(body, s_pos[0], s_pos[1], s_pos[2]);
        dBodySetLinearVel(body, s_vel[0], s_vel[1], s_vel[2]);
        dBodySetAngularVel(body, s_ang[0], s_ang[1], s_ang[2]);
        dBodySetQuaternion(body, q);
    }
}

double OMPLTVSStateSpace::distance(const ompl::base::State *s1, const ompl::base::State *s2) const {
    const double *p1 = s1->as<OMPLTVSStateSpace::StateType>()->getBodyPosition(0);
    const double *p2 = s2->as<OMPLTVSStateSpace::StateType>()->getBodyPosition(0);
    double dx = fabs(p1[0] - p2[0]);
    double dy = fabs(p1[1] - p2[1]);
    double dz = fabs(p1[2] - p2[2]);
    return sqrt(dx * dx + dy * dy + dz * dz);
}

void OMPLTVSStateSpace::registerProjections(void) {
    registerDefaultProjection(ompl::base::ProjectionEvaluatorPtr(new OMPLTVSStateProjectionEvaluator(this)));
}

double OMPLTVSStateSpace::getMeasure() const {
    ompl::base::RealVectorBounds b = components_[COMPONENT_POSITION]->as<ompl::base::RealVectorStateSpace>()->getBounds();
    return b.getVolume();
}
