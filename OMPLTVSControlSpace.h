//
//  OMPLTVSControlSpace.h
//  tvs
//
//  Created by Federico Ferri on 03/01/2015.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#ifndef OMPL_EXTENSION_OMPLTVS_CONTROL_SPACE_
#define OMPL_EXTENSION_OMPLTVS_CONTROL_SPACE_

#include <ompl/control/spaces/RealVectorControlSpace.h>
#include "OMPLTVSStateSpace.h"

class OMPLTVSControlSampler : public ompl::control::ControlSampler {
public:
    OMPLTVSControlSampler(const ompl::control::ControlSpace *space) : ompl::control::ControlSampler(space) {}
    virtual void sample(ompl::control::Control *control);
protected:
    ompl::RNG rng_;
};

class OMPLTVSControlSpace : public ompl::control::RealVectorControlSpace {
public:
    OMPLTVSControlSpace(const ompl::base::StateSpacePtr &stateSpace);
    virtual ~OMPLTVSControlSpace() {}
    const OMPLTVSEnvironmentPtr& getEnvironment() const {return stateSpace_->as<OMPLTVSStateSpace>()->getEnvironment();}
    virtual ompl::control::ControlSamplerPtr allocDefaultControlSampler(void) const {return ompl::control::ControlSamplerPtr(new OMPLTVSControlSampler(this));}
};

#endif
