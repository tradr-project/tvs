//
//  OMPLControlSpace.cpp
//  tvs
//
//  Created by Main User on 18/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "OMPLControlSpace.h"

OMPLControlSpace::OMPLControlSpace(const ob::StateSpacePtr &stateSpace) : RealVectorControlSpace(stateSpace, 2) {
    setName("OMPLControlSpace");
    ob::RealVectorBounds bounds(2);
    bounds.setLow(1.1);
    bounds.setHigh(1.1);
    setBounds(bounds);
}

OMPLControlSpace::~OMPLControlSpace() {
}
