//
//  Environment.h
//  tvs
//
//  Created by Federico Ferri on 17/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#ifndef UTILS_H_INCLUDED
#define UTILS_H_INCLUDED

template <typename T> int sgn(T val);

std::string getDateTimeString(const char *format = "%Y%m%d%H%M%S");

// linear profile integrator
class LinVelProfInt {
private:
    double x, tx, vmax;
public:
    LinVelProfInt();
    virtual ~LinVelProfInt();
    void step(double timeStep);
    double get();
    void reset();
    void set(double target);
    void setSlope(double slope);
};

#endif // UTILS_H_INCLUDED

