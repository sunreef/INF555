//
// Created by victor on 24/09/15.
//

#ifndef INF555_PROJECT_PARTICLE_H
#define INF555_PROJECT_PARTICLE_H

#include "vect.h"
#include <iostream>

struct Particle {
    Vect pos;
    Vect speed;
    double w;
    double r;
    double rho;
    double pressure;
    int cellX, cellY, cellZ;

    Particle(double x, double y, double z, double weight, double radius);

    Particle(Vect &p, double weight, double radius);

    void print() const;
};


#endif //INF555_PROJECT_PARTICLE_H
