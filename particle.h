//
// Created by victor on 24/09/15.
//

#ifndef INF555_PROJECT_PARTICLE_H
#define INF555_PROJECT_PARTICLE_H

#include "vect.h"
#include <iostream>
#include <vector>
#include <memory>

using namespace std;


class Particle {

public:
    static int particlesCount;
    Vect pos;
    Vect speed;
    double w;
    double r;
    double rho;
    double pressure;
    int cellX, cellY, cellZ;
    vector<shared_ptr<Particle>> neighbours;

    Particle(double x, double y, double z, double weight, double radius);

    Particle(Vect &p, double weight, double radius);

    void print() const;


    int getId() { return id; }

private:

    int id;
};



#endif //INF555_PROJECT_PARTICLE_H
