//
//  physique.cpp
//  INF555___Project
//
//  Created by Galashov Alexandr on 08/11/2015.
//
//


#include "physique.hpp"

#include <iostream>



using namespace std;
/*
Vect computeViscosityForce(shared_ptr<Particle> p, Kernel &w, double viscosity) {
    Vect result(0, 0, 0);
    
    for (shared_ptr<Particle> n: p->neighbours) {
        // masse / rho
        double temp = n->w / n->rho;
        // xij
        Vect xij = p->pos - n->pos;
        double norm = xij.norm();
        temp *= xij * w.grad(p->pos, n->pos, norm);
        temp /= norm + 0.01 * w.getSmoothingDistance() * w.getSmoothingDistance();
        result += (p->speed - n->speed) * temp;
    }
    // you forgot 2
    result *= 2;
    
    result *= viscosity;
    
    
    return result;
}

double computeNewRho(shared_ptr<Particle> p, Kernel &w, double timeStep) {
    double sum1 = 0;
    double sum2 = 0;
    
    for (shared_ptr<Particle> n: p->neighbours) {
        double norm = (n->pos - p->pos).norm();
        sum1 += n->w * w(norm);
        sum2 += (p->speed - n->speed) * w.grad(p->pos, n->pos, norm);
    }
    return sum1 + timeStep * sum2;
}

Vect computePressureForce(shared_ptr<Particle> p, Kernel &w) {
    Vect result(0, 0, 0);
    
    for (shared_ptr<Particle> n: p->neighbours) {
        result += w.grad(p->pos, n->pos) * ((p->pressure / (p->rho * p->rho) + n->pressure / (n->rho * n->rho)) * n->w);
    }
    
    
    // there is a minus here, so
    result *=-1;
    
    return result;
}

*/