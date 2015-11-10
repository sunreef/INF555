//
// Created by victor on 24/09/15.
//

#include "particle.h"

int Particle::particlesCount = 0;


Particle::Particle(double x, double y, double z, double weight, double radius) : pos(x, y, z), w(weight), r(radius),
                                                                                 speed(0, 0, 0), id(particlesCount),
                                                                                 cellX(0), cellZ(0), cellY(0) {
    particlesCount++;
    neighbours = set< shared_ptr<Particle> >();
}

void Particle::print() const {
    std::cout << "Particle " << id << endl;
    std::cout << "Position: (" << pos.x << ", " << pos.y << ", " << pos.z << ")  ";
    std::cout << "Weight: " << w << "  Radius: " << r << std::endl;
}

Particle::Particle(Vect &p, double weight, double radius) : pos(p), w(weight), r(radius), speed(0, 0, 0),
                                                            id(particlesCount), cellX(0), cellZ(0), cellY(0) {
    particlesCount++;
    neighbours = set< shared_ptr<Particle> >();
}
