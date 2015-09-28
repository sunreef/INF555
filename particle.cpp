//
// Created by victor on 24/09/15.
//

#include "particle.h"


Particle::Particle(double x, double y, double z, double weight, double radius) : pos(x, y, z), w(weight), r(radius),
                                                                                 speed(0, 0, 0) {
}

void Particle::print() const {
    std::cout << "Position: (" << pos.x << ", " << pos.y << ", " << pos.z << ")  ";
    std::cout << "Weight: " << w << "  Radius: " << r << std::endl;
}

Particle::Particle(Vect &p, double weight, double radius) : pos(p), w(weight), r(radius), speed(0, 0, 0) {
}
