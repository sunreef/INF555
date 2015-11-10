//
// Created by victor on 25/09/15.
//

#include "cell.h"

Cell::Cell() : corner(0, 0, 0), size(0), particlesCount(0) {
    particles = vector< shared_ptr<Particle> >();

}

Cell::Cell(Vect c, double s) : corner(c), size(s) {
    particlesCount = 0;
}

void Cell::add(shared_ptr<Particle> p) {
    particles.push_back(p);
    particlesCount++;
}

void Cell::remove(shared_ptr<Particle> p) {
    for (vector< shared_ptr<Particle> >::iterator it = particles.begin(); it != particles.end(); it++) {
        if ((*it)->id == p->id) {
            particles.erase(it);
            particlesCount--;
            break;
        }
    }
}

Cell::~Cell() {
}

bool Cell::isEmpty() {
    return (particles.size() == 0);
}
