//
// Created by victor on 25/09/15.
//

#include "cell.h"

Cell::Cell() : corner(0, 0, 0), size(0), particlesCount(0) {

}

Cell::Cell(Vect c, double s) : corner(c), size(s) {

}

void Cell::add(Particle p) {
    particles.push_back(p);
    particlesCount++;
}

Cell::~Cell() {
}
