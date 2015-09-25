//
// Created by victor on 25/09/15.
//

#ifndef INF555_PROJECT_CELL_H
#define INF555_PROJECT_CELL_H

#include "vect.h"
#include "particle.h"
#include <vector>

using namespace std;

struct Cell {

    double size;
    Vect corner;

    vector<Particle *> particles;
    int particlesCount;

    Cell();

    Cell(Vect c, double s);

    ~Cell();

    void add(Particle p);


};


#endif //INF555_PROJECT_CELL_H
