//
// Created by victor on 25/09/15.
//

#ifndef INF555_PROJECT_CELL_H
#define INF555_PROJECT_CELL_H

#include "particle.h"
#include <vector>
#include <memory>

using namespace std;

struct Cell {

    double size;
    Vect corner;

    vector<shared_ptr<Particle>> particles;
    int particlesCount;

    Cell();

    Cell(Vect c, double s);

    ~Cell();

    void add(shared_ptr<Particle> p);
    bool isEmpty();


    void remove(shared_ptr<Particle> p);
};


#endif //INF555_PROJECT_CELL_H
