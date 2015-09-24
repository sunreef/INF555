//
// Created by victor on 24/09/15.
//

#ifndef INF555_PROJECT_GRID_H
#define INF555_PROJECT_GRID_H

#include <vector>
#include "vect.h"
#include "particle.h"

using namespace std;


class Grid {
    static constexpr double sizeThreshold = 0.4;
    Vect center;
    double size;

    Grid *parent;
    vector<Grid *> children;
    vector<Particle *> particles;

public:
    Grid(Grid *p, Vect c, double s);

    ~Grid();

    void insert(Particle &p);

};


#endif //INF555_PROJECT_GRID_H
