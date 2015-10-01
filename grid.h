//
// Created by victor on 24/09/15.
//

#ifndef INF555_PROJECT_GRID_H
#define INF555_PROJECT_GRID_H

#include "cell.h"

using namespace std;


class Grid {
    double sizeThreshold = 0.8;

    Vect corner;
    double size;
    int numberOfParticles;


    vector<Cell> cells;

public:
    Grid(Vect c, double s);

    ~Grid();

    int rows;

    bool insert(Particle &p);

    int getNumberOfParticles();

    Cell getCell(int x, int y, int z);

    void neighbours(const Particle &p, double l, vector<Particle*> result);

};


#endif //INF555_PROJECT_GRID_H
