//
// Created by victor on 24/09/15.
//

#ifndef INF555_PROJECT_GRID_H
#define INF555_PROJECT_GRID_H

#include "cell.h"

using namespace std;


class Grid {
    static constexpr double sizeThreshold = 0.4;

    Vect corner;
    double size;
    int numberOfParticles;
    int rows;

    vector<Cell> cells;

public:
    Grid(Vect c, double s);

    ~Grid();

    void insert(const Particle &p);

    int getNumberOfParticles();

    Cell getCell(int x, int y, int z);

};


#endif //INF555_PROJECT_GRID_H
