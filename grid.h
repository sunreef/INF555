//
// Created by victor on 24/09/15.
//

#ifndef INF555_PROJECT_GRID_H
#define INF555_PROJECT_GRID_H

#include <thread>
#include <algorithm>
#include <mutex>
#include <deque>
#include "cell.h"

using namespace std;


class Grid {
public:
    double sizeThreshold = 0.4;
    double timeStep;

    Vect corner;
    int numberOfParticles;
    vector<Cell> cells;

    vector<shared_ptr<Particle>> particles;


    Grid(Vect c, int r, double thresh, double time);

    ~Grid();

    int rows;

    bool insert(shared_ptr<Particle> p);

    int getNumberOfParticles();

    Cell getCell(int cell);

    void neighbours(shared_ptr<Particle>& p, double l);

    void computeNeighbours();

    void update(shared_ptr<Particle> p);
    void update();

    void remove(shared_ptr<Particle> p);

    shared_ptr<Particle> getParticle(int i);


};


#endif //INF555_PROJECT_GRID_H
