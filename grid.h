//
// Created by victor on 24/09/15.
//

#ifndef INF555_PROJECT_GRID_H
#define INF555_PROJECT_GRID_H

#include "cell.h"

using namespace std;


class Grid {
    double sizeThreshold = 0.2;

    Vect corner;
    double size;
    int numberOfParticles;


    vector<Cell> cells;
    vector<shared_ptr<Particle>> particles;

public:
    Grid(Vect c, double s);

    ~Grid();

    int rows;

    bool insert(Particle &p);

    int getNumberOfParticles();

    Cell getCell(int x, int y, int z);

    void neighbours(shared_ptr<Particle> p, double l);

    void update(Particle p);


//    template< Particle>
//    struct GridIterator : std::iterator<Particle> {
//
//        GridIterator(Grid &g) : currentCell(0), currentParticle(0) {
//            grid = make_shared(g);
//            while (grid->cells[currentCell].isEmpty() && currentCell < grid->cells.size()) {
//                currentCell++;
//            }
//        }
//
//        GridIterator &begin() {
//            GridIterator b(*grid);
//            b.currentCell = 0;
//        }
//
//        GridIterator &operator++() {
//            if (currentParticle == grid->cells[currentCell].particles.size() - 1) {
//
//                while (grid->cells[currentCell].isEmpty() && currentCell < grid->cells.size()) {
//                    currentCell++;
//                }
//                currentParticle = 0;
//
//                return *this;
//            }
//            else {
//                currentParticle++;
//                return *this;
//            }
//        }
//
//
//    private:
//        shared_ptr<Grid> grid;
//        int currentCell;
//        int currentParticle;
//    };

};


#endif //INF555_PROJECT_GRID_H
