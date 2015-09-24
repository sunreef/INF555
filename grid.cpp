//
// Created by victor on 24/09/15.
//

#include "grid.h"

Grid::Grid(Grid *p, Vect c, double s) : center(c), size(s), parent(p) {

    if (size <= sizeThreshold) {
        return;
    }
    else {
        children = vector<Grid *>(8);
        int k = 0;
        for (double x = -0.25; x < 0.5; x += 0.5) {
            for (double y = -0.25; y < 0.5; y += 0.5) {
                for (double z = -0.25; z < 0.5; z += 0.5) {
                    children[k] = new Grid(this, (center + Vect(x * size, y * size, z * size)), size / 2);
                    k++;
                }
            }
        }
    }

}

Grid::~Grid() {
    for (Grid *c: children) {
        delete c;
    }
}

void Grid::insert(Particle &p) {
    Vect v = p.pos;

    if (children.size() == 0) {
        particles.push_back(&p);
//
//        p.print();
//        std::cout << center.x << ", " << center.y << ", " << center.z << std::endl;
        return;
    }
    else {
        int x, y, z;
        if (v.x < center.x) {
            x = 0;
        }
        else {
            x = 1;
        }

        if (v.y < center.y) {
            y = 0;
        }
        else {
            y = 1;
        }

        if (v.z < center.z) {
            z = 0;
        }
        else {
            z = 1;
        }

        children[4 * x + 2 * y + z]->insert(p);
    }

}
