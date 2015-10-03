//
// Created by victor on 24/09/15.
//

#include "grid.h"

mutex mtx;

Grid::Grid(Vect c, double s) : corner(c), size(s), numberOfParticles(0) {
    rows = (int) (size / sizeThreshold);

    cells = vector<Cell>(rows * rows * rows);

    for (int x = 0; x < rows; x++) {
        for (int y = 0; y < rows; y++) {
            for (int z = 0; z < rows; z++) {
                cells[z + rows * (y + rows * x)] = Cell(
                        corner + Vect(x * sizeThreshold, y * sizeThreshold, z * sizeThreshold), sizeThreshold);
            }
        }
    }

}

Grid::~Grid() {
}

bool Grid::insert(Particle &p) {
    Vect v = p.pos - corner;

    int x = v.x / sizeThreshold;
    int y = v.y / sizeThreshold;
    int z = v.z / sizeThreshold;

    if (x < 0 || y < 0 || z < 0 || x >= rows || y >= rows || z >= rows) {
        cout << "The particle is not inside the grid. It won't be inserted." << endl;
        return false;
    }
    else {
        shared_ptr<Particle> s = make_shared<Particle>(p);
        cells[z + rows * (y + rows * x)].add(s);
        particles.push_back(s);
        p.cellX = x;
        p.cellY = y;
        p.cellZ = z;
        numberOfParticles++;
        return true;
    }
}

Cell Grid::getCell(int cell) {
    return cells[cell];
}

int Grid::getNumberOfParticles() {
    return numberOfParticles;
}


void parallelSearch(shared_ptr<Particle> p, Grid *g, int cell, double l) {

}


void Grid::neighbours(shared_ptr<Particle> &p, double l) {
    for (int x = max(0, p->cellX - 1); x <= min(rows - 1, p->cellX + 1); x++) {
        for (int y = max(0, p->cellY - 1); y <= min(rows - 1, p->cellY + 1); y++) {
            for (int z = max(0, p->cellZ - 1); z <= min(rows - 1, p->cellZ + 1); z++) {
                int cell = z + rows * (y + rows * x);
                Cell c = getCell(cell);
                for (int i = 0; i < c.particlesCount; i++) {
                    shared_ptr<Particle> p2 = c.particles[i];
                    double norm = (p->pos - p2->pos).norm();
                    if (norm < l * l && norm > 0) {
                        p->neighbours.push_back(p2);
                    }
                }
            }
        }
    }
}

void Grid::update(Particle &p) {
    int id = p.getId();
    int x = p.pos.x / sizeThreshold;
    int y = p.pos.y / sizeThreshold;
    int z = p.pos.z / sizeThreshold;

    p.cellX = x;
    p.cellY = y;
    p.cellZ = z;
}

shared_ptr<Particle> Grid::getParticle(int i) {
    if (i > numberOfParticles - 1) {
        cout << "Error: index out of bonds when querying particle";
        return 0;
    }

    return particles[i];
}


void parallelNeighbourSearch(Grid &g, const vector<shared_ptr<Particle>> &particles, int &count, double l) {
    bool loop = true;
    while (loop) {
        mtx.lock();
        shared_ptr<Particle> p = particles[count];
        count++;
        mtx.unlock();
        if (count > particles.size() - 1) {
            break;
        }
        else {
            g.neighbours(p, l);
        }
    }

}

void Grid::computeNeighbours(double l) {

    int numberOfThreads = 8;

    vector<thread> threads;
    int count = 0;

    for (int i = 0; i < numberOfThreads; i++) {
        threads.push_back(thread(&parallelNeighbourSearch, std::ref(*this), std::cref(particles), std::ref(count), l));
    }
    for (int i = 0; i < numberOfThreads; i++) {
        threads[i].join();
        cout << "Thread " << i << " stopped" << endl;
    }

    cout << count << endl;
}
