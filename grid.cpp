//
// Created by victor on 24/09/15.
//

#include "grid.h"

mutex mtx;
mutex mtx2;


Grid::Grid(Vect c, int r, double thresh, double time) : corner(c), rows(r), numberOfParticles(0), sizeThreshold(thresh),
                                                        timeStep(time) {

//    double wallRho = 1.0;
//    double wallPressure = 10000;
//    int sampling = 10;
//    double sampleSize = sizeThreshold / sampling;
    cells = vector<Cell>(rows * rows * rows);

    for (int x = 0; x < rows; x++) {
        for (int y = 0; y < rows; y++) {
            for (int z = 0; z < rows; z++) {
                Cell c(corner + Vect(x * sizeThreshold, y * sizeThreshold, z * sizeThreshold), sizeThreshold);
                cells[z + rows * (y + rows * x)] = c;

//                if (x == 0 || x == rows - 1) {
//                    for (int i = 0; i < sampling; i++) {
//                        for (int j = 0; j < sampling; j++) {
//                            Vect v = c.corner + Vect(((x==0) ? 0 : sizeThreshold), j * sampleSize, i * sampleSize);
//                            cout << v.x << endl;
//                            Particle p(v, 1.0, 0.1);
//                            p.rho = wallRho;
//                            p.pressure = wallPressure;
//                            c.add(make_shared<Particle>(v, 1.0, 0.1));
//                            cout << "added wall aprticle" << endl;
//                        }
//                    }
//                    cout << c.particles.size() << endl;
//                }
//                if (y == 0 || y == rows - 1) {
//                    for (int i = 0; i < sampling; i++) {
//                        for (int j = 0; j < sampling; j++) {
//                            Vect v = c.corner + Vect(j * sampleSize,((y==0) ? 0 : sizeThreshold), i * sampleSize);
//                            Particle p(v, 1.0, 0.1);
//                            p.rho = wallRho;
//                            p.pressure = wallPressure;
//                            c.add(make_shared<Particle>(p));
//                        }
//                    }
//                }
//                if (z == 0 || z == rows - 1) {
//                    for (int i = 0; i < sampling; i++) {
//                        for (int j = 0; j < sampling; j++) {
//                            Vect v = c.corner + Vect(j * sampleSize, i * sampleSize, ((z==0) ? 0 : sizeThreshold));
//                            Particle p(v, 1.0, 0.1);
//                            p.rho = wallRho;
//                            p.pressure = wallPressure;
//                            c.add(make_shared<Particle>(p));
//                        }
//                    }
//                }

            }
        }
    }

}

Grid::~Grid() {
}

bool Grid::insert(shared_ptr<Particle> p) {
    Vect v = p->pos - corner;

    int x = v.x / sizeThreshold;
    int y = v.y / sizeThreshold;
    int z = v.z / sizeThreshold;

    if (x < 0 || y < 0 || z < 0 || x >= rows || y >= rows || z >= rows) {
        cout << x << ", " << y << ", " << z << endl;
        cout << "The particle is not inside the grid. It won't be inserted." << endl;
        return false;
    }
    else {
        cells[z + rows * (y + rows * x)].add(p);
        particles.push_back(p);

        p->cellX = x;
        p->cellY = y;
        p->cellZ = z;
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


void Grid::neighbours(shared_ptr<Particle> &p, double l) {
    p->neighbours.clear();
    l *= l;
    for (int x = max(0, p->cellX - 1); x <= min(rows - 1, p->cellX + 1); x++) {
        for (int y = max(0, p->cellY - 1); y <= min(rows - 1, p->cellY + 1); y++) {
            for (int z = max(0, p->cellZ - 1); z <= min(rows - 1, p->cellZ + 1); z++) {
                int cell = z + rows * (y + rows * x);
                Cell c = cells[cell];
                for (shared_ptr<Particle> p2: c.particles) {
                    double norm = (p->pos - p2->pos).norm();
                    if (norm < l && norm > 0) {
                        p->neighbours.insert(p2);
                    }
                }
            }
        }
    }
}

void Grid::update() {

    double speedRatio = 1.0;

    for (Cell c: cells) {
        vector<shared_ptr<Particle>> toRemove, cellRemove;
        for (shared_ptr<Particle> p:c.particles) {
            p->pos += p->speed * timeStep;

            if (p->pos.x < corner.x) {
                p->pos.x = 2 * (corner.x) - p->pos.x;
                p->speed.x *= -speedRatio;
            }

            if (p->pos.y < corner.y) {
                p->pos.y = 2 * (corner.y) - p->pos.y;
                p->speed.y *= -speedRatio;
            }

            if (p->pos.z < corner.z) {
                p->pos.z = 2 * (corner.z) - p->pos.z;
                p->speed.z *= -speedRatio;
            }
            double size = sizeThreshold * (rows);

            if (p->pos.x > corner.x + size) {
                p->pos.x = 2 * (corner.x + size) - p->pos.x;
                p->speed.x *= -speedRatio;
            }

            if (p->pos.y > corner.y + size) {
                p->pos.y = 2 * (corner.y + size) - p->pos.y;
                p->speed.y *= -speedRatio;
            }

            if (p->pos.z > corner.z + size) {
                p->pos.z = 2 * (corner.z + size) - p->pos.z;
                p->speed.z *= -speedRatio;
            }

            Vect v = p->pos;
            int x = v.x / sizeThreshold;
            int y = v.y / sizeThreshold;
            int z = v.z / sizeThreshold;
            if (x < 0 || y < 0 || z < 0 || x > rows || y > rows || z > rows) {
                toRemove.push_back(p);

//                cout << p->pos.x << "  " << p->pos.y << "  " << p->pos.z << endl;
                cout << p->rho << endl;
                cout << "The particle is not inside the grid. It won't be inserted." << endl;
                continue;
            }
            if (x != p->cellX || y != p->cellY || z != p->cellZ) {
                cellRemove.push_back(p);
                cells[z + rows * (y + rows * x)].add(p);

                p->cellX = x;
                p->cellY = y;
                p->cellZ = z;
            }
        }
        for (shared_ptr<Particle> p:toRemove) {
            c.remove(p);
            remove(p);
        }
        for (shared_ptr<Particle> p: cellRemove) {
            c.remove(p);
        }
    }
}

//void Grid::update(shared_ptr<Particle> p) {
//
//    double speedRatio = 0.5;
//
//    p->pos += p->speed * timeStep;
//
//    if (p->pos.x < corner.x) {
//        p->pos.x = 2 * (corner.x) - p->pos.x;
//        p->speed.x *= -speedRatio;
//    }
//
//    if (p->pos.y < corner.y) {
//        p->pos.y = 2 * (corner.y) - p->pos.y;
//        p->speed.y *= -speedRatio;
//    }
//
//    if (p->pos.z < corner.z) {
//        p->pos.z = 2 * (corner.z) - p->pos.z;
//        p->speed.z *= -speedRatio;
//    }
//    double size = sizeThreshold * rows;
//
//    if (p->pos.x > corner.x + size) {
//        p->pos.x = 2 * (corner.x + size) - p->pos.x;
//        p->speed.x *= -speedRatio;
//    }
//
//    if (p->pos.y > corner.y + size) {
//        p->pos.y = 2 * (corner.y + size) - p->pos.y;
//        p->speed.y *= -speedRatio;
//    }
//
//    if (p->pos.z > corner.z + size) {
//        p->pos.z = 2 * (corner.z + size) - p->pos.z;
//        p->speed.z *= -speedRatio;
//    }
//
//    Vect v = p->pos;
//    int x = v.x / sizeThreshold;
//    int y = v.y / sizeThreshold;
//    int z = v.z / sizeThreshold;
//    if (x < 0 || y < 0 || z < 0 || x >= rows || y >= rows || z >= rows) {
//        Cell c = cells[p->cellZ + rows * (p->cellY + rows * p->cellX)];
//        c.remove(p);
//        cout << p->pos.x << "  " << p->pos.y << "  " << p->pos.z << endl;
//        cout << "The particle is not inside the grid. It won't be inserted." << endl;
//        return;
//    }
//    if (x != p->cellX || y != p->cellY || z != p->cellZ) {
//        Cell c = cells[p->cellZ + rows * (p->cellY + rows * p->cellX)];
//        c.remove(p);
//        cells[z + rows * (y + rows * x)].add(p);
//
//        p->cellX = x;
//        p->cellY = y;
//        p->cellZ = z;
//    }
//}

void Grid::remove(shared_ptr<Particle> p) {
    for (vector<shared_ptr<Particle>>::iterator it = particles.begin(); it != particles.end(); it++) {
        if ((*it)->id == p->id) {
            particles.erase(it);
            numberOfParticles--;
            break;
        }
    }
}

shared_ptr<Particle> Grid::getParticle(int i) {
    if (i > numberOfParticles - 1) {
        cout << "Error: index out of bonds when querying particle";
        return 0;
    }

    return particles[i];
}


void parallelNeighbourSearch(Grid &g, vector<shared_ptr<Particle>> &particles, int &count,
                             double l, int id) {
    bool loop = true;
    while (loop) {
        mtx.lock();
        int temp = count;
        count++;
        mtx.unlock();
        if (temp > particles.size() - 1) {
            break;
        }
        else {
            shared_ptr<Particle> p = particles[temp];
            g.neighbours(p, l);
        }
    }

}

void Grid::computeNeighbours() {
    int numberOfThreads = 8;

    vector<thread> threads;
    int count = 0;
    vector<int> progress(numberOfThreads);
    for (int i = 0; i < numberOfThreads; i++) {
        progress[i] = -1;
    }
    cout << "Calculating neighbours using " << numberOfThreads << " threads..." << endl;

    for (int i = 0; i < numberOfThreads; i++) {
        threads.push_back(thread(&parallelNeighbourSearch, std::ref(*this), std::ref(particles),
                                 std::ref(count), sizeThreshold, i));
    }
    for (int i = 0; i < numberOfThreads; i++) {
        threads[i].join();
    }

    cout << "Done" << endl;
}
