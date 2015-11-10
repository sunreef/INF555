//
// Created by victor on 24/09/15.
//

#include "grid.h"

mutex mtx;
mutex mtx2;

Grid::Grid(Vect c, int r, double thresh, double time) : corner(c), rows(r), numberOfParticles(0), sizeThreshold(thresh), timeStep(time) {

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
   // l *= l;
    
    for (int x = max(0, p->cellX - 1); x <= min(rows - 1, p->cellX + 1); x++) {
        
        for (int y = max(0, p->cellY - 1); y <= min(rows - 1, p->cellY + 1); y++) {
            
            for (int z = max(0, p->cellZ - 1); z <= min(rows - 1, p->cellZ + 1); z++) {
                
                int cell = z + rows * (y + rows * x);
                
                Cell c = getCell(cell);
                
                if (c.particles.size() == 0) {
                    continue;
                }
                for (int i = 0; i < c.particlesCount; i++) {
                    shared_ptr<Particle> p2 = c.particles[i];

                    double norm = (p->pos - p2->pos).norm();
                    if (norm < l && norm > 0) {
                        p->neighbours.insert(p2);
                    }
                }
            }
        }
    }
}

void Grid::update(shared_ptr<Particle> p) {


    p->pos += p->speed * timeStep;
    

    if (p->pos.x < corner.x) {
        p->pos.x += 2 * (corner.x - p->pos.x);
        p->speed.x *= -1;
    }

    if (p->pos.y < corner.y) {
        p->pos.y += 2 * (corner.y - p->pos.y);
        p->speed.y *= -1;
    }

    if (p->pos.z < corner.z) {
        p->pos.z += 2 * (corner.z - p->pos.z);
        p->speed.z *= -1;
    }
    
    double size = sizeThreshold * rows;

    if (p->pos.x > corner.x + size) {
        p->pos.x += 2 * (corner.x + size - p->pos.x);
        p->speed.x *= -1;
    }

    if (p->pos.y > corner.y + size) {
        p->pos.y += 2 * (corner.y + size - p->pos.y);
        p->speed.y *= -1;
    }

    if (p->pos.z > corner.z + size) {
        p->pos.z += 2 * (corner.z + size - p->pos.z);
        p->speed.z *= -1;
    }

    
    
    Vect v = p->pos;
    int x = v.x / sizeThreshold;
    int y = v.y / sizeThreshold;
    int z = v.z / sizeThreshold;
    if (x < 0 || y < 0 || z < 0 || x >= rows || y >= rows || z >= rows) {
        Cell c = cells[p->cellZ + rows * (p->cellY + rows * p->cellX)];
        c.remove(p);
        cout << "The particle is not inside the grid. It won't be inserted." << endl;
        return;
    }
    if (x != p->cellX || y != p->cellY || z != p->cellZ) {
        Cell c = cells[p->cellZ + rows * (p->cellY + rows * p->cellX)];
        c.remove(p);
        cells[z + rows * (y + rows * x)].add(p);

        p->cellX = x;
        p->cellY = y;
        p->cellZ = z;
    }

}

shared_ptr<Particle> Grid::getParticle(int i) {
    if (i > numberOfParticles - 1) {
        cout << "Error: index out of bonds when querying particle";
        return 0;
    }

    return particles[i];
}


void parallelNeighbourSearch(Grid &g, vector<shared_ptr<Particle> > &particles, int &count,
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
            p->neighbours.clear();
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
}
