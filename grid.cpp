//
// Created by victor on 24/09/15.
//

#include "grid.h"

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
        cells[z + rows * (y + rows * x)].add(p);
        p.cellX = x;
        p.cellY = y;
        p.cellZ = z;
        numberOfParticles++;
        return true;
    }
}

Cell Grid::getCell(int x, int y, int z) {
    return cells[z + rows * (y + rows * x)];
}

int Grid::getNumberOfParticles() {
    return numberOfParticles;
}

void Grid::neighbours(shared_ptr<Particle> p, double l, vector<shared_ptr<Particle>> result) {
    for (int x = max(0, p->cellX - 1); x <= min(rows - 1, p->cellX + 1); x++) {
        for (int y = max(0, p->cellY - 1); y <= min(rows - 1, p->cellY + 1); y++) {
            for (int z = max(0, p->cellZ - 1); z <= min(rows - 1, p->cellZ + 1); z++) {
                Cell c = getCell(x, y, z);
                for (int i = 0; i < c.particlesCount; i++) {
                    shared_ptr<Particle> p2 = c.particles[i];
                    double norm = (p->pos - p2->pos).norm();
                    if (norm < l * l && norm > 0) {
                        result.push_back(p2);
                    }
                }
            }
        }
    }
}
