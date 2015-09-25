//
// Created by victor on 24/09/15.
//

#include "grid.h"

Grid::Grid(Vect c, double s) : corner(c), size(s), numberOfParticles(0) {
    rows = (int) size / sizeThreshold;

    cells = new Cell[rows * rows * rows];

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
    delete cells;
}

void Grid::insert(Particle &p) {
    Vect v = p.pos - corner;

    int x = v.x / sizeThreshold;
    int y = v.y / sizeThreshold;
    int z = v.z / sizeThreshold;

    if (x < 0 || y < 0 || z < 0 || x >= rows || y >= rows || z >= rows) {
        cout << "The particle is not inside the grid. It won't be inserted." << endl;
        return;
    }
    else {
        cells[z + rows * (y + rows * x)].add(p);
        numberOfParticles++;
    }

}

int Grid::getNumberOfParticles() {
    return numberOfParticles;
}

Cell Grid::getCell(int x, int y, int z) {
    return cells[z + rows * (y + rows * x)];
}
