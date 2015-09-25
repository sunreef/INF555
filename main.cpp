#include <iostream>
#include <ctime>
#include "particle.h"
#include "grid.h"
#include "kernel.h"

using namespace std;


int main() {

    clock_t t = clock();
    Vect v(0, 0, 0);
    Grid g(v, 10);

    srand(0);

    for (int i = 0; i < 100000; i++) {
        double x = (double) (rand() % 1000) / 200.0;
        double y = (double) (rand() % 1000) / 200.0;
        double z = (double) (rand() % 1000) / 200.0;

        Particle p(x, y, z, 1, 0.1);
        g.insert(p);
    }
    cout << g.getNumberOfParticles() << endl;

    cout << g.getCell(0, 0, 0).particlesCount << endl;

    Kernel w(0.4);
    cout << w(0.0) << endl;

    t = clock() - t;


    cout << (double) t / CLOCKS_PER_SEC;
    return 0;
}