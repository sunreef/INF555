#include <iostream>
#include <ctime>
#include "particle.h"
#include "grid.h"

using namespace std;

int main() {

    clock_t t = clock();
    Vect v(0, 0, 0);
    Grid g(0, v, 10);

    for(int i = 0; i < 1000; i++) {
        double x = (double) (rand() % 1000) / 200.0;
        double y = (double) (rand() % 100) / 200.0;
        double z = (double) (rand() % 100) / 200.0;

        Particle p(x,y,z,1,0.1);

        g.insert(p);
    }


    t = clock() - t;


    cout << (double) t / CLOCKS_PER_SEC;
    return 0;
}