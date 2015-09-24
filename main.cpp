#include <iostream>
#include <ctime>
#include <thread>
#include "particle.h"
#include "grid.h"

using namespace std;

void insertRandomParticle(Grid &g, int number) {
    for (int i = 0; i < number; i++) {
        double x = (double) (rand() % 1000) / 200.0;
        double y = (double) (rand() % 100) / 200.0;
        double z = (double) (rand() % 100) / 200.0;

        Particle p(x, y, z, 1, 0.1);
        g.insert(p);
    }
}

int main() {

    clock_t t = clock();
    Vect v(0, 0, 0);
    Grid g(0, v, 10);


    vector<thread> threads;
    for (int j = 0; j < 8; j++) {
        threads.push_back(thread([&] { insertRandomParticle(g, 1000); }));
    }
    for (int j = 0; j < 8; j++) {
        threads[j].join();
    }

    t = clock() - t;


    cout << (double) t / CLOCKS_PER_SEC;
    return 0;
}