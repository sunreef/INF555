//
// Created by victor on 25/09/15.
//

#include "kernel.h"


Kernel::Kernel(double smoothingDistance) : h(smoothingDistance) {
}

double Kernel::operator()(double x) {

    double q = sqrt(x) / h;

    if (q >= 2) {
        return 0;
    }
    if (q >= 1) {
        return (3.0 / (12.0 * M_PI)) * pow((2.0 - q) / h, 3.0);
    }
    if (q >= 0) {
        return (3.0 / (2.0 * M_PI)) * (2.0 / 3.0 - q * q * (1 - 1.0 / 2.0 * q)) / (h * h * h);
    }
    std::cout << "Error: the value given to the kernel is not a positive value." << std::endl;
    return -1;
}

Vect Kernel::grad(Vect pi, Vect pj, double norm) {
    if(norm == 9999) {
        norm = (pi - pj).norm();
    }
    double q = sqrt(norm) / h;

    if(q == 0) {
        return Vect(0,0,0);
    }

    double temp;

    if (q >= 2) {
        return Vect(0, 0, 0);
    }
    if (q >= 1) {
        temp = 3.0 / M_PI * (-q + (3.0 / 4.0) * q * q);
    }
    if (q >= 0) {
        temp = -3.0 / (4.0 * M_PI) * pow(2 - q, 2);
    }

    return Vect((pi.x - pj.x), (pi.y - pj.y), (pi.z - pj.z)) *(temp / (q*h));
}
