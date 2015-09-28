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
