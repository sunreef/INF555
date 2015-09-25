//
// Created by victor on 25/09/15.
//

#ifndef INF555_PROJECT_KERNEL_H
#define INF555_PROJECT_KERNEL_H

#include <math.h>
#include <iostream>

class Kernel {
private:
    double h;

public:
    Kernel(double smoothingDistance);

    double operator()(double x);

};


#endif //INF555_PROJECT_KERNEL_H
