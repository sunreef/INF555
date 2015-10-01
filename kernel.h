//
// Created by victor on 25/09/15.
//

#ifndef INF555_PROJECT_KERNEL_H
#define INF555_PROJECT_KERNEL_H

#include <math.h>
#include <iostream>
#include "vect.h"

class Kernel {
private:
    double h;

public:
    Kernel(double smoothingDistance);

    double operator()(double x);
    Vect grad(Vect pi, Vect pj);

};


#endif //INF555_PROJECT_KERNEL_H
