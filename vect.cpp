//
// Created by victor on 24/09/15.
//

#include "vect.h"

Vect::Vect(double x1, double y1, double z1) : x(x1), y(y1), z(z1) {
}

double Vect::norm() {
    return *this * *this;
}

