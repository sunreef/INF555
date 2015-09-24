//
// Created by victor on 24/09/15.
//

#ifndef INF555_PROJECT_POINT_H
#define INF555_PROJECT_POINT_H


struct Vect {
    double x, y, z;

    Vect(double x1, double y1, double z1);

    double norm();

    Vect operator+(Vect p2) {
        return Vect(x + p2.x, y + p2.y, z + p2.z);
    }

    void operator+=(Vect p2) {
        x += p2.x;
        y += p2.y;
        z += p2.z;
    }

    Vect operator-(Vect p2) {
        return Vect(x - p2.x, y - p2.y, z - p2.z);
    }

    void operator-=(Vect p2) {
        x -= p2.x;
        y -= p2.y;
        z -= p2.z;
    }

    Vect operator*(double l) {
        return Vect(x * l, y * l, z * l);
    }

    void operator*=(double l) {
        x *= l;
        y *= l;
        z *= l;
    }

    double operator*(Vect p2) {
        return x * p2.x + y * p2.y + z * p2.z;
    }

    Vect operator^(Vect p2) {
        return Vect(y * p2.z - z * p2.y, z * p2.x - x * p2.z, x * p2.y - y * p2.x);
    }
};


#endif //INF555_PROJECT_POINT_H
