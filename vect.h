//
// Created by victor on 24/09/15.
//

#ifndef INF555_PROJECT_POINT_H
#define INF555_PROJECT_POINT_H


struct Vect {
    double x, y, z;

    Vect(double x1, double y1, double z1);
    Vect();

    double norm();

    Vect operator+(Vect p2) const {
        return Vect(x + p2.x, y + p2.y, z + p2.z);
    }

    void operator+=(Vect p2) {
        x += p2.x;
        y += p2.y;
        z += p2.z;
    }

    Vect operator-(Vect p2) const {
        return Vect(x - p2.x, y - p2.y, z - p2.z);
    }

    void operator-=(Vect p2) {
        x -= p2.x;
        y -= p2.y;
        z -= p2.z;
    }

    Vect operator*(double l) const {
        return Vect(x * l, y * l, z * l);
    }

    void operator*=(double l) {
        x *= l;
        y *= l;
        z *= l;
    }

    double operator*(Vect p2) const {
        return x * p2.x + y * p2.y + z * p2.z;
    }

    Vect operator^(Vect p2) const {
        return Vect(y * p2.z - z * p2.y, z * p2.x - x * p2.z, x * p2.y - y * p2.x);
    }

    double& operator[](int i) {
        if(i == 0) {
            return x;
        }
        if(i == 1) {
            return y;
        }
        if(i == 2) {
            return z;
        }
    }
};


#endif //INF555_PROJECT_POINT_H
