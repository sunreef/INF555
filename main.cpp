#include <iostream>
#include <ctime>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "grid.h"
#include "kernel.h"


using namespace std;
using namespace pcl;

double viscosity = 0.000001;
double rhoInitial = 1.0;
double timeStep = 0.02;
double stiffness = 0.1;
int numberOfParticles = 4000;
double kernelSmoothingLength = 0.1;

int timeInMilli() {
    timeval t;
    gettimeofday(&t, NULL);

    string buf(20, '\0');
    strftime(&buf[0], buf.size(), "%H:%M:%S:", localtime(&t.tv_sec));
    string str_hr = buf.substr(0, 2);
    string str_min = buf.substr(3, 2);
    string str_sec = buf.substr(6, 2);

    int hr = atoi(str_hr.c_str());
    int min = atoi(str_min.c_str());
    int sec = atoi(str_sec.c_str());
    int milli = t.tv_usec / 1000;

    int timeInMilli = (((hr * 60) + min) * 60 + sec) * 1000 + milli;
    return timeInMilli;
}


Vect computeViscosityForce(shared_ptr<Particle> p, Kernel &w) {
    Vect result(0, 0, 0);
    double d = 0.01 * w.getSmoothingDistance() * w.getSmoothingDistance();

    for (shared_ptr<Particle> n: p->neighbours) {
        double temp = 1.0 / n->rho;
        Vect xij = p->pos - n->pos;
        double norm = xij.norm();
        temp *= xij * w.grad(p->pos, n->pos, norm);
        temp /= norm + d;

        result += (p->speed - n->speed) * temp;
    }
    result *= viscosity;


    return result;
}

double computeNewRho(shared_ptr<Particle> p, Kernel &w) {
    double sum1 = 0;
    double sum2 = 0;

    for (shared_ptr<Particle> n: p->neighbours) {
        double norm = (n->pos - p->pos).norm();
        sum1 += n->w * w(norm);
        sum2 += ((p->speed - n->speed) * w.grad(p->pos, n->pos, norm));
    }
    if(sum1 + timeStep * sum2 < 0.1) {
        cout << "Small rho" << endl;
    }

    return sum1 + timeStep * sum2;
}

Vect computePressureForce(shared_ptr<Particle> p, Kernel &w) {
    Vect result(0, 0, 0);

    for (shared_ptr<Particle> n: p->neighbours) {
        result += w.grad(p->pos, n->pos) * ((p->pressure / (p->rho * p->rho) + n->pressure / (n->rho * n->rho)) * n->w);
    }
    return result*1000;
}


int main() {

    clock_t tps = clock();

    Kernel w(kernelSmoothingLength);
    Vect v(0, 0, 0);
    Grid g(v, 10, 2 * kernelSmoothingLength, timeStep);

    srand(tps);

    PointCloud<PointXYZRGB>::Ptr pc(new PointCloud<PointXYZRGB>(10, 10, PointXYZRGB(0, 255, 0)));

    double mass = pow(kernelSmoothingLength, 3) * rhoInitial;
    for (int i = 0; i < numberOfParticles; i++) {
        double x = (double) (rand() % 200) / 200.0;
        double y = (double) (rand() % 100) / 200.0;
        double z = (double) (rand() % 200) / 200.0;

        shared_ptr<Particle> p = make_shared<Particle>(x, y, z, mass, 0.1);
        p->speed = Vect(0, 0, 0);
        p->rho = rhoInitial;
        g.insert(p);

        PointXYZRGB pt(255, 0, 0);
        pt.x = x;
        pt.y = y;
        pt.z = z;
        pc->push_back(pt);
    }


//    for (double x = 0; x < 4.0; x += 0.08) {
//        for (double y = 0; y < 4.0; y += 0.08) {
//            for (double z = 0; z < 4.0; z += 0.08) {
//                shared_ptr<Particle> p = make_shared<Particle>(x, y, z, 1, 0.05);
//                g.insert(p);
//                p->rho = rhoInitial;
//                p->speed = Vect(0,0,0);
//                PointXYZRGB pt(0, 255, 0);
//                pt.x = x;
//                pt.y = y;
//                pt.z = z;
//                pc->push_back(pt);
//            }
//        }
//    }

    tps = clock();
    int a = timeInMilli();
    visualization::PCLVisualizer cv("Cloud");


    cv.addPointCloud(PointCloud<PointXYZRGB>::ConstPtr(pc));
    cv.spinOnce(1);
    for (int t = 0; t < 100; t++) {
        g.computeNeighbours();

        Vect gravity = Vect(0, -9.8, 0);
        for (shared_ptr<Particle> p : g.particles) {
            Vect visco = computeViscosityForce(p, w);
            p->speed += (visco + gravity) * (timeStep);
        }

        for (shared_ptr<Particle> p : g.particles) {
            p->rho = computeNewRho(p, w);
//            cout << p->rho << endl;
            p->pressure = stiffness * (p->rho - rhoInitial);
        }

        for (shared_ptr<Particle> p: g.particles) {
            Vect pressureForce = computePressureForce(p, w);
//            cout << pressureForce.norm() << "  -  " << p->neighbours.size() << endl;
            p->speed -= pressureForce * timeStep;
        }

        g.update();

        PointCloud<PointXYZRGB>::Ptr pc2(new PointCloud<PointXYZRGB>(10, 10, PointXYZRGB(0, 255, 0)));
        for (int i = 0; i < g.particles.size(); i++) {
            shared_ptr<Particle> pa = g.getParticle(i);
            PointXYZRGB pt(255, 0, 0);
            pt.x = pa->pos.x;
            pt.y = pa->pos.y;
            pt.z = pa->pos.z;
            pc2->push_back(pt);
        }
        cv.removeAllPointClouds();
        cv.addPointCloud(PointCloud<PointXYZRGB>::ConstPtr(pc2));
        cout << "Time " << t << endl;
        cv.spinOnce();
    }

    int b = timeInMilli();
    tps = clock() - tps;


    cout << "Machine time(biased by multithreading): " << (double) tps / CLOCKS_PER_SEC << endl;
    cout << "Real time: " << (double) (b - a) / 1000.0 << endl;
    return 0;
}
