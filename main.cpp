#include <iostream>
#include <ctime>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "grid.h"
#include "kernel.h"


using namespace std;
using namespace pcl;


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

//    cout << hr << endl;
//    cout << min << endl;
//    cout << sec << endl;
//    cout << milli << endl;

    int timeInMilli = (((hr * 60) + min) * 60 + sec) * 1000 + milli;
    return timeInMilli;
}

int main() {


    double kernelSmoothingLength = 0.2;

    clock_t tps = clock();

    Kernel w(kernelSmoothingLength);
    Vect v(0, 0, 0);
    Grid g(v, 10, 2 * kernelSmoothingLength);

    srand(tps);

    PointCloud<PointXYZRGB>::Ptr pc(new PointCloud<PointXYZRGB>(10, 10, PointXYZRGB(0, 255, 0)));

    for (int i = 0; i < 5000; i++) {
        double x = (double) (rand() % 500) / 200.0;
        double y = (double) (rand() % 200) / 200.0;
        double z = (double) (rand() % 500) / 200.0;

        shared_ptr<Particle> p = make_shared<Particle>(x, y, z, 1.0, 0.1);
        p->speed = Vect(0.1, 0.1, 0.1);
        g.insert(p);

        PointXYZRGB pt(255, 0, 0);
        pt.x = x;
        pt.y = y;
        pt.z = z;
        pc->push_back(pt);
    }


//    for (double x = 0; x < 10.0; x += 0.2) {
//        for (double y = 0; y < 10.0; y += 0.2) {
//            for (double z = 0; z < 10.0; z += 0.2) {
//                Particle p = Particle(x, y, z, 1, 0.05);
//                g.insert(p);
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

//    cv.addPointCloud(PointCloud<PointXYZRGB>::ConstPtr(pc));
//    cv.spinOnce(1);
//    for (int t = 0; t < 100; t++) {
//        g.computeNeighbours();
//        PointCloud<PointXYZRGB>::Ptr pc2(new PointCloud<PointXYZRGB>(10, 10, PointXYZRGB(0, 255, 0)));
//        for (int i = 0; i < g.particles.size(); i++) {
//            shared_ptr<Particle> pa = g.getParticle(i);
//            g.update(pa);
//            PointXYZRGB pt(255, 0, 0);
//            pt.x = pa->pos.x;
//            pt.y = pa->pos.y;
//            pt.z = pa->pos.z;
//            pc2->push_back(pt);
//        }
//        cv.removeAllPointClouds();
//        cv.addPointCloud(PointCloud<PointXYZRGB>::ConstPtr(pc2));
//        cv.spinOnce();
//        cout << "Time " << t << endl;
//    }



    int n = 0;

    g.computeNeighbours();

    for (int i = 0; i < g.getNumberOfParticles(); i++) {
        shared_ptr<Particle> p = g.getParticle(i);

        cout << p->neighbours.size() << endl;
        double rho = 0;
        for (shared_ptr<Particle> p2: p->neighbours) {
            rho += p2->w * w((p->pos - p2->pos).norm());
        }

        p->rho = rho;

        double pressure = 0.5 * (pow(rho, 7) - 1);
        p->pressure = pressure;
    }


    int b = timeInMilli();
    tps = clock() - tps;


    cout << "Machine time(biased by multithreading): " << (double) tps / CLOCKS_PER_SEC << endl;
    cout << "Real time: " << (double) (b - a) / 1000.0 << endl;
    return 0;
}
