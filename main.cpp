#include <iostream>
#include <ctime>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "grid.h"
#include "kernel.h"


using namespace std;
using namespace pcl;

int main() {

    clock_t t = clock();
    Vect v(0, 0, 0);
    Grid g(v, 10);

    srand(t);

    PointCloud<PointXYZRGB>::Ptr pc(new PointCloud<PointXYZRGB>(10, 10, PointXYZRGB(0, 255, 0)));

    for (int i = 0; i < 50000; i++) {
        double x = (double) (rand() % 1000) / 200.0;
        double y = (double) (rand() % 1000) / 200.0;
        double z = (double) (rand() % 1000) / 200.0;

        Particle p(x, y, z, 1.0, 0.1);
        g.insert(p);

        PointXYZRGB pt(0, 255, 0);
        pt.x = x;
        pt.y = y;
        pt.z = z;

        pc->push_back(pt);
    }

    visualization::PCLVisualizer cv("Cloud");

    cv.addPointCloud(PointCloud<PointXYZRGB>::ConstPtr(pc));

    cv.spin();


    t = clock();

//    for (double x = 0; x < 5.0; x += 0.2) {
//        for (double y = 0; y < 5.0; y += 0.2) {
//            for (double z = 0; z < 5.0; z += 0.2) {
//                Particle p = Particle(x, y, z, 1, 0.05);
//                g.insert(p);
//            }
//        }
//    }

    Kernel w(0.1);
    for (int x = 0; x < g.rows; x++) {
        for (int y = 0; y < g.rows; y++) {
            for (int z = 0; z < g.rows; z++) {
                Cell c = g.getCell(x, y, z);

                for (shared_ptr<Particle> p: c.particles) {

                    g.neighbours(p, 0.2);


                    double rho = 0;
                    for (shared_ptr<Particle> p2: p->neighbours) {
                        rho += p2->w * w((p->pos - p2->pos).norm());
                    }

                    p->rho = rho;

                    double pressure = 0.5 * (pow(rho, 7) - 1);
                    p->pressure = pressure;
                }
            }
        }
    }

    for (int x = 0; x < g.rows; x++) {
        for (int y = 0; y < g.rows; y++) {
            for (int z = 0; z < g.rows; z++) {
                Cell c = g.getCell(x, y, z);
                for (shared_ptr<Particle> p: c.particles) {

                    double gradP = 0;


                }
            }
        }
    }


    t = clock() - t;


    cout << (double) t / CLOCKS_PER_SEC;
    return 0;
}
