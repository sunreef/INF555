#include <iostream>
#include <ctime>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "grid.h"
#include "kernel.h"
#include "vizualisation.hpp"
#include "hui.hpp"
#include "physique.hpp"
#include "supp_functions.hpp"


#include "AnimateActors.h"
#include <vtkSmartPointer.h>
#include <vtkAnimationCue.h>
#include <vtkRenderer.h>
#include <vtkSphereSource.h>
#include <vtkConeSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkCommand.h>
#include <vtkAnimationScene.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkCamera.h>
#include <vtkNew.h>


using namespace std;
using namespace pcl;

double viscosity = 0.00001;
double rhoInitial = 1.0;

//double stiffness = 0.000001;
double stiffness = 10;

const int number_cells = 20;
const double path = 1.0 / (double)number_cells;

const  double kernelSmoothingLength = path; //0.1;
const double lambda = 0.4;

const double epsilon = 0.5;
const double alpha = 0.5;

//double timeStep = lambda * kernelSmoothingLength / (100.0);
double timeStep = 0.0001;

#define PI 3.14


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


Vect computeViscosityForce(shared_ptr<Particle> p, Kernel &w, double viscosity) {
    Vect result(0, 0, 0);
    
    for (shared_ptr<Particle> n: p->neighbours) {
        // masse / rho
        double temp = n->w / n->rho;
        // xij
        Vect xij = p->pos - n->pos;
        double norm = xij.norm();
        temp *= xij * w.grad(p->pos, n->pos, norm);
        temp /= norm + 0.01 * w.getSmoothingDistance() * w.getSmoothingDistance();
        result += (p->speed - n->speed) * temp;
    }
    // you forgot 2
    result *= 2;
    
    result *= viscosity;
    
    
    return result;
}

double computeNewRho(shared_ptr<Particle> p, Kernel &w, double timeStep) {
    double sum1 = 0;
    double sum2 = 0;
    
    for (shared_ptr<Particle> n: p->neighbours) {
        double norm = (n->pos - p->pos).norm();
        sum1 += n->w * w(norm);
        sum2 += (p->speed - n->speed) * w.grad(p->pos, n->pos, norm);
    }
    return sum1 + timeStep * sum2;
}

Vect computePressureForce(shared_ptr<Particle> p, Kernel &w) {
    Vect result(0, 0, 0);
    
    
    
    
    for (shared_ptr<Particle> n: p->neighbours) {
        result += w.grad(p->pos, n->pos) * ((p->pressure / (p->rho * p->rho) + n->pressure / (n->rho * n->rho)) * n->w);
    }
    
    
    // there is a minus here, so
    //result *=-1;
    
    return result;
}

Vect artificialViscosity(shared_ptr<Particle> p, Kernel &w)
{
    Vect result(0,0,0);
    
    for(shared_ptr<Particle> q : p->neighbours)
    {
        double norm = (q->pos - p->pos).norm();
        Vect temp = (q->speed - p->speed);
        temp *= w(norm) * q->w / q->rho;
        result+= temp;
    }
    
    return result;
}


const int scene_mode = 0;
const int number_particles = 500;
const int window_size = 800;

// ghost particles
// * air particles around liquid
// * solid paricles : correct the density summation,


// surface sampling:
// surface -> interior volume -> volume relaxation


/*
class Face
{
private:
    double x0,y0,z0;
    double dx,dy,dz;
    Vect direction;
public:
    Face(double x0, double y0, double z0, double dx, double dy, double dz, Vect direction)
    {
        this->x0 = x0;
        this->y0 = y0;
        this->z0 = z0;
        this->dx = dx;
        this->dy = dy;
        this->dz = dz;
        this->direction = direction;
    }
    
};

class WallBoundary
{
    Face bottom,right,left,front,back;
    
    WallBoundary(double wallHeight,double wallWidth, double wallDepth, )
    {

    }
    
    int inside(shared_ptr<Particle> p)
    {
        
    }
    
    double distance(Particle p)
    {
        
    }

};


*/

int main() {
    
    //hui();

    
    
    // for visualization
    
    vtkSmartPointer<vtkRenderWindowInteractor> iren =
        vtkSmartPointer<vtkRenderWindowInteractor>::New();
    vtkSmartPointer<vtkRenderer> ren1 =
        vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkRenderWindow> renWin =
        vtkSmartPointer<vtkRenderWindow>::New();
    renWin->SetSize(window_size,window_size);
    renWin->SetMultiSamples(0);
    iren->SetRenderWindow(renWin);
    renWin->AddRenderer(ren1);
    

    
    // Create an Animation Scene
    vtkSmartPointer<vtkAnimationScene> scene =
    vtkSmartPointer<vtkAnimationScene>::New();
    if(1 == scene_mode)
    {
        cout << "real-time mode" << endl;
        scene->SetModeToRealTime();
    }
    else
    {
        cout << "sequence mode" << endl;
        scene->SetModeToSequence();
    }

    scene->SetLoop(0);
    scene->SetFrameRate(1);
    scene->SetStartTime(0);
    scene->SetEndTime(1);
    
    vtkSmartPointer<AnimationSceneObserver> sceneObserver =
    vtkSmartPointer<AnimationSceneObserver>::New();
    sceneObserver->SetRenderWindow(renWin);
    scene->AddObserver(vtkCommand::AnimationCueTickEvent,sceneObserver);
    
    // Create an Animation Cue for each actor
    vtkSmartPointer<vtkAnimationCue> cue1 =
        vtkSmartPointer<vtkAnimationCue>::New();
    cue1->SetStartTime(0);
    cue1->SetEndTime(1);
    
    scene->AddCue(cue1);
    
    
//    vector< vtkNew<vtkSphereSource> > particle_sources;
 //   vector< vtkNew<vtkPolyDataMapper> > particle_mappers;
 //   vector< vtkNew<vtkActor> > spheres;
    
    vector< vtkSmartPointer<vtkSphereSource>> particle_sources;
    vector< vtkSmartPointer<vtkPolyDataMapper>> particle_mappers;
    vector< vtkSmartPointer<vtkActor>> spheres;
    vector< ActorAnimator* > actor_animators;
    
    /*
    
    for(int i = 0 ; i < number_particles; i++)
    {

      
    }
    


    
    */
    

    // Assumption : Particle Coordinates are from 0 to 1
    // coordinates : 0 -- 1
    


    clock_t tps = clock();

    // has to be 2 times less then cell size
    Kernel w(kernelSmoothingLength);
    Vect v(0, 0, 0);
    Grid g(v, 2 * number_cells, 2 * kernelSmoothingLength, timeStep);
    
    srand(tps);
    
    // rayon of particle
    double particle_size = path / 10.0; // has to be less then kernelSmoothingLength
    double volume_cell = pow( (2 * kernelSmoothingLength), 3);
    
    double mass_particle = volume_cell * rhoInitial;
    
    //double volumeParticle =
    
    vector< Particle> particles;
    
    for(int i = 0 ; i < number_particles; i++)
    {
        
        // x,x are from 0.2 to 0.8
        // y is from 0 to 0.5
        double x_phys = 0.3 + (double) ( rand() % 50 ) / 100;
        double y_phys = 0.3 + (double) ( rand() % 100 ) / 1000;
        double z_phys = 0.3 + (double) ( rand() % 50 ) / 100;
        
        shared_ptr<Particle> p = make_shared<Particle>(x_phys, y_phys, z_phys, mass_particle, particle_size);
        
    
        p->speed = Vect(0, 0, 0);
        p->rho = rhoInitial;
        g.insert(p);
        
        double x_graph = x_phys * window_size - (double) window_size / 2.0;
        double y_graph = y_phys * window_size - (double) window_size / 2.0;
        double z_graph = z_phys * window_size - (double) window_size / 2.0;
        
        
        
        particle_sources.push_back(vtkSmartPointer<vtkSphereSource>::New());
        particle_mappers.push_back(vtkSmartPointer<vtkPolyDataMapper>::New());
        spheres.push_back(vtkSmartPointer<vtkActor>::New());
        
        particle_sources[i]->SetCenter(x_graph,y_graph,z_graph);
        particle_sources[i]->SetRadius(20);
        particle_sources[i]->Update();
        particle_sources[i]->SetPhiResolution(50);
        particle_sources[i]->SetThetaResolution(50);
        
        particle_mappers[i]->SetInputConnection(particle_sources[i]->GetOutputPort());
        
        spheres[i]->SetMapper(particle_mappers[i]);
        ren1->AddActor(spheres[i]);
        actor_animators.push_back(new ActorAnimator(p));
        actor_animators[i]->SetActor(spheres[i]);
        actor_animators[i]->AddObserversToCue(cue1);
    }
    

    /*
    
    tps = clock();
    int a = timeInMilli();
    
//    visualization::PCLVisualizer cv("Cloud");


//    cv.addPointCloud(PointCloud<PointXYZRGB>::ConstPtr(pc));
 //   cv.spinOnce(1);
    
    
    
    while(true)
    {
        for (int t = 0; t < 100; t++) {
            g.computeNeighbours();

       
        // ok
        Vect gravity = Vect(0, -9.8, 0);
        for (shared_ptr<Particle> p : g.particles) {
            
            Vect visco = computeViscosityForce(p, w);
            
            p->speed += (visco + gravity) * (timeStep);
            
            
        }
            
        
        // ok
        for (shared_ptr<Particle> p : g.particles) {
            p->rho = computeNewRho(p, w);
            cout << p->rho << "  -  " << p->neighbours.size() << endl;
            p->pressure = stiffness * (pow(p->rho / rhoInitial, 7) - 1);
        }
            
        // ok

        for (shared_ptr<Particle> p: g.particles) {
            //ok
            Vect pressureForce = computePressureForce(p, w);
          //  cout << pressureForce.norm() << "  -  " << p->neighbours.size() << endl;
            
            // ok
            p->speed += pressureForce * timeStep;
        }
        
        

        PointCloud<PointXYZRGB>::Ptr pc2(new PointCloud<PointXYZRGB>(10, 10, PointXYZRGB(0, 255, 0)));
        for (int i = 0; i < g.particles.size(); i++) {
            shared_ptr<Particle> pa = g.getParticle(i);
           // cout << pa->speed.x << endl;
            g.update(pa);
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
    }
    
     */
    // ANIMATION
    int k = 2000;
    
    renWin->Render();
    ren1->ResetCamera();
    ren1->GetActiveCamera()->Dolly(0.5);
    ren1->GetActiveCamera()->SetObliqueAngles(45,95);
    ren1->ResetCameraClippingRange();
    // Create Cue observer.
    scene->Play();
    scene->Stop();
    
    
    while(true)
    {
        
       
        g.computeNeighbours();
            
            
        // ok
            
        Vect gravity = Vect(0, -9.8, 0);
        for (shared_ptr<Particle> p : g.particles) {
            
            if( p->neighbours.size() != 0)
            {
                Vect visco = computeViscosityForce(p, w,viscosity);
                
                p->speed += (visco + gravity) * (timeStep);
            }
                
            
                
                
        }
            
            
        // ok
        for (shared_ptr<Particle> p : g.particles) {
            if(p->neighbours.size() != 0)

            {
                p->rho = computeNewRho(p, w,timeStep);
                //cout << p->rho << "  -  " << p->neighbours.size() << endl;
                p->pressure = stiffness * (pow(p->rho / rhoInitial, 7) - 1);
            }
        }
            
        // ok
            
        for (shared_ptr<Particle> p: g.particles) {
            
            if(p->neighbours.size() != 0)
            {
                //ok
                Vect pressureForce = computePressureForce(p, w);
                //  cout << pressureForce.norm() << "  -  " << p->neighbours.size() << endl;
                
                // ok
                cout << "pressure"<< endl;
                cout << pressureForce.x << " " <<pressureForce.y << " "<<pressureForce.z<<endl;
                p->speed -= pressureForce * timeStep;
                cout << "final speed" << endl;
                cout<< p->speed.x << " " << p->speed.y << " " << p->speed.z << endl;
                
            }
           
        }
        
        
        // XSPH Artificial Viscosity
        
        for(shared_ptr<Particle> p: g.particles)
        {
            if(p->neighbours.size() != 0)
            {
                Vect artificialViscosityTerm = artificialViscosity(p, w);
                
                artificialViscosityTerm *= epsilon;
                p->speed += artificialViscosityTerm;
            }
        }
        
            
            
            
        
        for (int i = 0; i < g.particles.size(); i++) {
            shared_ptr<Particle> pa = g.getParticle(i);
            
            
       //     cout << "before "<<endl;
       //     cout << "phys" << endl;
         //   cout << pa->pos.x << " " <<pa->pos.y << " "<< pa->pos.z << endl;

            
            
            // cout << pa->speed.x << endl;
            vector<double> temp_position(3);
            temp_position[0] = (double)phys_to_graph(window_size, g.getParticle(i)->pos.x);
            temp_position[1] = (double)phys_to_graph(window_size, g.getParticle(i)->pos.y);
            temp_position[2] = (double)phys_to_graph(window_size, g.getParticle(i)->pos.z);
            
        //    cout << "graph"<< endl;
        //    cout << temp_position[0] << " " << temp_position[1] << " " << temp_position[2] <<endl;
            
            actor_animators[i]->SetStartPosition(temp_position);
            
            
            g.update(pa);
            temp_position[0] = (double)phys_to_graph(window_size, g.getParticle(i)->pos.x);
            temp_position[1] = (double)phys_to_graph(window_size, g.getParticle(i)->pos.y);
            temp_position[2] = (double)phys_to_graph(window_size, g.getParticle(i)->pos.z);
            
            pa = g.getParticle(i);
         //   cout << "after"<<endl;
        //    cout << "phys" << endl;
         //   cout << pa->pos.x << " " <<pa->pos.y << " "<< pa->pos.z << endl;
        //    cout << "graph"<< endl;
         //   cout << temp_position[0] << " " << temp_position[1] << " " << temp_position[2] <<endl;
            
            actor_animators[i]->SetEndPosition(temp_position);
        //    PointXYZRGB pt(255, 0, 0);
     //       pt.x = pa->pos.x;
     //       pt.y = pa->pos.y;
     //       pt.z = pa->pos.z;
   //         pc2->push_back(pt);
        }
            

    
    
    
        
        
    
    // Create Cue observer.
        scene->Play();
        scene->Stop();
        if(k == 0)
            break;
        
        k++;
        
        cout << k << endl;
    }
    
  //  iren->Start();
    
    /*

    int b = timeInMilli();
    tps = clock() - tps;


    cout << "Machine time(biased by multithreading): " << (double) tps / CLOCKS_PER_SEC << endl;
    cout << "Real time: " << (double) (b - a) / 1000.0 << endl;
    return 0;
     
    
    
    
    */
    
    return 0;
 
 
}
