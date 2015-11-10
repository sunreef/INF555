//
//  vizualisation.cpp
//  INF555___Project
//
//  Created by Galashov Alexandr on 07/11/2015.
//
//

#include "vizualisation.hpp"

#include <vector>
#include <ctime>

using namespace std;


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
#include <vtkActor.h>
#include <vtkArrayCalculator.h>
#include <vtkCamera.h>
#include <vtkClipDataSet.h>
#include <vtkCutter.h>
#include <vtkDataSetMapper.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkLookupTable.h>
#include <vtkNew.h>
#include <vtkPlane.h>
#include <vtkPointData.h>
#include <vtkPointSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRibbonFilter.h>
#include <vtkStreamTracer.h>
#include <vtkSmartPointer.h>
#include <vtkUnstructuredGrid.h>
#include <vtkXMLUnstructuredGridReader.h>


/*
class ParticleVizualisation
{
private:
    
    
    // data parametres
    int N = 1;
    vector< vtkNew<vtkSphereSource> > sphereSources; // here information physical information of particles
    vector< vtkNew<vtkPolyDataMapper> > sphereMappers;
    vector< vtkNew<vtkActor> > sphereActors;
    
    // window Parametres
    int window_width = 600;
    int window_height = 600;
    
    // render parametres
    vtkSmartPointer<vtkRenderWindowInteractor> renWinInteractor;
    vtkSmartPointer<vtkRenderer> renderer;
    vtkSmartPointer<vtkRenderWindow> renWin;
    vtkSmartPointer<vtkAnimationScene> scene;
    vtkSmartPointer<AnimationSceneObserver> sceneObserver;
    vtkSmartPointer<vtkAnimationCue> animationCue;
    
    // animation parametres (technique)
    int multiSamples = 0;
    int frameRate = 5;
    int sceneStartTime = 0;
    int sceneEndTime = 20;
    int animStartTime = 5;
    int animEndTime = 23;
    vector<ActorAnimator> animateSpheres;
    
    
    const int mode = 0;
    
    
    void setup()
    {
        this->renWinInteractor =
            vtkSmartPointer<vtkRenderWindowInteractor>::New();
        
        this->renderer =
            vtkSmartPointer<vtkRenderer>::New();
        this->renWin =
            vtkSmartPointer<vtkRenderWindow>::New();
        
        
        
        renWin->SetMultiSamples(multiSamples);
        renWin->SetSize(window_width,window_height);
        
        renWinInteractor->SetRenderWindow(renWin);
        renWin->AddRenderer(renderer);
        
        scene =
            vtkSmartPointer<vtkAnimationScene>::New();
        
        if(mode == 1)
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
        scene->SetFrameRate(frameRate);
        scene->SetStartTime(sceneStartTime);
        scene->SetEndTime(sceneEndTime);
        
        
        sceneObserver =
            vtkSmartPointer<AnimationSceneObserver>::New();
        
        sceneObserver->SetRenderWindow(renWin);
        scene->AddObserver(vtkCommand::AnimationCueTickEvent,sceneObserver);
        
        animationCue =
            vtkSmartPointer<vtkAnimationCue>::New();
        
        animationCue->SetStartTime(animStartTime);
        animationCue->SetEndTime(animEndTime);
        scene->AddCue(animationCue);


    }
    
    
    
    
public:
    ParticleVizualisation()
    {
//        vtkNew<vtkSphereSource> sp;
//        sphereSources.push_back(sp);
//        sphereMappers.clear();
  //      sphereActors.clear();
      
        
        setup();

    }
    
    ParticleVizualisation(int window_width, int window_height)
    {
        this->window_height = window_height;
        this->window_width = window_width;
        setup();
    }
    

    
    void addParticle(vtkSmartPointer<vtkSphereSource> *sphereSource, vtkSmartPointer<vtkPolyDataMapper> *sphereMapper, vtkSmartPointer<vtkActor> *sphereActor)
    {
        N++;
        
        this->sphereSources.push_back((*sphereSource));
        this->sphereMappers.push_back((*sphereMapper));
        this->sphereActors.push_back((*sphereActor));
    }
    
    void deleteParticle(int i )
    {
        if( i < 0 && i > N )
        {
            throw 1;
            //  return NULL;
        }
        else{
            N--;
           this->sphereActors.erase(i);
            this->sphereMappers.erase(i);
            this->sphereActors.erase(i);
        }
    }
    

    vtkNew<vtkSphereSource>& getSource(int i)
    {
        if( i < 0 && i > N )
        {
            throw 1;
          //  return NULL;
        }
        else{
            return (this->sphereSources)[i];
        }
    }
    
    vtkNew<vtkPolyDataMapper>& getMapper(int i)
    {
        if( i < 0 && i > N )
        {
            throw 1;
            //return NULL;
        }
        else{
            return (this->sphereMappers)[i];
        }
    }
    
    vtkNew<vtkActor>& getActor(int i)
    {
        if( i < 0 && i > N )
        {
            throw 1;
            //return NULL;
        }
        else{
            return (this->sphereActors)[i];
        }
    }
    
    
    void generate_particles(int N, double a, double b, double c, double r)
    {
        
        //ParticleVizualisation pV;
        
        for(int i = 0 ; i < N; i++)
        {
            // Generate a sphere
            vtkNew<vtkSphereSource> sphereSource;
            
            
            sphereSource->SetRadius(r);
            
            double x = (double) (rand() % ((int ) (a - 2*r) )) + r ;
            double y = (double) (rand() % ((int ) (b - 2*r) )) + r;
            double z = (double) (rand() % ((int ) (c - 2*r) )) + r;
            
            sphereSource->SetCenter(x,y,z);
            sphereSource->Update();
            
            vtkNew<vtkPolyDataMapper> sphereMapper;
            
            sphereMapper->SetInputConnection( sphereSource->GetOutputPort());
            
            vtkNew<vtkActor> sphere;
            
            sphere->SetMapper(sphereMapper.Get());
            
            renderer->AddActor(sphere.Get());
            
//            this->addParticle(&sphereSource,&sphereMapper,&sphere);
            
            
            
            this->sphereSources.push_back(sphereSource);
            this->sphereMappers.push_back(sphereMapper);
            this->sphereActors.push_back(sphere);
            
            ActorAnimator animSphere;
            animSphere.SetActor(sphere.Get());
            animSphere.AddObserversToCue(this->animationCue);
            
            this->animateSpheres.push_back(animSphere);
            
            
        }
        
        
        //return pV;
        
        
        
    }
    
    void visualize()
    {
        this->renWin->Render();
        
        this->renderer->ResetCamera();
        this->renderer->GetActiveCamera()->Dolly(.5);
        this->renderer->ResetCameraClippingRange();
        
        this->scene->Play();
        this->scene->Stop();
        
        this->renWinInteractor->Start();
    }
};



void test()
{
    int argc = 0;
    char *argv[] = {"ok","-real"};

    
    ParticleVizualisation pV;
    pV.generate_particles(5,300,300,300,15);

    pV.visualize();
    
}
*/