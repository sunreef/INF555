//
//  hui.cpp
//  INF555___Project
//
//  Created by Galashov Alexandr on 08/11/2015.
//
//

#include "hui.hpp"

#include <vector>

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
#include <vtkCollection.h>

int hui()
{
    int N = 10000;
    
    int argc = 0;
    char *argv[] = {"ok","-real"};
    
    
    // Create the graphics structure. The renderer renders into the
    // render window.
    vtkSmartPointer<vtkRenderWindowInteractor> iren =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
    vtkSmartPointer<vtkRenderer> ren1 =
    vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkRenderWindow> renWin =
    vtkSmartPointer<vtkRenderWindow>::New();
    renWin->SetSize(800,800);
    renWin->SetMultiSamples(0);
    iren->SetRenderWindow(renWin);
    renWin->AddRenderer(ren1);
    
    
    vector< vtkNew<vtkSphereSource> > sphereSources;
    vector< vtkNew<vtkPolyDataMapper> > sphereMappers;
    vector< vtkNew<vtkActor> > spheres;
//    vtkCollection< vtkSphereSource> sphereSources;
    
    for(int i = 0 ; i < N; i++)
    {
        vtkNew<vtkSphereSource> sphereSource;
        sphereSource->SetCenter(rand()%100,rand()%100,rand()%100);
        sphereSource->SetRadius(1);
        sphereSource->Update();
        sphereSource->SetPhiResolution(1);
        sphereSource->SetThetaResolution(1);
        
        vtkNew<vtkPolyDataMapper> sphereMapper;
         sphereMapper->SetInputConnection( sphereSource->GetOutputPort());
        
        vtkNew<vtkActor> sphere;
        sphere->SetMapper(sphereMapper.Get());
        ren1->AddActor(sphere.Get());
        
       // sphereSources.push_back(sphereSource);
       // sphereMappers.push_back(sphereMapper);
      // spheres.push_back(sphere);
    }


    // Create an Animation Scene
    vtkSmartPointer<vtkAnimationScene> scene =
    vtkSmartPointer<vtkAnimationScene>::New();
    if(argc>=2 && strcmp(argv[1],"-real")==0)
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
    scene->SetFrameRate(5);
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
/*
    
    // Create an ActorAnimator for each actor;
    ActorAnimator animateSphere;
    animateSphere.SetActor(sphere);
    animateSphere.AddObserversToCue(cue1);
    
    
    */
    renWin->Render();
    ren1->ResetCamera();
    ren1->GetActiveCamera()->Dolly(.5);
    ren1->ResetCameraClippingRange();
    
    // Create Cue observer.
    scene->Play();
    scene->Stop();
    
    iren->Start();
    return EXIT_SUCCESS;
}