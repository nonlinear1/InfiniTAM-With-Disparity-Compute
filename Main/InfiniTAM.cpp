// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "InfiniTAM.h"


ComputeDisparity *computeDisparity;
bool g_WaitComputeDisparity = false;

//************************************************************//
//  File disparity  color: close-white-max  far-black-min(0)  //
//  Infinitam depth color: colse-green      far-red           //
//  Feed            image: RGB or Gray      depth:depth image //
//************************************************************//


int main(int argc, char** argv)
{
    glutInit(&argc, argv);
    printf("Initialising ...\n");
    computeDisparity = new ComputeDisparity(FILE_MODE);
    ITMLibSettings *settings = new ITMLibSettings();
    
    ImageSourceEngine *imageSource = new ImageFileReader(settings->calibPath.c_str(), 
                                                         settings->leftImagePath.c_str(), 
                                                         settings->rightImagePath.c_str(), 
                                                         settings->disparityPath.c_str());
    imageSource->calib.disparityCalib.type = ITMDisparityCalib::TRAFO_AFFINE;
    imageSource->calib.disparityCalib.params = Vector2f(1.0f/1000.0f, 0.0f);

    ITMMainEngine *mainEngine = new ITMMainEngine(settings, &imageSource->calib, imageSource->getRGBImageSize());
    
    thread computeDisparityThread(&ComputeDisparity::Run,computeDisparity);
    computeDisparityThread.detach();

    UIEngine::Instance()->Initialise(imageSource, mainEngine, settings->deviceType);
    UIEngine::Instance()->Run();
    UIEngine::Instance()->Shutdown();

    delete mainEngine;
    delete settings;
    delete imageSource;
    return 0;
}

