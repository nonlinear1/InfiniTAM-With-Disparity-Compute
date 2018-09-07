#include "InfiniTAMThread.h"


InfiniTAMThread::InfiniTAMThread()
{

}

InfiniTAMThread::~InfiniTAMThread()
{
    UIEngine::Instance()->Shutdown();

    delete mainEngine;
    delete settings;
    delete imageSource;
}

void InfiniTAMThread::InitInfiniTAM(std::string inputDirectory )
{
    std::string posePath       = inputDirectory + INFINITAM_POSE_PATH;
    std::string calibPath      = inputDirectory + INFINITAM_CALIB_PATH;
    std::string dispPath       = inputDirectory + INFINITAM_DISPARITY_PATH;
    std::string leftImagePath  = inputDirectory + INFINITAM_LEFT_IMAGE_PATH;
    std::string rightImagePath = inputDirectory + INFINITAM_RIGHT_IMAGE_PATH;
    
    imageSource = new ImageFileReader( calibPath.data(), leftImagePath.data(),
                                       rightImagePath.data(), dispPath.data() );
    if (imageSource == NULL)  return;
    if (imageSource->calib.disparityCalib.params == Vector2f(0.0f, 0.0f))
    {
        imageSource->calib.disparityCalib.type = ITMDisparityCalib::TRAFO_AFFINE;
        imageSource->calib.disparityCalib.params = Vector2f(1.0f/1000.0f, 0.0f);
    }
    settings = new ITMLibSettings();
    settings->posePath = posePath;

    mainEngine = new ITMMainEngine( settings, &imageSource->calib, imageSource->getRGBImageSize() );
    UIEngine::Instance()->Initialise( imageSource, mainEngine, settings->deviceType );
}


void InfiniTAMThread::Run()
{
    UIEngine::Instance()->Run();
}
