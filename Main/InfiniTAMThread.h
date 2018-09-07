#ifndef INFINITAMTHREAD_H
#define INFINITAMTHREAD_H

#include <iostream>
#include <string>

#include "InfiniTAM.h"
#include "Engine/UIEngine.h"

//kitti dataset
const static std::string INFINITAM_POSE_PATH  = "/poses.txt";
const static std::string INFINITAM_CALIB_PATH = "/calib.txt";
const static std::string INFINITAM_LEFT_IMAGE_PATH  = "/image_2/%06i.png";
const static std::string INFINITAM_RIGHT_IMAGE_PATH = "/image_3/%06i.png";
const static std::string INFINITAM_DISPARITY_PATH   = "/Disp-Net/%06i.pfm";

class InfiniTAMThread
{

public:

    InfiniTAMThread();
    ~InfiniTAMThread();
    
    void InitInfiniTAM(std::string inputDirectory );
    void Run();

protected:


};

#endif
