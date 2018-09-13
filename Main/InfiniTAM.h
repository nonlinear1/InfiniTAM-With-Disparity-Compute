#ifndef INFINITAM_H
#define INFINITAM_H

#define FILE_MODE  0
#define ELAS_MODE  1
#define BM_MODE    2
#define SGBM_MODE  3
#define SPS_MODE   4

#include <thread>
#include <mutex>
#include <GL/glut.h>
#include "Engine/UIEngine.h"
#include "Engine/ImageSourceEngine.h"
#include "ITMLib/Engine/ITMMainEngine.h"

#include "ComputeDisparity.h"

extern ComputeDisparity *computeDisparity;
extern bool g_WaitComputeDisparity;

using namespace InfiniTAM::Engine;


#endif
