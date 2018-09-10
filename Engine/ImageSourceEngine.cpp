// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ImageSourceEngine.h"

#include "../Utils/FileUtils.h"

#include <Eigen/Core>

#include <stdio.h>


using namespace InfiniTAM::Engine;

//=================================   Creat Calibration==============================//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double, 3, 4> ReadProjection(const std::string &expected_label, std::istream &in)
{
  Eigen::Matrix<double, 3, 4> matrix;
  std::string label;
  in >> label;
  assert(expected_label == label && "Unexpected token in calibration file.");

  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 4; ++col) {
      in >> matrix(row, col);
    }
  }

  return matrix;
}

void ReadKittiOdometryCalibration(const std::string &fpath,
                                  Eigen::Matrix<double, 3, 4> &left_gray_proj/*,
                                  Eigen::Matrix<double, 3, 4> &right_gray_proj,
                                  Eigen::Matrix<double, 3, 4> &left_color_proj,
                                  Eigen::Matrix<double, 3, 4> &right_color_proj,
                                  Eigen::Matrix4d &velo_to_left_cam*/) {
  static const std::string kLeftGray = "P0:";
//  static const std::string kRightGray = "P1:";
//  static const std::string kLeftColor = "P2:";
//  static const std::string kRightColor = "P3:";
  std::ifstream in(fpath);
  if (! in.is_open()) {
    std::cout<<"Can't open calib.txt. " <<std::endl;
  }

  left_gray_proj = ReadProjection(kLeftGray, in);
//  right_gray_proj = ReadProjection(kRightGray, in);
//  left_color_proj = ReadProjection(kLeftColor, in);
//  right_color_proj = ReadProjection(kRightColor, in);

//  std::string dummy;
//  in >> dummy;
//  if (dummy != "Tr:") {
//    std::getline(in, dummy); // skip to the end of current line
//    in >> dummy;
//    std::cout<<dummy<<std::endl;
//  }

//  for (int row = 0; row < 3; ++row) {
//    for (int col = 0; col < 4; ++col) {
//      in >> velo_to_left_cam(row, col);
//    }
//  }
//  velo_to_left_cam(3, 0) = 0.0;
//  velo_to_left_cam(3, 1) = 0.0;
//  velo_to_left_cam(3, 2) = 0.0;
//  velo_to_left_cam(3, 3) = 1.0;
}

ITMRGBDCalib *CreateItmCalib(const Eigen::Matrix<double, 3, 4> left_cam_proj/*, const Eigen::Vector2i frame_size*/)
{
  ITMRGBDCalib *calib = new ITMRGBDCalib;
  float kMetersToMillimeters = 1.0f / 1000.0f;

  ITMIntrinsics intrinsics;
  float fx = static_cast<float>(left_cam_proj(0, 0));
  float fy = static_cast<float>(left_cam_proj(1, 1));
  float cx = static_cast<float>(left_cam_proj(0, 2));
  float cy = static_cast<float>(left_cam_proj(1, 2));
  float sizeX = 2000;
  float sizeY = 1000;
  intrinsics.SetFrom(fx, fy, cx, cy, sizeX, sizeY);

  calib->intrinsics_rgb = intrinsics;
  calib->intrinsics_d = intrinsics;

  Matrix4f identity; identity.setIdentity();
  calib->trafo_rgb_to_depth.SetFrom(identity);

  calib->disparityCalib.SetFrom(kMetersToMillimeters, 0.0f, ITMDisparityCalib::TRAFO_AFFINE);
  return calib;
}

void GetKittiCalib(const char *fileName, ITMRGBDCalib & calib)
{
    Eigen::Matrix<double, 3, 4> left_gray_proj;
    Eigen::Matrix<double, 3, 4>  right_gray_proj;
    Eigen::Matrix<double, 3, 4>  left_color_proj;
    Eigen::Matrix<double, 3, 4>  right_color_proj;
    Eigen::Matrix4d velo_to_left_gray_cam;
//    Eigen::Vector2i frame_size(imgSize.width, imgSize.height);

    ReadKittiOdometryCalibration(fileName, left_gray_proj/*, right_gray_proj, left_color_proj,             right_color_proj,velo_to_left_gray_cam*/);


    //Stereo camera calibration.  Camera Focal and Baseling
    computeDisparity->stereoImage.focalLength = left_gray_proj(0, 0);
    computeDisparity->stereoImage.baseLine = 0.537150654273f;

    calib = *CreateItmCalib(left_gray_proj/*, frame_size*/);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////

ImageSourceEngine::ImageSourceEngine(const char *calibFilename)
{
    GetKittiCalib(calibFilename, calib);
}

ImageFileReader::ImageFileReader(const char *calibFilename, const char *leftRgbImageMask, 
                                 const char *rightRgbImageMask, const char *depthImageMask)
    :ImageSourceEngine(calibFilename)
{    
    strncpy(this->leftRgbImageMask, leftRgbImageMask, BUF_SIZE);
    strncpy(this->rightRgbImageMask, rightRgbImageMask, BUF_SIZE);
	strncpy(this->depthImageMask, depthImageMask, BUF_SIZE);

	currentFrameNo = 0;
	cachedFrameNo = -1;

    cached_left_rgb = NULL;
    cached_right_rgb = NULL;
	cached_depth = NULL;
}

ImageFileReader::~ImageFileReader()
{
    delete cached_right_rgb;
	delete cached_left_rgb;
	delete cached_depth;
}

void ImageFileReader::loadIntoCache(void)
{
	if (currentFrameNo == cachedFrameNo && currentFrameNo != 0) return;
	cachedFrameNo = currentFrameNo;

	//TODO> make nicer
	cached_left_rgb = new ITMUChar4Image(true, false);
    cached_right_rgb = new ITMUChar4Image(true, false); 
	cached_depth = new ITMShortImage(true, false);

	char str[2048];
    char str2[2048];

    sprintf(str,  leftRgbImageMask,  currentFrameNo);
    sprintf(str2, rightRgbImageMask, currentFrameNo);
    
    // if not file mode, read rgb-image and copy to computeDisparity->stereoImage.I1.I2.step.height.width
    // meanwhile need to wait compute disparity.( wait = true ). 
	if (!ReadImageFromFile(cached_left_rgb, str, cached_right_rgb, str2)) 
	{
		delete cached_left_rgb; cached_left_rgb = NULL;
        delete cached_right_rgb; cached_right_rgb = NULL;
        return;
	}
	
    Vector2i newSize(cached_left_rgb->noDims.x, cached_left_rgb->noDims.y);
    cached_depth->ChangeDims(newSize);
}

bool ImageFileReader::hasMoreImages(void)
{
	loadIntoCache();
	return ( cached_left_rgb!=NULL );
}

void ImageFileReader::getImages(ITMUChar4Image *leftRgb, ITMUChar4Image *rightRgb, ITMShortImage *rawDepth)
{
    char str[2048];
    char str2[2048];

    // RGB use cache
    leftRgb->SetFrom(cached_left_rgb, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
    delete cached_left_rgb;
    cached_left_rgb = NULL;

    // Depth dont use cache 
    sprintf(str, depthImageMask, currentFrameNo);
    ReadImageFromFile(rawDepth, str);

	++currentFrameNo;
}

Vector2i ImageFileReader::getDepthImageSize(void)
{
	loadIntoCache();
	return cached_depth->noDims;
}

Vector2i ImageFileReader::getRGBImageSize(void)
{
	loadIntoCache();
	if (cached_left_rgb != NULL) return cached_left_rgb->noDims;
	return cached_depth->noDims;
}

CalibSource::CalibSource(const char *calibFilename, Vector2i setImageSize, float ratio)
	: ImageSourceEngine(calibFilename)
{
	this->imgSize = setImageSize;
	this->ResizeIntrinsics(calib.intrinsics_d, ratio);
	this->ResizeIntrinsics(calib.intrinsics_rgb, ratio);
}

void CalibSource::ResizeIntrinsics(ITMIntrinsics &intrinsics, float ratio)
{
	intrinsics.projectionParamsSimple.fx *= ratio;
	intrinsics.projectionParamsSimple.fy *= ratio;
	intrinsics.projectionParamsSimple.px *= ratio;
	intrinsics.projectionParamsSimple.py *= ratio;
	intrinsics.projectionParamsSimple.all *= ratio;
}

RawFileReader::RawFileReader(const char *calibFilename, const char *leftRgbImageMask, const char *rightRgbImageMask,
                             const char *depthImageMask, Vector2i setImageSize, float ratio) 
	: ImageSourceEngine(calibFilename)
{
	this->imgSize = setImageSize;
	this->ResizeIntrinsics(calib.intrinsics_d, ratio);
	this->ResizeIntrinsics(calib.intrinsics_rgb, ratio);
	
	strncpy(this->leftRgbImageMask, leftRgbImageMask, BUF_SIZE);
    strncpy(this->rightRgbImageMask, rightRgbImageMask, BUF_SIZE);
	strncpy(this->depthImageMask, depthImageMask, BUF_SIZE);

	currentFrameNo = 0;
	cachedFrameNo = -1;

	cached_left_rgb = NULL;
	cached_depth = NULL;
}

void RawFileReader::ResizeIntrinsics(ITMIntrinsics &intrinsics, float ratio)
{
	intrinsics.projectionParamsSimple.fx *= ratio;
	intrinsics.projectionParamsSimple.fy *= ratio;
	intrinsics.projectionParamsSimple.px *= ratio;
	intrinsics.projectionParamsSimple.py *= ratio;
	intrinsics.projectionParamsSimple.all *= ratio;
}

void RawFileReader::loadIntoCache(void)
{
	if (currentFrameNo == cachedFrameNo) return;
	cachedFrameNo = currentFrameNo;

	//TODO> make nicer
	cached_left_rgb = new ITMUChar4Image(imgSize, MEMORYDEVICE_CPU);
    cached_right_rgb = new ITMUChar4Image(imgSize, MEMORYDEVICE_CPU);
	cached_depth = new ITMShortImage(imgSize, MEMORYDEVICE_CPU);

	char str[2048], str2[2048]; FILE *f, *f2; bool success = false;

	sprintf(str,  leftRgbImageMask,  currentFrameNo);
    sprintf(str2, rightRgbImageMask, currentFrameNo);

	f = fopen(str, "rb");
	if (f)
	{
		size_t tmp = fread(cached_left_rgb->GetData(MEMORYDEVICE_CPU), sizeof(Vector4u), imgSize.x * imgSize.y, f);
		fclose(f);
		if (tmp == (size_t)imgSize.x * imgSize.y) success = true;
	}
	if (f2)
	{
		size_t tmp = fread(cached_right_rgb->GetData(MEMORYDEVICE_CPU), sizeof(Vector4u), imgSize.x * imgSize.y, f2);
		fclose(f2);
		if (tmp == (size_t)imgSize.x * imgSize.y) success = true;
	}
	if (!success)
	{
		delete cached_left_rgb; cached_left_rgb = NULL;
        delete cached_right_rgb; cached_right_rgb = NULL;
		printf("error reading file '%s'\n", str);
	}

	sprintf(str, depthImageMask, currentFrameNo); success = false;
	f = fopen(str, "rb");
	if (f)
	{
		size_t tmp = fread(cached_depth->GetData(MEMORYDEVICE_CPU), sizeof(short), imgSize.x * imgSize.y, f);
		fclose(f);
		if (tmp == (size_t)imgSize.x * imgSize.y) success = true;
	}
	if (!success)
	{
		delete cached_depth; cached_depth = NULL;
		printf("error reading file '%s'\n", str);
	}
}


bool RawFileReader::hasMoreImages(void)
{
	loadIntoCache(); 

	return ((cached_left_rgb != NULL) || (cached_depth != NULL));
}

void RawFileReader::getImages(ITMUChar4Image *leftRgb, ITMUChar4Image *rightRgb, ITMShortImage *rawDepth)
{
	bool bUsedCache = false;

	if (!bUsedCache) this->loadIntoCache();

	++currentFrameNo;
}
