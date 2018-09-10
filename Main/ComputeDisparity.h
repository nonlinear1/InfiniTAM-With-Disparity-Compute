#ifndef COMPUTEDISPARITY_H
#define COMPUTEDISPARITY_H

#include <time.h>
#include "Disparity/libelas/src/elas.h"
#include "Disparity/BM_SGBM/blockmatching.h"
#include "Disparity/BM_SGBM/sgblockmatching.h"
#include "Disparity/SPS_Stereo/SPSStereo.h"


class ComputeDisparity
{

public:

    ComputeDisparity(char disparityMode);
    ~ComputeDisparity();

    void Run();

    struct StereoImageParameters
    {
        unsigned char *I1, *I2;  // I1, I2 are the input image.
        float         *D1, *D2;  // D1, D2 are the disparity map. color: close-white-max  far-black-min(0) 
                                 //                               color: colse-green      far-red
        int width, height, step;    
        double focalLength, baseLine;  //  stereo focal length, baseline.

        StereoImageParameters() // Default constructor.
        {
            I1 = I2 = 0;
            D1 = D2 = 0;
            width = height = step = 0;
        }

        // Copy constructor.
        StereoImageParameters( ComputeDisparity::StereoImageParameters &src)
        {
            I1 = I2 = 0;
            D1 = D2 = 0;
            width = src.width; height = src.height; step = src.step;

            if (src.I1!=0) {
                I1 = (unsigned char*)malloc(src.step*src.height*sizeof(unsigned char));
                memcpy(I1, src.I1, src.step*src.height*sizeof(unsigned char));
            }
            if (src.I2!=0) {
                I2 = (unsigned char*)malloc(src.step*src.height*sizeof(unsigned char));
                memcpy(I2, src.I2, src.step*src.height*sizeof(unsigned char));
            }
            if (src.D1!=0) {
                D1 = (float*)malloc(src.step*src.height*sizeof(float));
                memcpy(D1, src.D1, src.step*src.height*sizeof(float));
            }
            if (src.D2!=0) {
                D2 = (float*)malloc(src.step * src.height * sizeof(float));
                memcpy(D2, src.D2, src.step * src.height * sizeof(float));
            }
        }
        
        ~StereoImageParameters ()
        {
            if (I1!=0) { free(I1); I1 = 0; }
            if (I2!=0) { free(I2); I2 = 0; }
            if (D1!=0) { free(D1); D1 = 0; }
            if (D2!=0) { free(D2); D2 = 0; }
        }
    };

    char computeDisparityType;
    StereoImageParameters stereoImage;


private:

    void bm_arrayToMat(uchar* arr, cv::Mat &image);
    void bm_matToArray(cv::Mat image, float* arr);
    void sps_arrayToPng(uchar* arr, png::image<png::rgb_pixel> &imagePng);
    void sps_pngToArrary(png::image<png::gray_pixel_16> &imagePng, float* arr);

    Elas::parameters param;
    BlockMatching bm;
    SGBlockMatching sgbm;
    SPSStereo sps;
};

#endif
