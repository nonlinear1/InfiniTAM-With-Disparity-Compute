#include "ComputeDisparity.h"
#include "InfiniTAM.h"

ComputeDisparity::ComputeDisparity(char disparityMode)
{
    this->computeDisparityType = disparityMode;
    
    // init the compute disparity mode parameters.
    this->sps.initSpsParameter();
    this->bm.initBmParameter();
    this->sgbm.initSgbmParameter();
    // elas
    this->param.postprocess_only_left = true;
    this->param.filter_adaptive_mean = true;
    this->param.support_texture = 30;
}

ComputeDisparity::~ComputeDisparity()
{

}

void ComputeDisparity::bm_arrayToMat(uchar* arr, cv::Mat &image)
{
    for(int i=0; i<this->stereoImage.height; i++)
        for(int j=0; j<this->stereoImage.step; j++)
        { image.at<uchar>(i,j) = *arr, arr++; }
}

void ComputeDisparity::bm_matToArray(cv::Mat image, float* arr)
{
    for(int i=0; i<this->stereoImage.height; i++)
        for(int j=0; j<this->stereoImage.width; j++)
        { *arr = image.at<float>(i,j); arr++; }
}

void ComputeDisparity::sps_arrayToPng(uchar* arr, png::image<png::rgb_pixel> &imagePng)
{
    for(int i=0; i<this->stereoImage.height; i++)
        for(int j=0; j<this->stereoImage.step; j++)
        {
            png::rgb_pixel rgbPixel(*arr, *arr, *arr);
            imagePng.set_pixel(j, i, rgbPixel);
            arr++;
        }
}

void ComputeDisparity::sps_pngToArrary(png::image<png::gray_pixel_16> &imagePng, float* arr)
{
    for(int i=0; i<this->stereoImage.height; i++)
        for(int j=0; j<this->stereoImage.width; j++)
        {   *arr = (imagePng.get_pixel(j, i) / 256.0); arr++;  }
}

void ComputeDisparity::Run()
{   
    clock_t startTime,endTime;
    while(1)
    {
        //  infinitam thread dont need to compute disparity , wait for read stereo image.
        while (!g_WaitComputeDisparity) usleep(1);   

        int32_t d_width  = this->stereoImage.width;
        int32_t d_height = this->stereoImage.height;
        int32_t d_step   = this->stereoImage.step;    

        //free the disparity pointer.
        if(this->stereoImage.D1 != 0)
        {    free(this->stereoImage.D1); this->stereoImage.D1 = 0;  }

        if(this->stereoImage.D2 != 0)
        {    free(this->stereoImage.D2); this->stereoImage.D2 = 0;  }
        
        this->stereoImage.D1 = (float*)malloc(d_width*d_height*sizeof(float));
        this->stereoImage.D2 = (float*)malloc(d_width*d_height*sizeof(float));

        //compute disparity mode,contain Elas, BM and SGBM.
        //init ELas
        Elas elas(param);
        const int32_t dims[3] = {d_width, d_height, d_step};

        //init SGBM BM
        Mat bm_leftImage( d_height, d_step, CV_8U);
        Mat bm_rightImage( d_height, d_step, CV_8U);

        bm_arrayToMat(this->stereoImage.I1, bm_leftImage);
        bm_arrayToMat(this->stereoImage.I2, bm_rightImage);

        //init SPS_Stereo
        png::image<png::rgb_pixel> sps_leftImage(d_step, d_height);
        png::image<png::rgb_pixel> sps_rightImage(d_step, d_height);
        png::image<png::gray_pixel_16> segmentImage;
        png::image<png::gray_pixel_16> disparityImage;

        sps_arrayToPng(this->stereoImage.I1, sps_leftImage);
        sps_arrayToPng(this->stereoImage.I2, sps_rightImage);
        
        startTime = clock();

        switch( this->computeDisparityType )
        {
            case ELAS_MODE:

                elas.process(this->stereoImage.I1,this->stereoImage.I2,this->stereoImage.D1,this->stereoImage.D2,dims);
                break;

            case SGBM_MODE:
                //sgbm->compute(I1, I2, D1)
                sgbm.setImages(bm_leftImage, bm_rightImage);
                this->bm_matToArray(sgbm.updateSGBM(), this->stereoImage.D1);
                break;

            case BM_MODE:
                //bm->compute(I1, I2, D1)
                bm.setImages(bm_leftImage, bm_rightImage);
                this->bm_matToArray(bm.updateBM(), this->stereoImage.D1);
                break;

            case SPS_MODE:
                //sps.compute(__, left, right, segmentImage, disparityImage);
                sps.compute(superpixelTotal, sps_leftImage, sps_rightImage, segmentImage, disparityImage);
                this->sps_pngToArrary(disparityImage, this->stereoImage.D1);
                break;
                
            default: break;
        }

        endTime = clock();
        
        cout <<"Compute disparity use: " << (double)(endTime-startTime)/CLOCKS_PER_SEC << "s." <<endl;
        
        g_WaitComputeDisparity = false;  // compete computing disparity.
    }
}
