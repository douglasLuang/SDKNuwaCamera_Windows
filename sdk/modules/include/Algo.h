#pragma once

#include <stdlib.h>
#include "deviceDataDef.h"

class Algo
{
public:
    void processDepthRawData(unsigned char* data, const size_t height, const size_t width);
    int depthToCloudDot(unsigned short* data, const size_t width, const size_t height, IrRgbParameter irRgbParameter,
        float* cloudDot);
    // cv::Mat ConvertDepthToPointCloud(cv::Mat depth, float K[4]);
    // void rotateImage(cv::Mat &srcMat, cv::Mat &dstMat);
    int denoises(unsigned char* data, const size_t width, const size_t height, const short newVal, const int maxSpeckleSize,
        const short maxDiff, const int max_th);
    int fillHoles(unsigned char* data, const size_t width, const size_t height, const int maxSpeckleSize,
        const int windows);
    int clusterSegmentationFilter(ushort* depth, int rows, int cols, ushort diffThreshold, int segmentSizeThreshold);
    void filling2(ushort* depth, int rows, int cols, int times);
    int downSample(unsigned short* dest,
        size_t& destWidth,
        size_t& destHeight,
        unsigned short* src,
        size_t srcWidth,
        size_t srcHeight,
        int channel,
        int step);
};
