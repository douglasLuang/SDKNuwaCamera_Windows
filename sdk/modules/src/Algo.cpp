#include <vector>
#include <cmath>
#include "Algo.h"
#include "Logger.h"
#include "deviceDataDef.h"
#include "dsense_interface.h"
//#include <arm_neon.h>

#define MAXDEPTHDISTANCE 5000

#define roiHighColor 254
class roiPoint
{
public:
    int xxx;
    int yyy;
public:
    bool operator<(const roiPoint& other) const
    {
        if (xxx < other.xxx) {
            return true;
        }
        else if (xxx == other.xxx) {
            return yyy < other.yyy;
        }
        else {
            return false;
        }
    }

};

#ifndef ROICONNECTEDCOMPONENT
#define ROICONNECTEDCOMPONENT
typedef struct {
    int totalPoints;
    int minX;
    int maxX;
    int minY;
    int maxY;
    int width;
    int height;
    roiPoint* points;
} roiConnectedComponent;
#endif

void Algo::processDepthRawData(unsigned char* data, const size_t height, const size_t width)
{
    if (data == NULL) {
        return;
    }
    unsigned short* data_ptr = (unsigned short*)data;
    unsigned short* data_end = data_ptr + height * width;
    while (data_ptr < data_end) {
        // base on little-endian
        *data_ptr = ((*data_ptr) >> 4) + (((*data_ptr) >> 3) & 1);
        data_ptr++;
    }

    return;
}

/*
int depthToCloudDot_neon(float* inputdepth,unsigned short *data, const size_t width, const size_t height, IrRgbParameter irRgbParameter, std::vector<float> &cloudDot)
{
    if (data == NULL) {
        LOG(ERROR) << "null ptr!\n";
        return -1;
    }

    cloudDot.resize(0);

    //begin
    for (int i = 0; i < width*height; i++) {
        inputdepth[i] = (float)data[i];
    }

    float fxir = irRgbParameter.irFx;
    float fyir = irRgbParameter.irFy;
    float cxir = irRgbParameter.irCx;
    float cyir = irRgbParameter.irCy;
    float32x4_t cxir_32 = vdupq_n_f32(cxir);
    float32x4_t fxir_32 = vdupq_n_f32(fxir);
    float32x4_t cyir_32 = vdupq_n_f32(cyir);
    float32x4_t fyir_32 = vdupq_n_f32(fyir);

    int step_neon = 4;
    int step = width / step_neon;
    float* tmp_depth = inputdepth;
    float* nextBuf = (float*)malloc(width*4 );
    if(nextBuf == NULL) {
        return -1;
    }
    for(int i = 0; i < width*2; i++) {
        nextBuf[i] = i;
    }
    float* xBuf = nextBuf + width*2;
    float* yBuf = xBuf + width;

    unsigned short* tmp_data = data;

    for (int i = 0; i < height; i++) {
        unsigned short* input_data = tmp_data;
        float* input_tmp_depth = tmp_depth;
        float* tmpj = nextBuf;
        float tmpi = i;
        float32x4_t i_32 = vdupq_n_f32(tmpi);
        for (int k = 0; k < step ; k++,input_data+=step_neon,input_tmp_depth+=step_neon,tmpj+=step_neon) {

            float32x4_t depth_32 = vld1q_f32(input_tmp_depth);
            float32x4_t j_32 = vld1q_f32(tmpj);

            float32x4_t jcxir_32 = vsubq_f32(j_32,cxir_32);
            float32x4_t jcxirdepth_32 = vmulq_f32(jcxir_32,depth_32);
            float32x4_t icyir_32 = vsubq_f32(i_32,cyir_32);
            float32x4_t icyirdepth_32 = vmulq_f32(icyir_32,depth_32);

            float32x4_t fxir_reciprocal = vrecpeq_f32(fxir_32);        //求得初始估计值
            fxir_reciprocal = vmulq_f32(vrecpsq_f32(fxir_32, fxir_reciprocal), fxir_reciprocal);    //逼近
            float32x4_t tmp0 = vmulq_f32(jcxirdepth_32,fxir_reciprocal);
            float32x4_t fyir_reciprocal = vrecpeq_f32(fyir_32);        //求得初始估计值
            fyir_reciprocal = vmulq_f32(vrecpsq_f32(fyir_32, fyir_reciprocal), fyir_reciprocal);    //逼近
            float32x4_t tmp1 = vmulq_f32(icyirdepth_32,fyir_reciprocal);

            vst1q_f32(xBuf,tmp0);
            vst1q_f32(yBuf,tmp1);

            cloudDot.emplace_back(tmp0[0]);
            cloudDot.emplace_back(tmp1[0]);
            cloudDot.emplace_back(input_data[0]);

            cloudDot.emplace_back(tmp0[1]);
            cloudDot.emplace_back(tmp1[1]);
            cloudDot.emplace_back(input_data[1]);

            cloudDot.emplace_back(tmp0[2]);
            cloudDot.emplace_back(tmp1[2]);
            cloudDot.emplace_back(input_data[2]);

            cloudDot.emplace_back(tmp0[3]);
            cloudDot.emplace_back(tmp1[3]);
            cloudDot.emplace_back(input_data[3]);
        }
        tmp_depth += width;
        tmp_data += width;
    }

    if(nextBuf != NULL) {
        free(nextBuf);
    }
    return 0;
}
*/

int Algo::depthToCloudDot(unsigned short* data, const size_t width, const size_t height, IrRgbParameter irRgbParameter,
    float* cloudDot)
{
    if (data == NULL) {
        LOG(ERROR) << "null ptr!\n";
        return -1;
    }

    //  cloudDot.resize(0);
    int count = 0;
    for (size_t y = 0; y < height; y++) {
        for (size_t x = 0; x < width; x++) {
            unsigned short depth = *(data + (y * width + x));
            if (depth > MAXDEPTHDISTANCE || std::fabs(depth - 0.0) < 1e-5) continue;
            float x_ = (x - irRgbParameter.irCx) * depth / irRgbParameter.irFx;
            float y_ = (y - irRgbParameter.irCy) * depth / irRgbParameter.irFy;
            *(cloudDot + (count * 3)) = x_;
            *(cloudDot + (count * 3 + 1)) = y_;
            *(cloudDot + (count * 3 + 2)) = depth;
            count++;
        }
    }

    return (count * 3);

    //  float bufferSize = sizeof(float)*width*height;
    //  depthToCloudDot_neon(&bufferSize,data,width,height,irRgbParameter,cloudDot);
    // return 0;
}

/* cv::Mat Algo::ConvertDepthToPointCloud(cv::Mat depth, float K[4])
{
    const float fx = K[0];
    const float fy = K[1];
    const float cx = K[2];
    const float cy = K[3];
    int step = 1;
    cv::Mat matdepth(depth.size(), CV_32FC3);
    int id = 0;
    for (int i = 0; i < depth.rows; i += step) {
        for (int j = 0; j < depth.cols; j += step) {
            double pixels_distance = depth.ptr<short>(i)[j] * 0.001;

            if (pixels_distance > 0.25f && pixels_distance < 5.0) {
                float vx = (j - cx) * pixels_distance / fx;
                float vy = (i - cy) * pixels_distance / fy;
                float vz = pixels_distance;
                cv::Vec3f val(vx, vy, vz);
                matdepth.at<cv::Vec3f>(i, j) = val;
            } else {
                matdepth.at<cv::Vec3f>(i, j) = cv::Vec3f(0.0f, 0.0f, 0.0f);
            }
        }
    }
    return matdepth.clone();
} */

/* void Algo::rotateImage(cv::Mat &srcMat, cv::Mat &dstMat)
{
    if (srcMat.empty())
        return;

    int width = srcMat.cols, height = srcMat.rows;

    if (srcMat.channels() == 3) {
        for (int k = 0; k < srcMat.channels(); k++) {
            for (int y = 0; y < height; y++) {
                for (int x = 0; x < width; x++) {
                    if (srcMat.type() == CV_32FC3) {
                        dstMat.at<cv::Vec3f>(x, y)[k] = srcMat.at<cv::Vec3f>(y, x)[k];
                    } else if (srcMat.type() == CV_16UC3) {
                        dstMat.at<cv::Vec3s>(x, y)[k] = srcMat.at<cv::Vec3s>(y, x)[k];
                    } else if (srcMat.type() == CV_8UC3) {
                        dstMat.at<cv::Vec3b>(x, y)[k] = srcMat.at<cv::Vec3b>(y, x)[k];
                    }
                }
            }
        }
    } else if (srcMat.channels() == 1) {
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                if (srcMat.type() == CV_32FC1) {
                    dstMat.at<float>(x, y) = srcMat.at<float>(y, x);
                } else if (srcMat.type() == CV_16UC1) {
                    dstMat.at<short>(x, y) = srcMat.at<short>(y, x);
                } else if (srcMat.type() == CV_8UC1) {
                    dstMat.at<char>(x, y) = srcMat.at<char>(y, x);
                }
            }
        }
    }
} */

int Algo::denoises(unsigned char* data, const size_t width, const size_t height, const short newVal,
    const int maxSpeckleSize, const short maxDiff, const int max_th)
{
    if (data == NULL) {
        LOG(ERROR) << "null ptr!\n";
        return -1;
    }

    unsigned char* tmpBuff = (unsigned char*)malloc(width * height * (3 * sizeof(int) + sizeof(unsigned char)));
    if (tmpBuff == NULL) {
        LOG(ERROR) << "try to apply memory failed!\n";
        return -2;
    }
    denoise((short*)data, newVal, maxSpeckleSize, maxDiff, tmpBuff, max_th, width, height);

    free(tmpBuff);
    return 0;
}

int Algo::fillHoles(unsigned char* data, const size_t width, const size_t height, const int maxSpeckleSize,
    const int windows)
{
    if (data == NULL) {
        LOG(ERROR) << "null ptr!\n";
        return -1;
    }

    int ret = 0;
    unsigned char* tempBuff = (unsigned char*)malloc(width * height * (4 * sizeof(int) + sizeof(unsigned char)));
    unsigned char* data_zero = (unsigned char*)malloc(width * height * sizeof(unsigned char));
    unsigned char* data_dst = (unsigned char*)malloc(width * height * sizeof(short));
    if (tempBuff == NULL ||
        data_zero == NULL) {
        LOG(ERROR) << "try to apply memory failed!\n";
        ret = -2;
        goto ERROR_EXIT;
    }

    memcpy(data_dst, data, width * height * sizeof(short));

    memset(data_zero, 0, width * height * sizeof(unsigned char));
    //memset(data_dst, 0, width * height * 2);
    filling((unsigned char*)data_zero, (short*)data_dst, (short*)data, 0, maxSpeckleSize, windows, tempBuff, width, height);
    
    
    
    free(tempBuff);
    free(data_zero);
    free(data_dst);
    return 0;

ERROR_EXIT: {
    if (tempBuff != NULL) {
        free(tempBuff);
        tempBuff = NULL;
    }
    if (data_zero != NULL) {
        free(data_zero);
        data_zero = NULL;
    }

    if (data_dst != NULL) {
        free(data_dst);
        data_dst = NULL;
    }

    return ret;
    }
}

int Algo::downSample(unsigned short* dest,
    size_t& destWidth,
    size_t& destHeight,
    unsigned short* src,
    size_t srcWidth,
    size_t srcHeight,
    int channel,
    int step)
{
    if (dest == NULL) {
        LOG(ERROR) << "dest is null ptr!";
        return -1;
    }
    if (src == NULL) {
        LOG(ERROR) << "src is null ptr!";
        return -1;
    }

    destHeight = srcHeight / step;
    destWidth = srcWidth / step;
    unsigned short* srcptr = src;
    unsigned short* destptr = dest;

    int step1 = (step - 1) * channel;
    int step2 = (step - 1) * srcWidth * channel;

    for (size_t h = 0; h < destHeight; h++) {
        for (size_t w = 0; w < destWidth; w++) {
            for (int c = 0; c < channel; c++) {
                *destptr = *srcptr;
                srcptr++;
                destptr++;
            }
            srcptr += step1;
        }
        src += step2;
    }
    return 0;
}

int Algo::clusterSegmentationFilter(ushort* depth, int rows, int cols, ushort diffThreshold, int segmentSizeThreshold)
{
    if (!depth) {
        printf("!depth\n");
        return -1;
    }

    int datalength = rows * cols;
    uchar* workBuffer = (uchar*)malloc(datalength);
    if (workBuffer == nullptr) {
        LOG(ERROR) << "malloc memory for filter failed" << std::endl;
        return -1;
    }
    roiPoint* points = (roiPoint*)malloc(datalength * sizeof(roiPoint));
    if (points == nullptr) {
        LOG(ERROR) << "malloc memory for roi failed" << std::endl;
        free(workBuffer);
        return -1;
    }
    int xxx1, yyy1, xxx2, yyy2;
    int minX, maxX, minY, maxY;
    int location1, location2;
    roiPoint* point1, * point2, * point3;
    int indexPoint, indexPointToBeProcessed;

    indexPoint = 0;
    indexPointToBeProcessed = 0;
#if 1
    minX = 1;
    maxX = cols - 1;
    minY = 1;
    maxY = rows - 1;
#else
    minX = 0;
    maxX = cols;
    minY = 0;
    maxY = rows;
#endif

    memset(workBuffer, 0, datalength);

    for (yyy1 = minY; yyy1 < maxY; ++yyy1) {
        for (xxx1 = minX; xxx1 < maxX; ++xxx1) {
            location1 = yyy1 * cols + xxx1;
            if (depth[location1] && workBuffer[location1] == 0) {
                workBuffer[location1] = roiHighColor;
                point1 = &points[indexPoint++];
                point1->xxx = xxx1;
                point1->yyy = yyy1;
                roiConnectedComponent comp;
                comp.minX = xxx1;
                comp.maxX = xxx1;
                comp.minY = yyy1;
                comp.maxY = yyy1;
                comp.totalPoints = 1;
                comp.points = point1;
                while (indexPointToBeProcessed < indexPoint) {
                    point2 = &points[indexPointToBeProcessed++];
                    ushort value = depth[point2->yyy * cols + point2->xxx];
#define __A_D_D__ \
                    workBuffer[location2] = roiHighColor; \
                    point3 = &points[indexPoint++]; \
                    point3->xxx = xxx2; \
                    point3->yyy = yyy2; \
                    ++comp.totalPoints;
                    if (point2->yyy >= minY) {
                        xxx2 = point2->xxx;
                        yyy2 = point2->yyy - 1;
                        location2 = yyy2 * cols + xxx2;
                        if (depth[location2] && workBuffer[location2] == 0 && abs(value - depth[location2]) < diffThreshold) {
                            __A_D_D__;
                            comp.minY = yyy2 < comp.minY ? yyy2 : comp.minY;
                        }
                    }
                    if (point2->xxx >= minX) {
                        xxx2 = point2->xxx - 1;
                        yyy2 = point2->yyy;
                        location2 = yyy2 * cols + xxx2;
                        if (depth[location2] && workBuffer[location2] == 0 && abs(value - depth[location2]) < diffThreshold) {
                            __A_D_D__;
                            comp.minX = xxx2 < comp.minX ? xxx2 : comp.minX;
                        }
                    }
                    if (point2->xxx < maxX) {
                        xxx2 = point2->xxx + 1;
                        yyy2 = point2->yyy;
                        location2 = yyy2 * cols + xxx2;
                        if (depth[location2] && workBuffer[location2] == 0 && abs(value - depth[location2]) < diffThreshold) {
                            __A_D_D__;
                            comp.maxX = xxx2 > comp.maxX ? xxx2 : comp.maxX;
                        }
                    }
                    if (point2->yyy < maxY) {
                        xxx2 = point2->xxx;
                        yyy2 = point2->yyy + 1;
                        location2 = yyy2 * cols + xxx2;
                        if (depth[location2] && workBuffer[location2] == 0 && abs(value - depth[location2]) < diffThreshold) {
                            __A_D_D__;
                            comp.maxY = yyy2 > comp.maxY ? yyy2 : comp.maxY;
                        }
                    }
                    if (point2->xxx >= minX && point2->yyy >= minY) {
                        xxx2 = point2->xxx - 1;
                        yyy2 = point2->yyy - 1;
                        location2 = yyy2 * cols + xxx2;
                        if (depth[location2] && workBuffer[location2] == 0 && abs(value - depth[location2]) < diffThreshold) {
                            __A_D_D__;
                            comp.minX = xxx2 < comp.minX ? xxx2 : comp.minX;
                            comp.minY = yyy2 < comp.minY ? yyy2 : comp.minY;
                        }
                    }
                    if (point2->xxx < maxX && point2->yyy >= minY) {
                        xxx2 = point2->xxx + 1;
                        yyy2 = point2->yyy - 1;
                        location2 = yyy2 * cols + xxx2;
                        if (depth[location2] && workBuffer[location2] == 0 && abs(value - depth[location2]) < diffThreshold) {
                            __A_D_D__;
                            comp.maxX = xxx2 > comp.maxX ? xxx2 : comp.maxX;
                            comp.minY = yyy2 < comp.minY ? yyy2 : comp.minY;
                        }
                    }
                    if (point2->xxx >= minX && point2->yyy < maxY) {
                        xxx2 = point2->xxx - 1;
                        yyy2 = point2->yyy + 1;
                        location2 = yyy2 * cols + xxx2;
                        if (depth[location2] && workBuffer[location2] == 0 && abs(value - depth[location2]) < diffThreshold) {
                            __A_D_D__;
                            comp.minX = xxx2 < comp.minX ? xxx2 : comp.minX;
                            comp.maxY = yyy2 > comp.maxY ? yyy2 : comp.maxY;
                        }
                    }
                    if (point2->xxx < maxX && point2->yyy < maxY) {
                        xxx2 = point2->xxx + 1;
                        yyy2 = point2->yyy + 1;
                        location2 = yyy2 * cols + xxx2;
                        if (depth[location2] && workBuffer[location2] == 0 && abs(value - depth[location2]) < diffThreshold) {
                            __A_D_D__;
                            comp.maxX = xxx2 > comp.maxX ? xxx2 : comp.maxX;
                            comp.maxY = yyy2 > comp.maxY ? yyy2 : comp.maxY;
                        }
                    }
#undef __A_D_D__
                }

                if (comp.totalPoints <= segmentSizeThreshold) {

                    for (int i = 0; i < comp.totalPoints; i++) {
                        int pos = comp.points[i].yyy * cols + comp.points[i].xxx;
                        depth[pos] = 0;
                    }


                }
            }
        }
    }

    free(workBuffer);
    free(points);

    return 0;
}

void Algo::filling2(ushort* depth, int rows, int cols, int times)
{
    ushort* buffer = new ushort[rows * cols];
    memcpy(buffer, depth, rows * cols * sizeof(ushort));
    for (int i = 0; i < times; i++) {
        for (int r = 1; r < rows - 1; r++) {
            for (int c = 1; c < cols - 1; c++) {
                int pos = r * cols + c;
                if (!buffer[pos]) {
                    if (buffer[pos - 1]) {
                        if (buffer[pos + 1] || buffer[pos + cols] || buffer[pos - cols]) {
                            depth[pos] = buffer[pos - 1];
                        }
                    }
                    else if (buffer[pos + 1]) {
                        if (buffer[pos + cols] || buffer[pos - cols]) {
                            depth[pos] = buffer[pos + 1];
                        }

                    }
                    else if (buffer[pos - cols]) {
                        if (buffer[pos + cols]) {
                            depth[pos] = buffer[pos - cols];
                        }

                    }
                    else if (buffer[pos - cols - 1] && buffer[pos + cols + 1]) {
                        depth[pos] = buffer[pos - cols - 1];
                    }
                    else if (buffer[pos + cols - 1] && buffer[pos - cols + 1]) {
                        depth[pos] = buffer[pos + cols - 1];
                    }
                }

            }
        }
        memcpy(buffer, depth, rows * cols * sizeof(ushort));
    }
    delete buffer;
}
