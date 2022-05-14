/********************************
author:huangruopu
date:2021/05/21
function: depth postprocess algorithm
version:v1.0_210521
mark: first release version

1.depth to rgb register
2.filling hole and denoise

********************************/
#pragma once
#ifdef __cplusplus
extern "C" {
#endif
#define MAX(a,b) (a>b?a:b)
#define MIN(a,b) (a<b?a:b)
    typedef unsigned char uchar;//8bit
    typedef unsigned short ushort;//16bit
    typedef unsigned __int64 uint64;//64bit

    typedef struct point2s
    {
        int x;
        int y;
    }point2s;
    typedef struct faceRectMy
    {
        int x;
        int y;
        int width;
        int height;
    }faceRectIn;//人脸框的位置大小
    typedef struct ir_rgb_Parameter
    {
        //ir和彩色摄像头内参
        float fxir;
        float fyir;
        float cxir;
        float cyir;
        float fxrgb;
        float fyrgb;
        float cxrgb;
        float cyrgb;

        //ir和彩色摄像头外参
        float R00;
        float R01;
        float R02;
        float R10;
        float R11;
        float R12;
        float R20;
        float R21;
        float R22;
        float T1;
        float T2;
        float T3;

    }ir_rgb_State;//摄像头内外参


    //深度图与彩色图对齐接口
    void __declspec(dllexport) depth2RGB
    (
        short* input_ori_depth,                //输入深度图
        short* output_ori_depth,               //输出对齐后的深度图
        float* inputdepth,                     //输入buffer,需要内存大小4*width*height字节
        float* outdepth,                       //输出buffer,需要内存大小4*width*height字节
        float* tmpdepth,                       //中间buffer，需要内存大小4*width*height字节
        int width_rgb,                         //彩色图宽度
        int height_rgb,                        //彩色图高度
        int width_ir,                          //红外图宽度
        int height_ir,                         //红外图高度
        ir_rgb_State& para                     //摄像头内外参
    );


    //去除深度图噪点的接口
    void __declspec(dllexport) denoise(
        short* img,                //ori depth map change to clean depth map
        short newVal,              //error depth,if more than it ,not a bad depth 0
        int maxSpeckleSize,        //the maximum of speckle  , if less than it, is a speckle 800
        short maxDiff,             //the maximum of difference , if the pixel difference of two adjacent point is less than it,two point is the same speckle region 10
        unsigned char* _buf,       //buffer size = width*height*(3*sizeof(int)+sizeof(uchar)) <= 20M
        int max_th,                //remove out depth
        int width,                 //image width
        int height                 //image height
    );

    //深度图补洞的接口
    void __declspec(dllexport) filling(
        unsigned char* guide,         // gray edge map
        short* img,                   //ori depth map change to filled depth map
        short* img_dst,               //filled depth map
        short newVal,                 //error depth,if more than it ,not a bad depth  0
        int maxSpeckleSize,           //the maximum of speckle  , if less than it, is a speckle  800 
        int windows,                  //searching window for fill average value  10
        unsigned char* _buf,          //buffer size = width*height*(3*sizeof(int)+sizeof(uchar)) <= 13M
        int width,                    //image width
        int height                    //image height
    );

    //红外对齐到RGB的接口
    int __declspec(dllexport) warpPers(
        unsigned char* src,          //输入红外图
        unsigned char* dst,          //输出对齐后的红外图
        float* M,                   //输入3x3的RT矩阵
        int* mapxy,                  //buffer 8*width*height字节
        int* mapa,                   //buffer 8*width*height字节
        int width,                   //输出图像宽 480
        int height,                  //输出图像宽 768
        int src_w,                   //输入图像宽 400
        int src_h                    //输入图像宽 640
    );

    //深度图膨胀的接口
    int __declspec(dllexport) dilate_lite(
        short* src,                  //输入深度图
        short* dst,                  //输出膨胀后的深度图
        int width,                   //输入图像宽 480
        int height                   //输入图像高 768
    );

    //深度图  旋转+翻转接口
    int __declspec(dllexport) rotate_flip_depth(
        short* src,                  //输入深度图
        short* dst,                  //输出旋转+翻转后的深度图
        int width,                   //输入图像宽 640
        int height                   //输入图像高 400
    );

    //红外图  旋转+翻转接口
    int __declspec(dllexport) rotate_flip_ir(
        unsigned char* src,          //输入红外图
        unsigned char* dst,          //输出旋转+翻转后的红外图
        int width,                   //输入图像宽 640
        int height                   //输入图像高 400
    );

    //彩色图 旋转+翻转接口
    int __declspec(dllexport) rotate_flip_RGB(
        unsigned char* src,          //输入彩色图
        unsigned char* dst,          //输出旋转+翻转后的彩色图
        int width,                   //输入图像宽 768
        int height                   //输入图像高 480
    );
    //检查红外图是否过曝
    //返回值 红外人脸框平均亮度值
    int check_expo_ir(
        unsigned char* src, //输入红外图 
        faceRectIn rect,    //输入人脸框
        int width,          //红外图全图宽度
        int height          //红外图全图高度
    );

    //深度图补洞接口
    int  __declspec(dllexport) filling_face(
        unsigned char* guide,         // gray edge map 
        short* img,                   //ori disparity map 
        short* img_dst,               //filled disparity map
        short newVal,                 //error disparity,if more than it ,not a bad disparity
        int maxSpeckleSize,           //the maximum of speckle  , if less than it, is a speckle
        int windows,                  //searching window for fill average value
        unsigned char* _buf,          //buffer size = width*height*(3*sizeof(int)+sizeof(uchar)) <= 13M + 4M
        int ltx,                       //left top x cood
        int lty,                       //left top y cood
        int face_width,                //face width
        int face_height,                //face height
        int width,                    //image width
        int height                    //image height
    );


#ifdef __cplusplus
}
#endif

