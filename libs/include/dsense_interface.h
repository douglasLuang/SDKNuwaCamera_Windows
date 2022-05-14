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
    }faceRectIn;//�������λ�ô�С
    typedef struct ir_rgb_Parameter
    {
        //ir�Ͳ�ɫ����ͷ�ڲ�
        float fxir;
        float fyir;
        float cxir;
        float cyir;
        float fxrgb;
        float fyrgb;
        float cxrgb;
        float cyrgb;

        //ir�Ͳ�ɫ����ͷ���
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

    }ir_rgb_State;//����ͷ�����


    //���ͼ���ɫͼ����ӿ�
    void __declspec(dllexport) depth2RGB
    (
        short* input_ori_depth,                //�������ͼ
        short* output_ori_depth,               //������������ͼ
        float* inputdepth,                     //����buffer,��Ҫ�ڴ��С4*width*height�ֽ�
        float* outdepth,                       //���buffer,��Ҫ�ڴ��С4*width*height�ֽ�
        float* tmpdepth,                       //�м�buffer����Ҫ�ڴ��С4*width*height�ֽ�
        int width_rgb,                         //��ɫͼ���
        int height_rgb,                        //��ɫͼ�߶�
        int width_ir,                          //����ͼ���
        int height_ir,                         //����ͼ�߶�
        ir_rgb_State& para                     //����ͷ�����
    );


    //ȥ�����ͼ���Ľӿ�
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

    //���ͼ�����Ľӿ�
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

    //������뵽RGB�Ľӿ�
    int __declspec(dllexport) warpPers(
        unsigned char* src,          //�������ͼ
        unsigned char* dst,          //��������ĺ���ͼ
        float* M,                   //����3x3��RT����
        int* mapxy,                  //buffer 8*width*height�ֽ�
        int* mapa,                   //buffer 8*width*height�ֽ�
        int width,                   //���ͼ��� 480
        int height,                  //���ͼ��� 768
        int src_w,                   //����ͼ��� 400
        int src_h                    //����ͼ��� 640
    );

    //���ͼ���͵Ľӿ�
    int __declspec(dllexport) dilate_lite(
        short* src,                  //�������ͼ
        short* dst,                  //������ͺ�����ͼ
        int width,                   //����ͼ��� 480
        int height                   //����ͼ��� 768
    );

    //���ͼ  ��ת+��ת�ӿ�
    int __declspec(dllexport) rotate_flip_depth(
        short* src,                  //�������ͼ
        short* dst,                  //�����ת+��ת������ͼ
        int width,                   //����ͼ��� 640
        int height                   //����ͼ��� 400
    );

    //����ͼ  ��ת+��ת�ӿ�
    int __declspec(dllexport) rotate_flip_ir(
        unsigned char* src,          //�������ͼ
        unsigned char* dst,          //�����ת+��ת��ĺ���ͼ
        int width,                   //����ͼ��� 640
        int height                   //����ͼ��� 400
    );

    //��ɫͼ ��ת+��ת�ӿ�
    int __declspec(dllexport) rotate_flip_RGB(
        unsigned char* src,          //�����ɫͼ
        unsigned char* dst,          //�����ת+��ת��Ĳ�ɫͼ
        int width,                   //����ͼ��� 768
        int height                   //����ͼ��� 480
    );
    //������ͼ�Ƿ����
    //����ֵ ����������ƽ������ֵ
    int check_expo_ir(
        unsigned char* src, //�������ͼ 
        faceRectIn rect,    //����������
        int width,          //����ͼȫͼ���
        int height          //����ͼȫͼ�߶�
    );

    //���ͼ�����ӿ�
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

