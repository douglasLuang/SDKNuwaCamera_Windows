#pragma once

#include <vector>
#include <string>

typedef  unsigned short ushort;

typedef	unsigned char u_int8_t;
typedef	unsigned short int u_int16_t;
typedef	unsigned int u_int32_t;
typedef struct {
    float irFx;
    float irFy;
    float irCx;
    float irCy;

    float rgbFx;
    float rgbFy;
    float rgbCx;
    float rgbCy;

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
} IrRgbParameter;


typedef enum {
    DEFAULT = 0,
    X2 = 2,
} Zoom;


typedef enum {
    ANGLE_0,
    CLOCKWISE_90,
    CLOCKWISE_180,
    CLOCKWISE_270,
    ANTICLOCKWISE_90,
    ANTICLOCKWISE_180,
    ANTICLOCKWISE_270,
} eAngle;


typedef struct {
    size_t width;
    size_t height;
    void* data;
    size_t size;
    int type;
} Frame;

typedef struct {
    /*
    * 0 ----- X
    * 1 ----- Y
    * 2 ----- Z
    */
    float* data;
    size_t length;
} CloudDot;

typedef CloudDot AS_POINTCLOUD_Data;

typedef struct {
    void (*onNewFrameDepth)(int deviceID, Frame* frame);

} NewFrameCallback;

typedef struct {
    void (*onNewFrameCloudDot)(int deviceID, CloudDot* cloudDot);
} NewFrameCloudDotCallBack;

typedef struct {
    size_t t_width;
    size_t t_height;
} DeviceResolution;



typedef enum ComEqType {
    _NONE = 0,
    _USB = 1,
    _TTYACM,
} AS_COM_TYPE_E;


typedef struct {
    unsigned short pid;
    unsigned short vid;
    unsigned short ep_in_addr;
    unsigned short ep_out_addr;
} usb_config;

enum UPDATE {
    _UPDATE_MCU = 1,
};

typedef enum asFPS_S {
    AS_FPS_30 = 1,
    AS_FPS_25,
    AS_FPS_20,
    AS_FPS_15,
    AS_FPS_10,
    AS_FPS_5,
    AS_FPS_8,
    AS_FPS_BUTT
} AS_FPS_E;

typedef struct asCAM_CFG_S {
    /*Automatic Temperature Compensation*/
    bool isAtcOn;
    /*Denoises for depth image*/
    bool isDenoisesOn;
    /*Fill hole for depth image*/
    bool isFillHolesOn;
    /*Anti distortion for depth image*/
    bool isAntiDistortion;
} AS_CAM_CFG_S;

typedef enum asSTREAM_OPTION {
    AS_STREAM_DOT = 2,
    AS_STREAM_DEPTH_IMAGE,
} AS_STREAM_OPTION;