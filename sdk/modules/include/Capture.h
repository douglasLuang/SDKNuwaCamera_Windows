/**
 * @file      Capture.h
 * @brief     angstrong NUWA camera sdk interface
 *
 * Copyright (c) 2022 Angstrong Tech.Co.,Ltd
 *
 * @author    Angstrong SDK develop Team
 * @date      2022/03/11
 * @version   1.0

 */
#pragma once

#include <memory>
#include <vector>
#include <string>

#include "deviceDataDef.h"

#define DLL1_EXPORTS
#ifdef DLL1_EXPORTS
#define DLL1_API __declspec(dllexport)
#else
#define DLL1_API __declspec(dllimport)
#endif

class Device;
class videoInput;
typedef Frame AngFrame;
typedef AngFrame AS_DEPTHIMAGE_Data;


typedef void(*AS_DepthImageCallback)(int deviceId, const AS_DEPTHIMAGE_Data* pstDepthImage, void *privateData);
typedef void(*AS_PointCloudCallback)(int deviceId, const AS_DEPTHIMAGE_Data* pstPointCloud, void *privateData);

typedef struct DEPTHIMAGE_CALLBACK_S {
    AS_DepthImageCallback callback;
    void *privateData;
} AS_DEPTHIMAGE_Callback;

typedef struct POINTCLOUD_CALLBACK_S {
    AS_PointCloudCallback callback;
    void *privateData;
} AS_POINTCLOUD_Callback;

class Capture
{
public:
    /**
     * registerDepthFrameCallback is deprecated and should not be used.
     **/
     DLL1_API int   registerDepthImageCallback(int deviceID, const AS_DEPTHIMAGE_Callback *pstCallback);
    /**
     * unregisterDepthFrameCallback is deprecated and should not be used.
     **/
    DLL1_API int unregisterDepthImageCallback(int deviceID, const AS_DEPTHIMAGE_Callback *pstCallback);

    /**
     * registerCloudDotFrameCallback is deprecated and should not be used.
     **/
    DLL1_API int registerPointCloudCallback(int deviceID, const AS_POINTCLOUD_Callback *pstCallback);
    /**
     * unregisterCloudDotFrameCallback is deprecated and should not be used.
     **/
    DLL1_API int unregisterPointCloudCallback(int deviceID, const AS_POINTCLOUD_Callback *pstCallback);

    DLL1_API Capture();
    DLL1_API int listDevice(std::vector<int>& deviceIDs);
    DLL1_API int initCapture();
    DLL1_API int deinitCapture();
    DLL1_API int openDevice(int deviceID);
    DLL1_API int closeDevice(int deviceID);
    DLL1_API int openStream(int deviceID);
    DLL1_API int closeStream(int deviceID);
    DLL1_API int readFrame(int deviceID, AngFrame *angFrame);
    DLL1_API int getSDKSoftwareVersion(std::string &version);
    DLL1_API int getSN(int deviceID, std::string &SN);
    DLL1_API int getIrRgbParameter(int deviceID, IrRgbParameter *irRgbParameter);
    DLL1_API int setDownSample(int deviceID, Zoom zoom);
    //int getCloudDot(int deviceID,std::vector<float> &cloudDot);

    DLL1_API int convertDepthToCloudDot(int deviceID, AngFrame angFrame, float *cloudDot);
    DLL1_API int setRotateAngle(int deviceID, eAngle rotate);

    DLL1_API int setVCSEL(int deviceID, bool enable);
    DLL1_API int setCameraType(int deviceID, AS_STREAM_OPTION  enOpt);
    DLL1_API int setCameraGain(int deviceID, int amplification);
    DLL1_API int upgradeHimax(int deviceID, char* firmwareData, int length);
    DLL1_API int resetHimax(int deviceID);
    /**
     * @brief     frame rate setting
     * @param[in]deviceID : device id([0,listDevice))
     * @param[in]enFps : frame rate
     * @return    0 success,non-zero error code.
     * @exception None
     * @author    Angstrong SDK develop Team
     * @date      2022/03/11
     */
    DLL1_API int setFrameRate(int deviceID, AS_FPS_E enFps);

    DLL1_API int setAsCamCfg(int deviceID, const AS_CAM_CFG_S *pstCamCfg);
    DLL1_API int getAsCamCfg(int deviceID, AS_CAM_CFG_S *pstCamCfg);

    std::vector<int> m_deviceIDs;
private:
    int m_deviceCnt = 0;
    videoInput &m_videoinput;
    std::vector<Device *> m_deviceList;

};