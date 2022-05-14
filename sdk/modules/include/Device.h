#pragma once

#include <mutex>
#include <thread>
#include <time.h>
#include "deviceDataDef.h"
#include "as_datadef.h"
#include "Algo.h"
#include "videoInput.h"
#include "libusb.h"
class Device
{
public:
    Device();
    ~Device();
    int initDevice(u_int8_t port);
    int deinitDevice();

    int openDevice(videoInput &videoinput);
    int closeDevice(videoInput &videoinput);
    int openStream(videoInput &videoinput);
    int closeStream(videoInput &videoinput);
    int readFrame(Frame* frame, videoInput &videoinput);
    int registerNewFrameCallback(NewFrameCallback callback);
    int unregisterNewFrameCallback();

    int registerNewFrameCloudDotCallback(NewFrameCloudDotCallBack callback);
    int unregisterNewFrameCloudDot();

    int getSN(std::string& SN);
    int getIrRgbParameter(IrRgbParameter* irRgbParameter);
    int convertDepthToCloudDot(Frame frame, float* cloudDot);
    void setDeviceResolution(DeviceResolution& Res);
    void setDeviceID(int deviceID)
    {
        this->deviceID = deviceID;
    };
    int getDeviceID()
    {
        return this->deviceID;
    };
    void setHandle(libusb_device* Handle)
    {
        this->handle = Handle;
    };
    void setRotateAngle(eAngle angle);
    void setDownSample(Zoom zoom);
    int setPwmDuty(int duty);
    int modifyFPS(AS_FPS_E enFps);
    uint8_t getDevport();
    int setCamCfg(const AS_CAM_CFG_S* pstCamCfg);
    int getCamCfg(AS_CAM_CFG_S* pstCamCfg);
    int getCameraType(u_int8_t port, AS_CAMERA_TYPE_E* penType);
    int setCameraType(AS_STREAM_OPTION type);
    int setCameraGain(int amplification);
    int upgradeHimax(char* firmwareData, int length);
    int resetHimax();
private:
    u_int8_t m_port;
    struct libusb_device* handle;
    ComEqType type;
    int deviceID;
    Zoom downSampleZoom;
    Algo m_algo;
    AS_CAM_CFG_S m_stCamCfg;
    AS_STREAM_OPTION_E _enOpt;
    std::mutex mutexRotateAngle;
    eAngle angle;

    /* device info */
    std::string m_SN;
    bool irRgbParamFlag;
    bool downSampleFlag;
   

    /**
     * @brief usb node
     *
     */
     // USB USBDeviceHandle;

    //static v4l2_buf_type m_v4l2BuffType;
    //V4L2Buffer m_v4l2Buffer[V4L2MMAP_NBBUFFER];

    DeviceResolution devRes;

    /*
        size_t m_width = 400;
        size_t m_height = 640;
    */

    IrRgbParameter m_irRgbParameter;

    /* frame thread */
    std::mutex m_mutexBuildStream;
    std::thread m_threadBuildStream;
    std::thread m_threadTemperatureCompensation;
    bool m_isTemperatureCompensationRunning = false;
    bool m_isRunning = false;

    /* new frame callback */

    std::mutex m_mutexFrameCallback;
    NewFrameCallback m_newFrameCallback;

    std::mutex m_mutexFrameCloudDotCallback;
    NewFrameCloudDotCallBack m_newFrameCloudDotCallback;
    std::mutex m_mutexDepthCloudDotSync;

    Frame m_frame;
    CloudDot cloudDot;

    AS_CAMERA_TYPE_E m_enCamera;

    int buildFrame(videoInput& videoinput);
    int buildCloudDotStream();
    bool isReadable(videoInput& videoinput);
    int FrameInit(Frame* frame);
    int FrameDeinit(Frame* frame);
    int getSnByUsbCmd(u_int8_t port);
    int getIrRgbParameterByUsbPort(uint8_t port);
    int setDefaultIrRgbParameter();
    bool isIrRgbParameterValid();
    int rotate(Frame* frame);
    int downSample(Zoom zoom);
    int roi(Frame* frame, int x, int width, int y, int height);
    int setBorder(Frame* frame, int width, int height);
    int setTemperatureCompensation(void);
    int resetStream(void);
    int setStreamOption(AS_STREAM_OPTION_E enOpt);
    
    int getPwmDuty(u_int8_t port, int* duty);
    int setLedTime(int time);
    int getLedTime(u_int8_t port, int* time);
};