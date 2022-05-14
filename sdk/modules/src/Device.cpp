#include <iostream>
#include <fstream>
#include <windows.h>
#include "VideoControl.h"
#include "Device.h"
#include "Logger.h"
#include "common.h"
#include "UsbControl.h"

/*multi thread*/
#define THREAD

/* set static device info */
/*
std::string Device::m_uartVid = "1a86";
std::string Device::m_uartPid = "5722";
std::string Device::m_ucameraVid = "2dbb";
std::string Device::m_ucameraPid = "0300";
*/


Device::Device()
    : type(ComEqType::_NONE),
    deviceID(-1),
    downSampleZoom(DEFAULT),
    angle(ANGLE_0),
    irRgbParamFlag(true),
    downSampleFlag(false)
{
    memset(&m_stCamCfg, 0, sizeof(AS_CAM_CFG_S));
    m_stCamCfg.isAtcOn = true;
    m_stCamCfg.isDenoisesOn = true;
    m_stCamCfg.isFillHolesOn = true;
    m_stCamCfg.isAntiDistortion = false;
}

Device::~Device() {}

int Device::initDevice(u_int8_t port)
{
    getCameraType(port, &m_enCamera);
    UsbControl::getSingleInstance()->closePort(port);
    if (UsbControl::getSingleInstance()->openPort(port) != 0)
    {
        return 0;
    }
    
    if (FrameInit(&m_frame) != 0) 
    {
        LOG(ERROR) << "malloc memory failed" << std::endl;
        return -1;
    }

    memset(&m_newFrameCallback, 0, sizeof(NewFrameCallback));
    memset(&m_newFrameCloudDotCallback, 0, sizeof(NewFrameCloudDotCallBack));


    if (getIrRgbParameterByUsbPort(port) != 0) 
    {
        LOG(ERROR) << "get ir and rgb parameter failed!\n";
        return -2;
    }
    if (isIrRgbParameterValid() != true) 
    {
        LOG(WARN) << "ir and rgb paremeter is not valid!\n";
        LOG(WARN) << "try to set default ir and rgb parameter!\n";
        setDefaultIrRgbParameter();
    }

    if (getSnByUsbCmd(port) != 0) 
    {
        LOG(ERROR) << "get SN failed!\n";
        return -3;
    }
    this->m_port = port;
    type = AS_COM_TYPE_E::_USB;

    return 0;
}

int Device::deinitDevice()
{
    FrameDeinit(&m_frame);

    return 0;
}

int Device::openDevice(videoInput &videoinput)
{
    LOG(INFO) <<  "deviceID: " << deviceID << std::endl;
    if (init(deviceID, &devRes, videoinput) != 0) {
        LOG(ERROR) << "init device failed!\n";
        return -1;
    }
    return 0;
}

int Device::closeDevice(videoInput &videoinput)
{
    if (deviceID >= 0) 
    {      
        closeStream(videoinput);
        videoinput.stopDevice(deviceID);
        //deviceID = -1;
    }
    return 0;
}

int Device::openStream(videoInput &videoinput)
{
    m_mutexBuildStream.lock();
    if (m_isRunning == true) {
        m_mutexBuildStream.unlock();
        return 0;
    }
    m_mutexBuildStream.unlock();

    //setPwmDuty(80);
    if (videoinput.isDeviceSetup(deviceID) != true) {
        return -1;
    }
    
    setStreamOption(AS_STREAM_OPTION_DEPTH_IMAGE);
    m_isRunning = true;
    if (m_newFrameCallback.onNewFrameDepth || m_newFrameCloudDotCallback.onNewFrameCloudDot) {
        LOG(INFO) << "create pthrad\n";
        m_threadBuildStream = std::thread(&Device::buildFrame, this, std::ref(videoinput));
    }
    return 0;
}

int Device::closeStream(videoInput &videoinput)
{
    m_mutexBuildStream.lock();
    if (m_isRunning == false) {
        m_mutexBuildStream.unlock();
        return 0;
    }
    m_mutexBuildStream.unlock();

    if (videoinput.isDeviceSetup(deviceID) != true) {
        return -1;
    }
    LOG(INFO) << " thread close\n" ;
    setStreamOption(AS_STREAM_OPTION_OFF);
    m_isRunning = false;
    m_isTemperatureCompensationRunning = false;
    if (m_threadBuildStream.joinable())
    {
        m_threadBuildStream.join();
    }
    if (m_threadTemperatureCompensation.joinable())
    {
        m_threadTemperatureCompensation.join();        
    }
    return 0;
}

#define RATIO(frm)               \
    frm->width ^= frm->height; \
    frm->height ^= frm->width; \
    frm->width ^= frm->height;

static int rotateANTI90(Frame* frame)
{
    int count = 0;
    unsigned short* src = (unsigned short*)frame->data;
    unsigned short* dest = (unsigned short*)malloc(frame->size);
    for (int x = frame->width - 1; x >= 0; x--) {
        for (size_t y = 1; y <= frame->height; y++) {
            *(dest + count) = *(src + (frame->width * (y - 1)) + x);
            count++;
        }
    }
    memcpy(frame->data, dest, frame->size);
    RATIO(frame);
    free(dest);
    return 0;
}


int Device::readFrame(Frame* frame, videoInput &videoinput)
{ 
    if (isReadable(videoinput) != true) {
        LOG(INFO) << "devcie " << deviceID << " is not ready!\n";
        return -1;
    }

    m_frame.width = devRes.t_width;
    m_frame.height = devRes.t_height;
    m_frame.size = devRes.t_width * devRes.t_height * 2;

    if (videoinput.isFrameNew(deviceID) != true)
    {
        Sleep(30);
        return -3;
    }
    if (videoinput.getPixels(deviceID, (unsigned char*)m_frame.data, false, false) != true)
    {
        
        LOG(INFO) << "get Pixels failed, try again!\n";
    }
    //m_frame.size = m_frame.height * m_frame.width *2;
    if (m_enCamera == AS_CAM_NUWA_X100) {
        rotateANTI90(&m_frame);
    }

    if (!m_isTemperatureCompensationRunning)
    {
        m_isTemperatureCompensationRunning = true;
        m_threadTemperatureCompensation = std::thread(&Device::setTemperatureCompensation, this);
    }


    if (_enOpt == AS_STREAM_OPTION_DOT)
    {
        unsigned short* data_before = (unsigned short*)m_frame.data;
        unsigned char* temp = new unsigned char[m_frame.width * m_frame.height];
        for (int i = 0; i < m_frame.height * m_frame.width; i++)
        {
            temp[i] = (unsigned char)(data_before[i] >> 8);
        }
        memset(m_frame.data, 0, m_frame.width * m_frame.height * 2);
        memcpy(m_frame.data, temp, m_frame.width * m_frame.height);
        m_frame.size = m_frame.width * m_frame.height;

        delete[]temp;
        m_frame.type = AS_STREAM_OPTION_DOT;
    }

   


    frame->width = m_frame.width;
    frame->height = m_frame.height;
    frame->size = m_frame.size;
    frame->data = m_frame.data;
    if (_enOpt == AS_STREAM_OPTION_DEPTH_IMAGE)
    {
        m_algo.processDepthRawData((unsigned char*)frame->data, frame->height, frame->width);
        if (m_stCamCfg.isDenoisesOn) {
            m_algo.denoises((unsigned char*)frame->data, frame->width, frame->height, 0, 800, 10, 4000);
        }
        if (m_stCamCfg.isFillHolesOn) {
            m_algo.fillHoles((unsigned char*)frame->data, frame->width, frame->height, 1000, 10);
        }
        downSample(downSampleZoom);   
        if (m_stCamCfg.isAntiDistortion) {
            setBorder(&m_frame, m_frame.width * 0.03, m_frame.height * 0.05);
        }
        m_frame.type = AS_STREAM_OPTION_DEPTH_IMAGE;
        rotate(&m_frame);
    }
    return 0;
}

int Device::registerNewFrameCallback(NewFrameCallback callback)
{
    m_mutexFrameCallback.lock();
    memcpy(&m_newFrameCallback, &callback, sizeof(NewFrameCallback));
    m_mutexFrameCallback.unlock();

    return 0;
}

int Device::unregisterNewFrameCallback()
{
    m_mutexFrameCallback.lock();
    memset(&m_newFrameCallback, 0, sizeof(NewFrameCallback));
    m_mutexFrameCallback.unlock();

    return 0;
}

int Device::registerNewFrameCloudDotCallback(NewFrameCloudDotCallBack callback)
{
    m_mutexFrameCloudDotCallback.lock();
    memcpy(&m_newFrameCloudDotCallback, &callback, sizeof(NewFrameCloudDotCallBack));
    m_mutexFrameCloudDotCallback.unlock();

    return 0;
}

int Device::unregisterNewFrameCloudDot()
{
    m_mutexFrameCloudDotCallback.lock();
    memset(&m_newFrameCloudDotCallback, 0, sizeof(NewFrameCloudDotCallBack));
    m_mutexFrameCloudDotCallback.unlock();

    return 0;
}

#if 1
#if 0
int Device::buildCloudDotStream()
{
    LOG(INFO) << "------------build CloudDot Stream--------------";
    while (true) {
        m_mutexBuildStream.lock();
        if (m_isRunning == false) {
            m_mutexBuildStream.unlock();
            break;
        }
        m_mutexBuildStream.unlock();
        m_mutexDepthCloudDotSync.lock();
        //      cloudDot.clear();
        if (convertDepthToCloudDot(m_frame, cloudDot) != 0) {
            LOG(ERROR) << "Convert CloudDot fail!\n";
            continue;
        }
        m_mutexDepthCloudDotSync.unlock();
        m_mutexFrameCloudDotCallback.lock();
        if (m_newFrameCloudDotCallback.onNewFrameCloudDot != NULL) {
            LOG(INFO) << "CloudDotCallback ";
            m_newFrameCloudDotCallback.onNewFrameCloudDot(deviceID, cloudDot);
        }
        m_mutexFrameCloudDotCallback.unlock();
    }
}
#endif

long long GetSysCurrentTime()
{
    SYSTEMTIME sys;
    GetLocalTime(&sys);
    long long curTime = ((long long)(sys.wSecond)) * 1000 + sys.wMilliseconds / 1000;
    return curTime;
}

int Device::buildFrame(videoInput &videoinput)
{
    int ret = 0;
#ifdef THREAD
    while (true) {
        /* check if it is still running */
        m_mutexBuildStream.lock();
        if (m_isRunning == false) {
            m_mutexBuildStream.unlock();
            break;
        }
        m_mutexBuildStream.unlock();       
        ret = readFrame(&m_frame, videoinput);
        if (ret != 0) {
            continue;
        }
        m_mutexFrameCallback.lock();
        if (m_newFrameCallback.onNewFrameDepth != NULL) {
            m_newFrameCallback.onNewFrameDepth(deviceID, &m_frame);
        }
        m_mutexFrameCallback.unlock();

        m_mutexFrameCloudDotCallback.lock();
        if (m_newFrameCloudDotCallback.onNewFrameCloudDot != NULL) {
            if (m_frame.type != AS_STREAM_OPTION_DOT)
            {
                float* arry = new float[m_frame.width * m_frame.height * 3];
                cloudDot.data = arry;
                memset(arry, 0, sizeof(arry));
                cloudDot.length = m_frame.width * m_frame.height * 3;
                cloudDot.length = convertDepthToCloudDot(m_frame, cloudDot.data);
                if (cloudDot.length < 0) {
                    LOG(ERROR) << "Convert CloudDot fail!\n";
                    delete[]arry;
                    continue;
                }
                m_newFrameCloudDotCallback.onNewFrameCloudDot(deviceID, &cloudDot);
                delete[]arry;
            }
        }
        m_mutexFrameCloudDotCallback.unlock();

        Sleep(10);

    }

#endif
    return 0;
}
#endif

int Device::getSN(std::string& SN)
{
    SN = m_SN;
    return 0;
}

uint8_t Device::getDevport()
{
    return this->m_port;
}

int Device::getIrRgbParameter(IrRgbParameter* irRgbParameter)
{
    if (irRgbParameter == NULL) {
        LOG(ERROR) << "null ptr!\n";
        return -1;
    }

    memcpy(irRgbParameter, &m_irRgbParameter, sizeof(IrRgbParameter));
    return 0;
}

int Device::convertDepthToCloudDot(Frame frame, float* cloudDot)
{
    int ret = 0;
    ret = m_algo.depthToCloudDot((unsigned short*)frame.data, frame.width, frame.height, m_irRgbParameter, cloudDot);
    if (ret < 0) {
        LOG(ERROR) << "convert depth to cloud dot failed!\n";
        return -1;
    }
    return ret;
}

void Device::setDownSample(Zoom zoom)
{
    this->downSampleZoom = zoom;
}

void Device::setDeviceResolution(DeviceResolution& Res)
{
    this->devRes = Res;
}

void Device::setRotateAngle(eAngle angle)
{
    mutexRotateAngle.lock();
    this->angle = angle;
    mutexRotateAngle.unlock();
}

bool Device::isReadable(videoInput &videoinput)
{
    return videoinput.isDeviceSetup(deviceID);
}

int Device::FrameInit(Frame* frame)
{
    if (frame == NULL)
        return -1;

    frame->width = devRes.t_width;
    frame->height = devRes.t_height;
    frame->size = frame->width * frame->height * 2;
    frame->data = malloc(frame->size);
    if (frame->data == NULL)
        return -1;

    return 0;
}

int Device::FrameDeinit(Frame* frame)
{
    if (frame == NULL)
        return 0;

    if (frame->data != NULL) {
        free(frame->data);
        frame->data = NULL;
    }

    free(frame);
    frame = NULL;

    return 0;
}

int Device::setDefaultIrRgbParameter()
{
    m_irRgbParameter.irFx = 425;
    m_irRgbParameter.irFy = 425;
    m_irRgbParameter.irCx = 320;
    m_irRgbParameter.irCy = 240;

    return 0;
}

int Device::getSnByUsbCmd(u_int8_t port)
{
    UsbControl* instance = UsbControl::getSingleInstance();

    std::string sn;
    if (instance == nullptr) {
        LOG(ERROR) << "get usb instance failed" << std::endl;
        return -1;
    }
    else if (instance->getSerialNumber(port, sn) != 0) {
        LOG(ERROR) << "get sn filed.\n";
        return -1;
    }
    m_SN = sn;
    return 0;
}

int Device::getIrRgbParameterByUsbPort(uint8_t port)
{
    char chs[256];
    int chs_num[128] = { 0 };
    char ch_temp[4] = { 0 };
    memset(chs, 0, sizeof(chs));
    if (UsbControl::getSingleInstance()->readInternalParamater(port, chs) != 0) {
        LOG(ERROR) << "read interal paramater filed.\n";
        return -1;
    }
    for (int i = 0; i < 128; i += 4)
    {
        memset(ch_temp, 0, sizeof(ch_temp));
        memcpy(ch_temp, &chs[i + 2], 2);
        memcpy(ch_temp + 2, &chs[i], 2);
        std::string temp = std::string(ch_temp);
        //sscanf(temp.c_str(), "%x", chs_num[i / 4]);
        chs_num[i / 4] = strtol(temp.c_str(), 0, 16);
        //chs_num[i / 4] = stoi(temp, 0, 16);
    }
    memset(&m_irRgbParameter, 0, sizeof(IrRgbParameter));
    m_irRgbParameter.irFx = float(chs_num[0]) * 1.0;//para[0];
    m_irRgbParameter.irFy = float(chs_num[1]) * 1.0;//para[1];
    m_irRgbParameter.irCy = float(chs_num[3]) * 1.0;//((float)para[2] / 2.6) * 2;
    m_irRgbParameter.irCx = float(chs_num[2]) * 1.0;//(para[2] - m_irRgbParameter.irCy / 2) * 2;

    m_irRgbParameter.irFx /= 2.0;
    m_irRgbParameter.irFy /= 2.0;
    m_irRgbParameter.irCy /= 2.0;
    m_irRgbParameter.irCx /= 2.0;
    // printf("fx:%lf,fy:%lf,cy:%lf,cx:%lf\n",m_irRgbParameter.irFx,m_irRgbParameter.irFy,m_irRgbParameter.irCx,m_irRgbParameter.irCy);

    return 0;
}

bool Device::isIrRgbParameterValid()
{
    if (m_irRgbParameter.irFx < 420 ||
        m_irRgbParameter.irFx > 430)
        return false;
    if (m_irRgbParameter.irFy < 420 ||
        m_irRgbParameter.irFy > 430)
        return false;
    if (m_irRgbParameter.irCx < 315 ||
        m_irRgbParameter.irCx > 325)
        return false;
    if (m_irRgbParameter.irCy < 195 + 40 ||
        m_irRgbParameter.irCy > 205 + 40)
        return false;

    return true;
}

#define RATIO(frm)                           \
    frm->width ^= frame->height;             \
    frm->height ^= frame->width;             \
    frm->width ^= frame->height;

#define IRRGBPARA(irRgbParam, flag, TF)                                                                                                       \
    /*LOG(INFO) << "TF " << TF;*/                                                                                                             \
    if(flag == TF) {                                                 \
        irRgbParam.irFx += irRgbParam.irFy;                                                                                                   \
        irRgbParam.irFy = irRgbParam.irFx - irRgbParam.irFy;                                                                                  \
        irRgbParam.irFx = irRgbParam.irFx - irRgbParam.irFy;                                                                                  \
        irRgbParam.irCx += irRgbParam.irCy;                                                                                                   \
        irRgbParam.irCy = irRgbParam.irCx - irRgbParam.irCy;                                                                                  \
        irRgbParam.irCx = irRgbParam.irCx - irRgbParam.irCy;                                                                                  \
        flag = !TF;                                                                                                                           \
        LOG(INFO) << "irFx:" << irRgbParam.irFx << " irFy:" << irRgbParam.irFy << " irCx:" << irRgbParam.irCx << " irCy:" << irRgbParam.irCy; \
    }



int Device::rotate(Frame* frame)
{
    unsigned short* src = NULL;
    unsigned short* dest = NULL;
    eAngle alg;
    int count = 0;

    mutexRotateAngle.lock();
    alg = this->angle;
    mutexRotateAngle.unlock();

    src = (unsigned short*)frame->data;
    dest = (unsigned short*)malloc(frame->size);
    if (dest == nullptr) {
        LOG(ERROR) << "malloc memory for rotate failed" << std::endl;
        return -1;
    }

    switch (alg) {
    case ANGLE_0:

        //  LOG(INFO) << " rotate angle is 0: " << frame->width << " height: " << frame->height;
        IRRGBPARA(m_irRgbParameter, irRgbParamFlag, false);
        break;
    case CLOCKWISE_90:

        for (size_t x = 0; x < frame->width; x++) {
            for (size_t y = frame->height; y > 0; y--) {
                *(dest + count) = *(src + (frame->width * (y - 1)) + x);
                count++;
            }
        }

        memcpy(frame->data, dest, frame->size);
        RATIO(frame);
        //  LOG(INFO) << " After the  clockwise rotate angle 90. width: " << frame->width << " height: " << frame->height;
        IRRGBPARA(m_irRgbParameter, irRgbParamFlag, true);
        break;
    case CLOCKWISE_180:

        for (int y = frame->height - 1; y >= 0; y--) {
            for (int x = frame->width - 1; x >= 0; x--) {
                *(dest + count) = *(src + (y * frame->width) + x);
                count++;
            }
        }

        memcpy(frame->data, dest, frame->size);
        //  LOG(INFO) << " After the  clockwise rotate angle 180. width: " << frame->width << " height: " << frame->height;
        IRRGBPARA(m_irRgbParameter, irRgbParamFlag, false);
        break;
    case CLOCKWISE_270:

        for (size_t x = frame->width - 1; x >= 0; x--) {
            for (size_t y = 1; y <= frame->height; y++) {
                *(dest + count) = *(src + (frame->width * (y - 1)) + x);
                count++;
            }
        }

        memcpy(frame->data, dest, frame->size);
        RATIO(frame);
        IRRGBPARA(m_irRgbParameter, irRgbParamFlag, true);
        //  LOG(INFO) << " After the  clockwise rotate angle 270. width: " << frame->width << " height: " << frame->height;

        break;
    case ANTICLOCKWISE_90:

        for (int x = frame->width - 1; x >= 0; x--) {
            for (size_t y = 1; y <= frame->height; y++) {
                *(dest + count) = *(src + (frame->width * (y - 1)) + x);
                count++;
            }
        }

        memcpy(frame->data, dest, frame->size);
        RATIO(frame);
        IRRGBPARA(m_irRgbParameter, irRgbParamFlag, true);
        //  LOG(INFO) << " After the  anticlockwise rotate angle 90. width: " << frame->width << " height: " << frame->height;

        break;
    case ANTICLOCKWISE_180:

        for (int y = frame->height - 1; y >= 0; y--) {
            for (int x = frame->width - 1; x >= 0; x--) {
                *(dest + count) = *(src + (y * frame->width) + x);
                count++;
            }
        }

        memcpy(frame->data, dest, frame->size);
        //  LOG(INFO) << " After the  anticlockwise rotate angle 180. width: " << frame->width << " height: " << frame->height;

        IRRGBPARA(m_irRgbParameter, irRgbParamFlag, false);
        break;
    case ANTICLOCKWISE_270:

        for (size_t x = 0; x < frame->width; x++) {
            for (size_t y = frame->height; y > 0; y--) {
                *(dest + count) = *(src + (frame->width * (y - 1)) + x);
                count++;
            }
        }

        memcpy(frame->data, dest, frame->size);
        RATIO(frame);
        IRRGBPARA(m_irRgbParameter, irRgbParamFlag, true);
        //  LOG(INFO) << " After the  anticlockwise rotate angle 270. width: " << frame->width << " height: " << frame->height;

        break;
    default:
        break;
    }
    free(dest);
    return 0;
}

int Device::roi(Frame* frame, int start_x, int width, int start_y, int height)
{
    unsigned short* tmp = (unsigned short*)malloc(frame->size);
    if (tmp == nullptr) {
        LOG(ERROR) << "malloc memory for roi failed\n";
        return -1;
    }

    // m_irRgbParameter.irFx -= start_x;
    // m_irRgbParameter.irFy -= start_y;
    // m_irRgbParameter.irCx -= start_x;
    // m_irRgbParameter.irCy -= start_y;

    memcpy(tmp, frame->data, frame->size);
    memset(frame->data, 0, frame->size);
#if 0
    for (int row = 0; row < height; row++) {
        for (int col = 0; col < width; col++) {
            ((unsigned short*)frame->data)[col + row * width] = tmp[start_y * frame->width + start_x + col + row * frame->width];
        }
    }
#else
    for (int row = 0; row < height; row++) {
        memcpy((unsigned short*)frame->data + row * width, tmp + start_y * frame->width + row * frame->width, width * 2);
    }
#endif

    frame->width = width;
    frame->height = height;
    frame->size = frame->width * frame->height * 2;

    if (tmp != nullptr) {
        free(tmp);
        tmp = nullptr;
    }

    return 0;
}

int Device::setBorder(Frame* frame, int width, int height)
{
    memset((unsigned short*)frame->data, 0x00, frame->width * height * 2);
    memset((unsigned short*)frame->data + frame->width * (frame->height - height), 0x00, frame->width * height * 2);
    for (unsigned int row = 0; row < frame->height - height * 2; row++) {
        memset((unsigned short*)frame->data + frame->width * height + row * frame->width, 0x00, width * 2);
        memset((unsigned short*)frame->data + frame->width * height + (row + 1) * frame->width - width, 0x00, width * 2);
    }
    return 0;
}


#define PIXrgb(irRgbParam, flag, TF, op)                                                                                                      \
    /*LOG(INFO) << "TF " << TF;*/                                                                                                             \
    if (flag == TF)                                                                                                                           \
    {                                                                                                                                         \
        irRgbParam.irFy = irRgbParam.irFy;                                                                                                    \
        irRgbParam.irFx #op = irRgbParam.irFx;                                                                                                \
        irRgbParam.irCx #op = irRgbParam.irCx;                                                                                                \
        irRgbParam.irCy #op = irRgbParam.irCy;                                                                                                \
        flag = !TF;                                                                                                                           \
        LOG(INFO) << "irFx:" << irRgbParam.irFx << " irFy:" << irRgbParam.irFy << " irCx:" << irRgbParam.irCx << " irCy:" << irRgbParam.irCy; \
    }

int Device::downSample(Zoom zoom)
{
    int ret = -1;
    /*if(zoom)
    {
        LOG(INFO) << "step = 0 ,down sample inoperative";
        return 0;
    }
    */
    //static unsigned short * dst = (unsigned short *)malloc(m_frame.size);
    if (!zoom) {
        // PIXrgb(m_irRgbParameter,downSampleFlag,true,"/");
        if (downSampleFlag) {
            m_irRgbParameter.irFx *= 2;
            m_irRgbParameter.irFy *= 2;
            m_irRgbParameter.irCx *= 2;
            m_irRgbParameter.irCy *= 2;
            downSampleFlag = false;
            LOG(INFO) << "irFx:" << m_irRgbParameter.irFx << " irFy:" << m_irRgbParameter.irFy << " irCx:" << m_irRgbParameter.irCx
                << " irCy:" << m_irRgbParameter.irCy;
        }

        return 0;
    }
    if (!downSampleFlag) {
        m_irRgbParameter.irFx /= 2;
        m_irRgbParameter.irFy /= 2;
        m_irRgbParameter.irCx /= 2;
        m_irRgbParameter.irCy /= 2;
        downSampleFlag = true;
        LOG(INFO) << "irFx:" << m_irRgbParameter.irFx << " irFy:" << m_irRgbParameter.irFy << " irCx:" << m_irRgbParameter.irCx
            << " irCy:" << m_irRgbParameter.irCy;
    }

    unsigned short* dst = new unsigned short[m_frame.size];
    memset(dst, 0, m_frame.size);
    ret = m_algo.downSample(dst, m_frame.width, m_frame.height, (unsigned short*)m_frame.data, m_frame.width,
        m_frame.height, 1, zoom);
    memcpy(m_frame.data, dst, m_frame.size);
    delete []dst;
    return ret;
}

int Device::setTemperatureCompensation(void)
{
    int ret = 0;
    if (type == ComEqType::_NONE) {
        LOG(ERROR) << "Set the communication device type first.\n";
        return -1;
    }
    else if (type == ComEqType::_USB) {
        if (UsbControl::getSingleInstance()->configAutoTemperatureCompensation(m_port, m_stCamCfg.isAtcOn) != 0) {
            return -1;
        }
    }
    return ret;
}

int Device::resetStream(void)
{
    if (type == ComEqType::_NONE) {
        LOG(ERROR) << "Set the communication device type first.\n";
        return -1;
    }
    else if (type == ComEqType::_USB) {
        if (UsbControl::getSingleInstance()->resetStream(m_port) != 0) {
            return -1;
        }
    }
    return 0;
}

int Device::setStreamOption(AS_STREAM_OPTION_E enOpt)
{
    if (type == AS_COM_TYPE_E::_NONE) 
    {
        LOG(ERROR) << "Set the communication device type first.\n";
        return -1;
    }
    else if (type == AS_COM_TYPE_E::_USB)
    {
        UsbControl::getSingleInstance()->setPwmDuty(m_port, 80);
        if (UsbControl::getSingleInstance()->setStreamType(m_port, enOpt) != 0) 
        {
            LOG(ERROR) << "setStreamType fail!\n";
            return -1;
        }
    }
    _enOpt = enOpt;
    return 0;
}

int Device::setCamCfg(const AS_CAM_CFG_S* pstCamCfg)
{
    memcpy(&m_stCamCfg, pstCamCfg, sizeof(AS_CAM_CFG_S));
    return 0;
}

int Device::getCamCfg(AS_CAM_CFG_S* pstCamCfg)
{
    memcpy(pstCamCfg, &m_stCamCfg, sizeof(AS_CAM_CFG_S));
    return 0;
}

int Device::modifyFPS(AS_FPS_E enFps)
{
    if (type == ComEqType::_NONE) {
        LOG(ERROR) << "Set the communication device type first.\n";
        return -1;
    }
    else if (type == ComEqType::_USB) {
        if (UsbControl::getSingleInstance()->modifyFPS(m_port, enFps) != 0) {
            LOG(ERROR) << "port : " << (int)deviceID << " modify fps failed.\n";
            return -1;
        }
    }
    return 0;
}

int Device::setPwmDuty(int duty)
{
    if (type == AS_COM_TYPE_E::_NONE) {
        LOG(ERROR) << "Set the communication device type first.\n";
        return -1;
    }
    else if (type == AS_COM_TYPE_E::_USB) {
        if (UsbControl::getSingleInstance()->setPwmDuty(m_port, duty) != 0) {
            return -1;
        }
    }
    LOG(INFO) << "DeviceID: " << deviceID;
    printf("port: %d\n", m_port);
    return 0;
}

int Device::getPwmDuty(u_int8_t port, int* duty)
{
    if (type == AS_COM_TYPE_E::_NONE) {
        LOG(ERROR) << "Set the communication device type first.\n";
        return -1;
    }
    else if (type == AS_COM_TYPE_E::_USB) {
        if (UsbControl::getSingleInstance()->getPwmDuty(m_port, duty) != 0) {
            return -1;
        }
    }
    return 0;
}

int Device::setLedTime(int time)
{
    if (type == AS_COM_TYPE_E::_NONE) {
        LOG(ERROR) << "Set the communication device type first.\n";
        return -1;
    }
    else if (type == AS_COM_TYPE_E::_USB) {
        if (UsbControl::getSingleInstance()->setLedTime(m_port, time) != 0) {
            return -1;
        }
    }
    return 0;
}

int Device::getLedTime(u_int8_t port, int* time)
{
    if (type == AS_COM_TYPE_E::_NONE) {
        LOG(ERROR) << "Set the communication device type first.\n";
        return -1;
    }
    else if (type == AS_COM_TYPE_E::_USB) {
        if (UsbControl::getSingleInstance()->getLedTime(m_port, time) != 0) {
            return -1;
        }
    }
    return 0;
}

int Device::getCameraType(u_int8_t port, AS_CAMERA_TYPE_E* penType)
{
    if (UsbControl::getSingleInstance()->getCameraType(port, penType) != 0) {
        return -1;
    }
    return 0;
}

int Device::setCameraType(AS_STREAM_OPTION type)
{
    if (type == AS_STREAM_DOT)
    {
        if (UsbControl::getSingleInstance()->setStreamType(m_port, AS_STREAM_OPTION_DOT) != 0) {
            return -1;
        }
        _enOpt = AS_STREAM_OPTION_DOT;
       //UsbControl::getSingleInstance()->setPwmDuty(m_port, 0);
    }
    else if (type == AS_STREAM_DEPTH_IMAGE)
    {
        if (UsbControl::getSingleInstance()->setStreamType(m_port, AS_STREAM_OPTION_DEPTH_IMAGE) != 0) {
            return -1;
        }
        _enOpt = AS_STREAM_OPTION_DEPTH_IMAGE;
        UsbControl::getSingleInstance()->setPwmDuty(m_port, 80);
    }

    return 0;
}

int Device::setCameraGain(int amplification)
{
    if (UsbControl::getSingleInstance()->setCameraGain(m_port, amplification) != 0) {
        return -1;
    }
    return 0;
}

int Device::upgradeHimax(char* firmwareData, int length)
{
    if (UsbControl::getSingleInstance()->upgradeHimax(m_port, firmwareData,length) != 0) {
        return -1;
    }
    return 0;
}

int Device::resetHimax()
{
    if (UsbControl::getSingleInstance()->resetHimax(m_port) != 0) {
        return -1;
    }
    return 0;
}