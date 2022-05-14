#include <mutex>
#include <algorithm>
#include <map>
#include <io.h>

#include "Capture.h"
#include "Logger.h"
#include "Device.h"
#include "UsbControl.h"
#include "libusb.h"

//#define MAX_SUPPORT_DEVICE_NUMBER 10
#define SDK_SOFTWARE_VERSION "v1.0.0.20220422"

#define CHECK_DEVICE_ID(deviceID, m_deviceIDs)                              \
        if( std::find(m_deviceIDs.begin(),m_deviceIDs.end(),deviceID) == m_deviceIDs.end() ){\
        LOG(ERROR) << "deviceID is not cerrect!\n";                         \
        return -1;                                                       \
    }


//Depth stream callback
static std::mutex m_mutexAngNewFrameDepthCallback;
static NewFrameCallback s_newFrameDepthCallback;

//CloudDot stream callback
static std::mutex m_mutexAngNewFrameCloudDotCallback;
static NewFrameCloudDotCallBack s_newFrameCloudDotCallback;

static std::map<int, AS_DEPTHIMAGE_Callback> m_DepthImageCallbackMap;

static std::map<int, AS_POINTCLOUD_Callback> m_PointCloudCallbackMap;

// modify camera resolution
/*
    [0][1]width height
*/
static DeviceResolution angRes[AS_CAM_BUTT] = {
    {640 ,400},
    {400, 640},
    {480, 640},
};

/*USB CONFIGURATION*/
/*
    [0]pid 
    [1]vid 
    [2]ep_in_addr
    [3]ep_out_addr;
 */
static usb_config s_stUsbConfig[] = {
    {0x5722, 0x1A86, 0x82, 0x02},
    {0x5723, 0x1A86, 0x82, 0x02},
    {0x5724, 0x1A86, 0x82, 0x02},
    {0x5723, 0x3482, 0x82, 0x02},
    {0x5724, 0x3482, 0x82, 0x02},
    {0x5725, 0x3482, 0x82, 0x02},
};

/*VIDEO INPUT*/
static videoInput s_videoinput;



static void onDepthImageCallback(int deviceID, AS_DEPTHIMAGE_Data* frame)
{
    m_mutexAngNewFrameDepthCallback.lock();
    std::map<int, AS_DEPTHIMAGE_Callback>::iterator iter = m_DepthImageCallbackMap.find(deviceID);
    if (iter != m_DepthImageCallbackMap.end()) {
        if (iter->second.callback != nullptr) {
            iter->second.callback(deviceID, (const AS_DEPTHIMAGE_Data *)frame, iter->second.privateData);
        }
    }
    m_mutexAngNewFrameDepthCallback.unlock();
}


static void onNewFramePointCloudCallback(int deviceID, AS_POINTCLOUD_Data* pstPointCloud)
{
    m_mutexAngNewFrameCloudDotCallback.lock();
    std::map<int, AS_POINTCLOUD_Callback>::iterator iter = m_PointCloudCallbackMap.find(deviceID);
    if (iter != m_PointCloudCallbackMap.end()) {
        if (iter->second.callback != nullptr) {
            iter->second.callback(deviceID, (const AS_DEPTHIMAGE_Data *)pstPointCloud, iter->second.privateData);
        }
    }
    m_mutexAngNewFrameCloudDotCallback.unlock();
}




Capture::Capture():m_videoinput(s_videoinput)
{
            
}

//#define GET_DEVICE
int Capture::listDevice(std::vector<int>& deviceIDs)
{
    m_deviceCnt = 0;
    std::vector<std::string> devVideoList;
    std::vector<u_int8_t> ports;
    std::vector<std::pair<std::string, std::string>> vid_pid;
    std::vector<std::string> locationPath;
    vid_pid.push_back(std::pair<std::string, std::string>("2DBB","0300"));
    vid_pid.push_back(std::pair<std::string, std::string>("0C45", "636B"));
    uint8_t path[20];
    //UsbControl::getSingleInstance()->deInitLibUsb();
    if (m_deviceList.size() != 0)
    {
        /*delete the old device*/
        deinitCapture();
    }
    int deviceCnt = videoInput::listDevices(vid_pid, deviceIDs);
    m_videoinput.getDeviceLocationPath(locationPath, vid_pid);
    UsbControl::getSingleInstance()->getUSBDeviceList(ports);
    if (ports.size() != deviceIDs.size())
    {
        LOG(INFO) << "libusb win32 is not equal to  camera num" << std::endl;
        deviceIDs.clear();
        return -1;
    }
    if (ports.empty())
    {
        LOG(INFO) << "Found " << ports.size() << " Nuwa camera" << std::endl;
        deviceIDs.clear();
        return 0;
    }


    for (int index = 0; index < ports.size();  index++)
    {
        devVideoList.push_back(videoInput::getDeviceName(ports[index]));
        LOG(INFO) << "port: "; printf("%d\n", ports[index]);
        LOG(INFO) <<  "locationPath[" << index << "]: " << locationPath.at(index) << std::endl;
        

    }
    /*printf the actual Device but not open and init Device */
    //LOG(INFO) << "find Device num  : " << ports.size() << std::endl;
    
    

    int len = ports.size();
    for (int index = 0; index < len; index++) 
    {
        Device* device = new Device();
        if (device == nullptr) 
        {
            deinitCapture();
            break;
        }
        AS_CAMERA_TYPE_E enCamera;
        device->getCameraType(ports.at(index), &enCamera);     
        device->setDeviceResolution(angRes[enCamera]);
        int deviceID = -1;
        for (int path_index = 0; path_index < locationPath.size(); path_index++)
        {
            std::string path = locationPath[path_index];
            std::string key_word = "USBROOT(0)#USB(" + std::to_string(ports.at(index)) + ")";
            if (path.find(key_word) != std::string::npos) 
            {
                deviceID = deviceIDs[path_index];
                device->setDeviceID(deviceID);

                /*total DeviceID vector for the Capture*/
                m_deviceIDs.push_back(deviceIDs[path_index]);
                break;
            }
        }
 
        if (device->initDevice(ports.at(index)) != 0) 
        {
            LOG(ERROR) << "init device " << m_deviceCnt << " port: " << ports.at(index) << "failed!\n";
            delete device;
            continue;
        }
        LOG(INFO) << "DeviceID: " << deviceID ;
        printf("port: %d\n",ports.at(index));
        m_deviceList.emplace_back(device);
        m_deviceCnt++;
    }

    

    LOG(INFO) << "Found " << m_deviceCnt << " Nuwa camera" << std::endl;
    return m_deviceCnt;
}

int Capture::initCapture()
{

    return 0;
}

int Capture::deinitCapture()
{
    /* free m_deviceList */
    for (int i = 0, cnt = m_deviceList.size(); i < cnt; i++) {
        m_deviceList[i]->closeDevice(m_videoinput);
        delete m_deviceList[i];
    }
    m_deviceList.resize(0);
    m_deviceCnt = 0;
    return 0;
}

int Capture::openDevice(int deviceID)
{
    CHECK_DEVICE_ID(deviceID, m_deviceIDs);
    for (int index = 0; index < m_deviceList.size(); index++)
    {
        if (m_deviceList[index]->getDeviceID() == deviceID)
        {
            if (m_deviceList.at(index)->openDevice(m_videoinput) != 0) {
                LOG(ERROR) << "open device failed!\n";
                return -2;
            }
            break;
        }
    }  
    return 0;
}

int Capture::closeDevice(int deviceID)
{
    CHECK_DEVICE_ID(deviceID, m_deviceIDs);

    for (int index = 0; index < m_deviceList.size(); index++)
    {
        if (m_deviceList[index]->getDeviceID() == deviceID)
        {
            m_deviceList.at(index)->closeDevice(m_videoinput);
            break;
        }
    }
    

    return 0;
}

int Capture::openStream(int deviceID)
{
    CHECK_DEVICE_ID(deviceID, m_deviceIDs);

    for (int index = 0; index < m_deviceList.size(); index++)
    {
        if (m_deviceList[index]->getDeviceID() == deviceID)
        {
            if (m_deviceList.at(index)->openStream(m_videoinput) != 0)
            {
                LOG(ERROR) << "deviceID " << deviceID << " open staream failed!\n"
                    << "\n";
                return -2;
            }
            break;
        }
    }

    return 0;
}

int Capture::closeStream(int deviceID)
{
    CHECK_DEVICE_ID(deviceID, m_deviceIDs);
    for (int index = 0; index < m_deviceList.size(); index++)
    {
        if (m_deviceList[index]->getDeviceID() == deviceID)
        {
            m_deviceList.at(index)->closeStream(m_videoinput);
            break;
        }
    }
    

    return 0;
}

int Capture::setRotateAngle(int deviceID, eAngle angle)
{
    CHECK_DEVICE_ID(deviceID, m_deviceIDs);
    for (int index = 0; index < m_deviceList.size(); index++)
    {
        if (m_deviceList[index]->getDeviceID() == deviceID)
        {
            m_deviceList.at(index)->setRotateAngle(angle);
            break;
        }
    }
    

    return 0;
}


int Capture::readFrame(int deviceID, AngFrame* angFrame)
{

    CHECK_DEVICE_ID(deviceID, m_deviceIDs);

    Frame frame;
    for (int index = 0; index < m_deviceList.size(); index++)
    {
        if (m_deviceList[index]->getDeviceID() == deviceID)
        {
            if (m_deviceList.at(index)->readFrame(&frame, m_videoinput) != 0) {
                LOG(ERROR) << "Device " << deviceID << " read stream failed!\n";
                return -3;
            }
            break;
        }
    }

   
    angFrame->width = frame.width;
    angFrame->height = frame.height;
    angFrame->size = frame.size;
    angFrame->data = frame.data;

    return 0;
}


int Capture::registerDepthImageCallback(int deviceID, const AS_DEPTHIMAGE_Callback* pstCallback)
{

    CHECK_DEVICE_ID(deviceID, m_deviceIDs);
    AS_DEPTHIMAGE_Callback stDepthImageCb;
    stDepthImageCb.callback = pstCallback->callback;
    stDepthImageCb.privateData = pstCallback->privateData;
    m_mutexAngNewFrameDepthCallback.lock();
    m_DepthImageCallbackMap.insert(std::pair<int, AS_DEPTHIMAGE_Callback>(deviceID, stDepthImageCb));
    m_mutexAngNewFrameDepthCallback.unlock();
    
    s_newFrameDepthCallback.onNewFrameDepth = onDepthImageCallback;
    for (int index = 0; index < m_deviceList.size(); index++)
    {
        if (m_deviceList[index]->getDeviceID() == deviceID)
        {
            m_deviceList.at(index)->registerNewFrameCallback(s_newFrameDepthCallback);
            break;
        }
    }
    return 0;
}


int Capture::unregisterDepthImageCallback(int deviceID, const AS_DEPTHIMAGE_Callback* pstCallback)
{

    CHECK_DEVICE_ID(deviceID, m_deviceIDs);

    m_mutexAngNewFrameDepthCallback.lock();
    m_DepthImageCallbackMap.erase(deviceID);
    m_mutexAngNewFrameDepthCallback.unlock();
    for (int index = 0; index < m_deviceList.size(); index++)
    {
        if (m_deviceList[index]->getDeviceID() == deviceID)
        {
            m_deviceList.at(index)->unregisterNewFrameCallback();
            break;
        }
    }

    return 0;
}


int Capture::registerPointCloudCallback(int deviceID, const AS_POINTCLOUD_Callback* pstCallback)
{

    CHECK_DEVICE_ID(deviceID, m_deviceIDs);
    AS_POINTCLOUD_Callback stPointCloudCb;
    stPointCloudCb.callback = pstCallback->callback;
    stPointCloudCb.privateData = pstCallback->privateData;
    m_mutexAngNewFrameCloudDotCallback.lock();
    m_PointCloudCallbackMap.insert(std::pair<int, AS_POINTCLOUD_Callback>(deviceID, stPointCloudCb));
    m_mutexAngNewFrameCloudDotCallback.unlock();

    s_newFrameCloudDotCallback.onNewFrameCloudDot = onNewFramePointCloudCallback;
    for (int index = 0; index < m_deviceList.size(); index++)
    {
        if (m_deviceList[index]->getDeviceID() == deviceID)
        {
            m_deviceList.at(index)->registerNewFrameCloudDotCallback(s_newFrameCloudDotCallback);
            break;
        }
    }
    

    return 0;
}


int Capture::unregisterPointCloudCallback(int deviceID, const AS_POINTCLOUD_Callback* pstCallback)
{

    CHECK_DEVICE_ID(deviceID, m_deviceIDs);

    m_mutexAngNewFrameCloudDotCallback.lock();
    m_PointCloudCallbackMap.erase(deviceID);
    m_mutexAngNewFrameCloudDotCallback.unlock(); 
    for (int index = 0; index < m_deviceList.size(); index++)
    {
        if (m_deviceList[index]->getDeviceID() == deviceID)
        {
            m_deviceList.at(index)->unregisterNewFrameCloudDot();
            break;
        }
    }

    return 0;
}

int Capture::getSN(int deviceID, std::string& SN)
{

    CHECK_DEVICE_ID(deviceID, m_deviceIDs);

    for (int index = 0; index < m_deviceList.size(); index++)
    {
        if (m_deviceList[index]->getDeviceID() == deviceID)
        {
            if (m_deviceList[index]->getSN(SN) != 0)
            {
                LOG(ERROR) << "device " << deviceID << " get SN failed!\n";
                return -1;
            }
            break;
        }
    }

    

    return 0;
}

int Capture::getIrRgbParameter(int deviceID, IrRgbParameter* irRgbParameter)
{
    CHECK_DEVICE_ID(deviceID, m_deviceIDs);

    if (irRgbParameter == NULL) {
        LOG(ERROR) << "null ptr!\n";
        return -1;
    }

    for (int index = 0; index < m_deviceList.size(); index++)
    {
        if (m_deviceList[index]->getDeviceID() == deviceID)
        {
            if (m_deviceList[index]->getIrRgbParameter(irRgbParameter) != 0) 
            {
                LOG(ERROR) << "device " << deviceID << " get ir rgb parameter failed!\n";
                return -2;
            }
            break;
        }
    }  

    return 0;
}

int Capture::convertDepthToCloudDot(int deviceID, AngFrame angFrame, float* cloudDot)
{

    CHECK_DEVICE_ID(deviceID, m_deviceIDs);
    int ret = 0;

    Frame frame;
    frame.width = angFrame.width;
    frame.height = angFrame.height;
    frame.size = angFrame.size;
    frame.data = angFrame.data;

    for (int index = 0; index < m_deviceList.size(); index++)
    {
        if (m_deviceList[index]->getDeviceID() == deviceID)
        {
            ret = m_deviceList[index]->convertDepthToCloudDot(frame, cloudDot);
            if (ret < 0) 
            {
                LOG(ERROR) << "device " << deviceID << " convert depth to cloud dot failed!\n";
                return -1;
            }
            break;
        }
    }   

    return ret;

}





int Capture::getSDKSoftwareVersion(std::string& version)
{
    version = SDK_SOFTWARE_VERSION;
    return 0;
}

int Capture::setDownSample(int deviceID, Zoom zoom)
{
    CHECK_DEVICE_ID(deviceID, m_deviceIDs);
    for (int index = 0; index < m_deviceList.size(); index++)
    {
        if (m_deviceList[index]->getDeviceID() == deviceID)
        {
            m_deviceList[index]->setDownSample(zoom);
            break;
        }
    }
    return 0;
}


int Capture::setAsCamCfg(int deviceID, const AS_CAM_CFG_S* pstCamCfg)
{
#
    CHECK_DEVICE_ID(deviceID, m_deviceIDs);
    LOG(INFO) << "set camera cfg: Atc[" << pstCamCfg->isAtcOn << "]" \
        << " Denoises[" << pstCamCfg->isDenoisesOn << "]" \
        << " FillHoles[" << pstCamCfg->isFillHolesOn << "]" \
        << " AntiDistortion[" << pstCamCfg->isAntiDistortion << "]" \
        << std::endl;
    for (int index = 0; index < m_deviceList.size(); index++)
    {
        if (m_deviceList[index]->getDeviceID() == deviceID)
        {
            m_deviceList[index]->setCamCfg(pstCamCfg);
            break;
        }
    }
    return 0;
}

int Capture::getAsCamCfg(int deviceID, AS_CAM_CFG_S* pstCamCfg)
{
    CHECK_DEVICE_ID(deviceID, m_deviceIDs);
    for (int index = 0; index < m_deviceList.size(); index++)
    {
        if (m_deviceList[index]->getDeviceID() == deviceID)
        {
            m_deviceList[index]->getCamCfg(pstCamCfg);
            break;
        }
    }
    return 0;
}

int Capture::setFrameRate(int deviceID, AS_FPS_E enFps)
{
    CHECK_DEVICE_ID(deviceID, m_deviceIDs);
    for (int index = 0; index < m_deviceList.size(); index++)
    {
        if (m_deviceList[index]->getDeviceID() == deviceID)
        {
            if (m_deviceList[index]->modifyFPS(enFps) != 0) {
                LOG(ERROR) << "set frame rate failed.\n";
                return -1;
            }
            break;
        }
    }
    return 0;


}
#if 1
int Capture::setVCSEL(int deviceID, bool enable)
{

    CHECK_DEVICE_ID(deviceID, m_deviceIDs);
    LOG(INFO) << "deviceID: " << deviceID << " enable: " << enable << std::endl;
    int duty = 0;
    for (int index = 0; index < m_deviceList.size(); index++)
    {
        if (m_deviceList[index]->getDeviceID() == deviceID)
        {
            if (enable != false)
            {
                duty = 80;
            }
            else if(enable == false) {
                duty = 0;
            }
            if (m_deviceList[index]->setPwmDuty(duty) != 0) {
                LOG(ERROR) << "set frame rate failed.\n";
                return -1;
            }
            break;
        }
    }

    return 0;
}
#endif  


int Capture::setCameraType(int deviceID, AS_STREAM_OPTION  enOpt)
{
    CHECK_DEVICE_ID(deviceID, m_deviceIDs);
    for (int index = 0; index < m_deviceList.size(); index++)
    {
        if (m_deviceList[index]->getDeviceID() == deviceID)
        {
            if (m_deviceList[index]->setCameraType(enOpt) != 0) {
                LOG(ERROR) << "set type failed.\n";
                return -1;
            }
            break;
        }
    }
}

int Capture::setCameraGain(int deviceID, int amplification)
{
    CHECK_DEVICE_ID(deviceID, m_deviceIDs);
    LOG(INFO) << "deviceID: " << deviceID << " amplification: " << amplification << std::endl;
    for (int index = 0; index < m_deviceList.size(); index++)
    {
        if (m_deviceList[index]->getDeviceID() == deviceID)
        {
            if (m_deviceList[index]->setCameraGain(amplification) != 0) {
                LOG(ERROR) << "set type failed.\n";
                return -1;
            }
            break;
        }
    }
    return 0;
}

int Capture::upgradeHimax(int deviceID, char* firmwareData, int length)
{
    CHECK_DEVICE_ID(deviceID, m_deviceIDs);
    LOG(INFO) << "deviceID: " << deviceID << " length: " << length << std::endl;
    for (int index = 0; index < m_deviceList.size(); index++)
    {
        if (m_deviceList[index]->getDeviceID() == deviceID)
        {
            if (m_deviceList[index]->upgradeHimax(firmwareData, length) != 0)
            {
                LOG(ERROR) << "upgrade Hiamx failed.\n";
                return -1;
            }
            break;
        }
    }
    return 0;
}

int Capture::resetHimax(int deviceID)
{
    CHECK_DEVICE_ID(deviceID, m_deviceIDs);
    LOG(INFO) << "deviceID: " << deviceID << " resetHimax " << std::endl;
    for (int index = 0; index < m_deviceList.size(); index++)
    {
        if (m_deviceList[index]->getDeviceID() == deviceID)
        {
            if (m_deviceList[index]->resetHimax() != 0)
            {
                LOG(ERROR) << "resete Hiamx failed.\n";
                return -1;
            }
            break;
        }
    }
    return 0;
}