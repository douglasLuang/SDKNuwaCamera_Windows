#include "UsbControl.h"
#include "stdbool.h"
#include <algorithm>
// #ifdef __cplusplus
// extern "C" {
// #endif
// #include "wch55xisp.h"
// #ifdef __cplusplus
// }
// #endif
//#include "unistd.h"
#include <mutex>
#include "TmpTable.h"


#ifndef TRUE
#define TRUE 0
#else 
#undef TRUE
#define TRUE 0
#endif

#ifndef FALSE
#define FALSE -1
#undef FALSE
#define FALSE 0
#endif

#define MAX_RETRY 5

#define ep_in 0x82
#define ep_out 0x02
static bool isInit = false; // init libusb library

static usb_config s_stUsbConfig[] = {
    {0x5722, 0x1A86, 0x82, 0x02},
    {0x5723, 0x1A86, 0x82, 0x02},
    {0x5724, 0x1A86, 0x82, 0x02},
    {0x5723, 0x3482, 0x82, 0x02},
    {0x5724, 0x3482, 0x82, 0x02},
    {0x5725, 0x3482, 0x82, 0x02},
};

static int  Gain_Multi[28][2] = {
    {0x03,0x20},//1
    {0x23,0x24},
    {0x23,0x35},
    {0x27,0x24},
    {0x27,0x2D},//5
    {0x27,0x35},
    {0x27,0x3E},
    {0x2F,0x24},
    {0x2F,0x28},
    {0x2F,0x2D},//10
    {0x2F,0x31},
    {0x2F,0x35},
    {0x2F,0x3A},
    {0x2F,0x3E},
    {0x3F,0x22},//15
    {0x3F,0x24},
    {0x3F,0x26},
    {0x3F,0x28},
    {0x3F,0x2A},
    {0x3F,0x2D},//20
    {0x3F,0x2F},
    {0x3F,0x31},
    {0x3F,0x33},
    {0x3F,0x35},
    {0x3F,0x38},//25
    {0x3F,0x3A},
    {0x3F,0x3C},
    {0x3F,0x3E},//28--28.094
};

UsbControl* UsbControl::_usbInstance = nullptr;
UsbControl::UsbControl() 
{
    initLibUsb();
    m_tfStatus = false;
}

UsbControl::~UsbControl()
{
    deInitLibUsb();
}

int UsbControl::initLibUsb()
{
    int ret = 0;
    if (isInit)
    {
        LOG(INFO) << "libusb library has already been initialized.\n";
        return 0;
    }
    ret = libusb_init(NULL);
    if (ret < 0)
    {
        LOG(INFO) << "inital libusb failed.  ret: " << ret << std::endl;
    }
    isInit = true;
    //libusb_set_debug(NULL, 3);
    return 0;
}

int UsbControl::deInitLibUsb()
{
    if (!isInit)
    {
        LOG(INFO) << "libusb library has already been exit.\n";
        return 0;
    }
    libusb_free_device_list(devs, 1);
    LOG(INFO) << "free device list successful.\n";
    libusb_exit(NULL);
    LOG(INFO) << "exit libusb library.\n";
    isInit = false;
    delete _usbInstance;
    _usbInstance = NULL;
    return 0;
}

static std::mutex mt;
UsbControl* UsbControl::getSingleInstance()
{
    mt.lock();
    if (_usbInstance == nullptr)
    {
        _usbInstance = new UsbControl();
    }
    mt.unlock();
    return _usbInstance;
}


int UsbControl::getUSBDeviceList(std::vector<uint8_t>& ports)
{
    int cnt = 0;
    uint8_t port = 0;
    AS_CAMERA_TYPE_E penType = AS_CAM_BUTT;
    cnt = libusb_get_device_list(NULL, &devs);
    libusb_device_List.clear();
    portMapDevice.clear();
    for (int index = 0; index < cnt; index++)
    {
        if ( checkPidVid(devs[index], &penType) == 0)
        {
            port = getPort(devs[index]);
            ports.push_back(port);
            libusb_device_List.push_back(devs[index]);
            portMapDevice.insert(std::pair<uint8_t, struct libusb_device*>(port, devs[index]));
        }
    }
    return cnt;
}

int UsbControl::modifyFPS(uint8_t port, AS_FPS_E enFps)
{
    auto pos = portMaphandle.find(port);
    if (pos == portMaphandle.end())
    {
        LOG(ERROR) << "can not find the libusb_handle,port: " << port << std::endl;
        return -1;
    }

    std::string cmd = std::string("ARG WRITE:F=") + std::to_string(enFps);
    std::string resp;
    int ret = response(port,cmd,resp,100);
    if (ret < 0)
    {
        return ret;
    }

    if ((resp.find("ERR") != std::string::npos) || (resp.find("FAIL") != std::string::npos) || (resp.length() == 0))
    {
        LOG(ERROR) << "Read return value failed.Return value is null or failed!\n";
        return -13;
    }

    return 0;

}

int UsbControl::checkPidVid(libusb_device* device, AS_CAMERA_TYPE_E* penType)
{
    struct libusb_device_descriptor desc;
    int ret = 0;
    ret = libusb_get_device_descriptor(device, &desc);
    if (ret < 0) 
    {
        LOG(INFO) << "failed to get device descriptor\n";
        return -1;
    }
    for (int index = 0; index < sizeof(s_stUsbConfig) / sizeof(s_stUsbConfig[0]); index++)
    {
        if (desc.idVendor == s_stUsbConfig[index].vid && desc.idProduct == s_stUsbConfig[index].pid) 
        {
            if (desc.idVendor == 0x3482 && desc.idProduct == 0x5723)
            {
                *penType = AS_CAM_NUWA_XB40;
            }
            else if (desc.idVendor == 0x3482 && desc.idProduct == 0x5724)
            {
                *penType = AS_CAM_NUWA_X100;
            }
            else if (desc.idVendor == 0x3482 && desc.idProduct == 0x5725)
            {
                *penType = AS_CAM_NUWA_HP60;
            }
            else {
                *penType = AS_CAM_BUTT;
                return -1;
            }
            return 0;
        }
    }
  
}


uint8_t UsbControl::getPort(libusb_device*& device)
{
    uint8_t path[20];
    int r = libusb_get_port_numbers(device, path, sizeof(path));
    if (r <= 0) {
        LOG(ERROR) << "port not available.\n";
    }
    uint8_t port = path[0];
    return port;

}

int UsbControl::getUsbDevice(uint8_t port, libusb_device*& device)
{
    auto pos = portMapDevice.find(port);
    if( pos == portMapDevice.end())
    {
        LOG(ERROR) << "can not find the libusb_device,port: " << port << std::endl;
        return -1;
    }
    device = pos->second;
    return 0;
}

int UsbControl::getUsbHandle(uint8_t port, libusb_device_handle*& handle)
{
    auto pos = portMaphandle.find(port);
    if (pos == portMaphandle.end())
    {
        LOG(ERROR) << "can not find the libusb_handle,port: " << port << std::endl;
        return -1;
    }
    handle = pos->second;
    return 0;
}

int UsbControl::getCameraType(unsigned char port, AS_CAMERA_TYPE_E* penType)
{
    auto port_device = portMapDevice.find(port);
    if(port_device == portMapDevice.end())
    {
        LOG(INFO) << "can not find the libusb_device accroding to the usb port,port: " << port << std::endl;
        return -1;
    }
    if (checkPidVid(port_device->second, penType) != 0)
    {
        return -1;
    }
    return 0;
}

int UsbControl::openPort(uint8_t port)
{
    int ret = 0;
    auto port_device = portMapDevice.find(port);
    if (port_device == portMapDevice.end())
    {
        LOG(ERROR) << "find port fail! port: "; printf("%d\n", port);
        return -1;
    }


    libusb_device_handle* handle;
    ret = libusb_open(port_device->second, &handle);
    if (ret)
    {
        LOG(INFO) << ret << std::endl;
        LOG(ERROR) << "Permission denied or Can not find the USB board! port: "; printf("%d\n", port);
        return -13;
    }


    
    ret = libusb_claim_interface(handle, 0x00);
    if (ret) {
        if (handle)
        {
            libusb_close(handle);
        }
        libusb_exit(NULL);
        LOG(ERROR) << "error claiming interface.USB device close and exit!!! port: " << port << std::endl;
    }
    libusb_control_transfer(handle, 0x21, 0x22, 0x02 | 0x01,0, NULL, 0, 10);
    portMaphandle.insert(std::pair<uint8_t, struct libusb_device_handle*>(port, handle));
    return ret;
}

int UsbControl::closePort(uint8_t port)
{
    auto port_handle = portMaphandle.find(port);
    if (port_handle == portMaphandle.end())
    {
        LOG(ERROR) << "find handle fail! port: "; printf("%d\n", port);
        return -1;
    }

    if (port_handle->second) 
    {
        libusb_control_transfer(port_handle->second, 0x21, 0x22, 0,
            0, NULL, 0, 10);
        libusb_release_interface(port_handle->second, 0);
        libusb_close(port_handle->second);
        portMaphandle.erase(port_handle);

    }
    return 0;
}


int UsbControl::getSerialNumber(uint8_t port, std::string& serialNumber)
{
    auto pos = portMaphandle.find(port);
    if (pos == portMaphandle.end())
    {
        LOG(ERROR) << "can not find the libusb_handle,port: " << port << std::endl;
        return -1;
    }

    std::string cmd = std::string("ARG READ:SN=");
    int ret = response(port, cmd, serialNumber, 100);
    if (ret < 0)
    {
        return ret;
    }
    //LOG(INFO) << "have not edited SN: " << serialNumber << std::endl;
    int index = serialNumber.find_last_of(":");

    serialNumber = serialNumber.substr(index + 1);
    return 0;
}

int UsbControl::setStreamType(uint8_t port, AS_STREAM_OPTION_E enOpt)
{
    
    std::string cmd = std::string("ARG WRITE:S=") + std::to_string(enOpt);
    std::string resp;
    int ret = response(port, cmd, resp, 100);
    if (ret < 0)
    {
        return ret;
    }
    return 0;
}

int UsbControl::write(uint8_t port, std::string cmd)
{
    auto port_handle = portMaphandle.find(port);
    if (port_handle == portMaphandle.end())
    {
        LOG(ERROR) << "find port fail! port: " << port << std::endl;
        return -1;
    }

    int ret = 0;
    int retry = 0;
    int length = 0;
    Wmutex.lock();
    while (1)
    {
        ret = libusb_bulk_transfer(port_handle->second, ep_out, (unsigned char*)cmd.c_str(), cmd.length(), &length, 200);
        if (ret == LIBUSB_SUCCESS || retry >= MAX_RETRY - 1)
        {
            break;
        }
        else if (ret == LIBUSB_ERROR_PIPE)
        {
            libusb_clear_halt(port_handle->second, ep_out);
        }
        retry++;
        
    }
    if ((ret != LIBUSB_SUCCESS) && length == 0)
    {
        LOG(WARN) << "send command fail!   cmd: " << cmd << std::endl;
    }
    Wmutex.unlock();
    return length;
}

int UsbControl::read(uint8_t port, std::string& Resp, int size)
{
    auto port_handle = portMaphandle.find(port);
    if (port_handle == portMaphandle.end())
    {
        LOG(ERROR) << "find port fail! port: " << port << std::endl;
        return -1;
    }

    //unsigned char* data = new unsigned char[size];
    unsigned char data[100] = { 0 };
    int ret = 0;
    int retry = 0;
    int length = 0;
    Rmutex.lock();
    while(1)
    {
        memset(data, 0, sizeof(100));
        ret = libusb_bulk_transfer(port_handle->second, ep_in, data, size, &length, 2000);
        if ( (ret == LIBUSB_SUCCESS) || (retry >= MAX_RETRY - 1) )
        {
            break;
        }
        else if (ret == LIBUSB_ERROR_PIPE)
        {
            libusb_clear_halt(port_handle->second, ep_in);
        }
        retry++;
       
    }
    if ((ret != LIBUSB_SUCCESS) && length == 0)
    {
        LOG(WARN) << "Read date fail!   ret " << ret << std::endl;
    }
    Resp.clear();
    Resp.append((char*)data);
    //delete []data;
    Rmutex.unlock();
    
    return length;
}


int UsbControl::response(uint8_t port, std::string cmd, std::string& Resp, int size)
{
    int ret = write(port, cmd);
    if (ret < 0)
    {
        return -1;
    }


    ret = read(port, Resp, size);
    if (ret < 0)
    {
        return -2;
    }
    return ret;
}

static void trim(char* str)
{
    char* tmp = str;
    char* p = str;

    while (*p != '\0') {
        if (*p != ' ') {
            *tmp++ = *p;
        }
        ++p;
    }
    *tmp = '\0';
}

int UsbControl::readInternalParamater(uint8_t port, char* param)
{
    std::string cmd = std::string("ARG READ:PA=");
    write(port, cmd);


    int cnt = 0;
    int c = 0;
    int length = 0;
    std::string resp;
    std::string info;
    do {       
        if ( (length = read(port, resp,64) ) < 0)
        {
            LOG(ERROR) << "read readInternalParamater fail!  port: " << port << std::endl;
        }
        cnt += length;
        c++;
        info.append(resp.c_str());
        resp.clear();
    } while (cnt < 200 && c < 18);

    int index = info.find_last_of(":");

    std::string para = info.substr(index + 1);

    char* chs = (char*)para.c_str();
    trim(chs);
    strcpy(param, chs);
    return 0;
}

int UsbControl::setTF(unsigned char port, bool enable)
{
    auto pos = portMaphandle.find(port);
    if (pos == portMaphandle.end())
    {
        LOG(ERROR) << "can not find the libusb_handle,port: " << port << std::endl;
        return -1;
    }

    std::string cmd = std::string("ARG WRITE:TF=") + std::string(std::to_string(enable));
    std::string resp;
    int ret = response(port, cmd, resp, 100);
    if (ret < 0)
    {
        return ret;
    }

    return 0;
}


int UsbControl::readTemperatureAdc(unsigned char port, int* ps32Ito, int* ps32Ntc)
{
    auto pos = portMaphandle.find(port);
    if (pos == portMaphandle.end())
    {
        LOG(ERROR) << "can not find the libusb_handle,port: " << port << std::endl;
        return -1;
    }

    std::string cmd = std::string("DB4=1");
    std::string resp;
    int ret = response(port, cmd, resp, 100);
    if (ret < 0)
    {
        return ret;
    }

    ret = sscanf(resp.c_str(), "ADC Value ITO:%d NTC:%d", ps32Ito, ps32Ntc);
    if (ret != 2) {
        // LOG(ERROR) << "get ito and ntc adc value failed\n";
        return -1;
    }

    return 0;
}

int UsbControl::adc2Temperature(int adc, int* ps32Tem)
{
    int ntcR = adc * 10000 / (1800 - adc) / 10;
    for (int i = 0; i < 101; i++) {
        if (ntcR > tempTab[i]) {
            *ps32Tem = i - 20;
            return 0;
        }
    }
    return -1;

}

int UsbControl::writeTemperature(unsigned char port, int s32Tem)
{
    auto pos = portMaphandle.find(port);
    if (pos == portMaphandle.end())
    {
        LOG(ERROR) << "can not find the libusb_handle,port: " << port << std::endl;
        return -1;
    }

    std::string cmd = std::string("ARG WRITE:TS=") + std::string(std::to_string(s32Tem));
    std::string resp;
    int ret = response(port, cmd, resp, 100);
    if (ret < 0)
    {
        return ret;
    }
    if ((resp.find("ERR") != std::string::npos) || (resp.find("FAIL") != std::string::npos) || (resp.length() == 0))
    {
        LOG(ERROR) << "Read return value failed.Return value is null or failed!\n";
        return -13;
    }
    return 0;
}

int UsbControl::configAutoTemperatureCompensation(unsigned char port, bool enbale)
{
    int ret = 0;
    int s32Ito = 0;
    int s32Ntc = 0;
    int s32Tmp = 0;
    static int s32LastTmp = 0;
    static bool bFirst = true;
#ifdef LOG_FOR_TEST_TS
    static int log_cnt = 0;
#endif
    if ((m_tfStatus != enbale) || bFirst) {
        setTF(port, enbale);
        s32LastTmp = 0;
        bFirst = false;
    }
    m_tfStatus = enbale;
#ifndef LOG_FOR_TEST_TS
    if (!m_tfStatus) {
        return 0;
    }
#endif
    ret = readTemperatureAdc(port, &s32Ito, &s32Ntc);
    if (ret != 0) {
        // LOG(ERROR) << "readTemperatureAdc failed\n";
        return -1;
    }

    ret = adc2Temperature(s32Ntc, &s32Tmp);
    if (ret != 0) {
        LOG(ERROR) << "adc to temmperature failed\n";
        return -2;
    }
#ifdef LOG_FOR_TEST_TS
    if (log_cnt++ % 20 == 0) {
        printf("TMP = %d\n", s32Tmp);
    }
    if (!m_tfStatus) {
        return 0;
    }
#endif
    if (s32Tmp != s32LastTmp) {
        ret = writeTemperature(port, s32Tmp);
        if (ret != 0) {
            LOG(ERROR) << "writeTemperature failed\n";
            return -3;
        }
    }
    s32LastTmp = s32Tmp;

    return ret;
}

int UsbControl::setPwmDuty(unsigned char port, int duty)
{
    auto pos = portMaphandle.find(port);
    if (pos == portMaphandle.end())
    {
        LOG(ERROR) << "can not find the libusb_handle,port: " << port << std::endl;
        return -1;
    }

    if (duty < 0 || duty > 99) {
        LOG(INFO) << "duty:" << duty << " not in range[0,99]" << std::endl;
        return 0;
    }

    std::string cmd = std::string("ARG WRITE:P=") + std::string(std::to_string(duty));
    std::string resp;
    int ret = response(port, cmd, resp, 100);
    if (ret < 0)
    {
        return ret;
    }

    if ((resp.find("ERR") != std::string::npos) || (resp.find("FAIL") != std::string::npos) || (resp.length() == 0))
    {
        LOG(ERROR) << "Read return value failed.Return value is null or failed!\n";
        return -13;
    }
    return 0;
}

int UsbControl::getPwmDuty(unsigned char port, int* duty)
{
    auto pos = portMaphandle.find(port);
    if (pos == portMaphandle.end())
    {
        LOG(ERROR) << "can not find the libusb_handle,port: " << port << std::endl;
        return -1;
    }

    std::string cmd = std::string("ARG READ:P=");
    std::string resp;
    int ret = response(port, cmd, resp, 100);
    if (ret < 0)
    {
        return ret;
    }

    if ((resp.find("ERR") != std::string::npos) || (resp.find("FAIL") != std::string::npos) || (resp.length() == 0))
    {
        LOG(ERROR) << "Read return value failed.Return value is null or failed!\n";
        return -13;
    }

    ret = sscanf(resp.c_str(), "READ OK:%d", duty);
    if (ret != 1) {
        *duty = -1;
        return -1;
    }
    return 0;

}

int UsbControl::setLedTime(unsigned char port, int time)
{
    auto pos = portMaphandle.find(port);
    if (pos == portMaphandle.end())
    {
        LOG(ERROR) << "can not find the libusb_handle,port: " << port << std::endl;
        return -1;
    }

    if (time < 0 || time > 99) {
        LOG(INFO) << "time:" << time << " not in range[0,99]" << std::endl;
        return 0;
    }

    std::string cmd = std::string("ARG WRITE:T=") + std::string(std::to_string(time));
    std::string resp;
    int ret = response(port, cmd, resp, 100);
    if (ret < 0)
    {
        return ret;
    }

    if ((resp.find("ERR") != std::string::npos) || (resp.find("FAIL") != std::string::npos) || (resp.length() == 0))
    {
        LOG(ERROR) << "Read return value failed.Return value is null or failed!\n";
        return -13;
    }
    return 0;
}

int UsbControl::getLedTime(unsigned char port, int* time)
{
    auto pos = portMaphandle.find(port);
    if (pos == portMaphandle.end())
    {
        LOG(ERROR) << "can not find the libusb_handle,port: " << port << std::endl;
        return -1;
    }


    std::string cmd = std::string("ARG READ:T=") ;
    std::string resp;
    int ret = response(port, cmd, resp, 100);
    if (ret < 0)
    {
        return ret;
    }

    if ((resp.find("ERR") != std::string::npos) || (resp.find("FAIL") != std::string::npos) || (resp.length() == 0))
    {
        LOG(ERROR) << "Read return value failed.Return value is null or failed!\n";
        return -13;
    }
    ret = sscanf(resp.c_str(), "READ OK:%d", time);
    if (ret != 1) {
        *time = -1;
        return -1;
    }
    return 0;

}

int UsbControl::resetStream(unsigned char port)
{
    auto pos = portMaphandle.find(port);
    if (pos == portMaphandle.end())
    {
        LOG(ERROR) << "can not find the libusb_handle,port: " << port << std::endl;
        return -1;
    }

    std::string cmd = std::string("ARG WRITE:S=0");
    std::string resp;
    int ret = response(port, cmd, resp, 100);
    if (ret < 0)
    {
        return ret;
    }

    cmd.clear();
    resp.clear();
    cmd = std::string("ARG WRITE:S=3");
    ret = response(port, cmd, resp, 100);
    if (ret < 0)
    {
        return ret;
    }

    return 0;
}


int UsbControl::setCameraGain(unsigned char port, int amplification)
{
    auto pos = portMaphandle.find(port);
    if (pos == portMaphandle.end())
    {
        LOG(ERROR) << "can not find the libusb_handle,port: " << port << std::endl;
        return -1;
    }
    if (amplification < 0 || amplification > 28)
    {
        LOG(INFO) << "please input a suitable amplification  [0,28]\n";
        return -1;
    }

    /*check the amplification to decide if to open the auto exposure */
    if (amplification != 0)
    {
        std::string cmd = std::string("ARG WRITE:E=0");
        std::string resp;
        int ret = response(port, cmd, resp, 100);
        if (ret < 0)
        {
            return ret;
        }
    }
    else {
        std::string cmd = std::string("ARG WRITE:E=1");
        std::string resp;
        int ret = response(port, cmd, resp, 100);
        if (ret < 0)
        {
            return ret;
        }
    }

    /*set the camera gain through the I2C */
    char intToHex[10];
    memset(intToHex, 0, sizeof(intToHex));
    sprintf(intToHex, "%02x", Gain_Multi[amplification - 1][0]);
    myToupper(intToHex);
    std::string cmd = std::string("I2C WRITE:K=3E08") + std::string(intToHex);
    std::string resp;
    int ret = response(port, cmd, resp, 100);
    if (ret < 0)
    {
        return ret;
    }
    //LOG(INFO) << "cmd: " << cmd << "resp: " << resp << std::endl;

    memset(intToHex, 0, sizeof(intToHex));
    sprintf(intToHex, "%02x", Gain_Multi[amplification - 1][1]);
    myToupper(intToHex);
    cmd = std::string("I2C WRITE:K=3E09") + std::string(intToHex);
    ret = response(port, cmd, resp, 100);
    if (ret < 0)
    {
        return ret;
    }
    //LOG(INFO) << "cmd: " << cmd << "resp: " << resp << std::endl;
    return 0;
}

void myToupper(char* str)
{
    int length = strlen(str);
    for (size_t i = 0; i < length; i++)
    {
        if (str[i] >= 'a' && str[i] <= 'z')
        {
            str[i] = toupper(str[i]);
        }
    }
}

char checkSum(char* data,int length)
{
    char sum = 0x00;
    while (data != nullptr)
    {
        if (sum + *data >= 0xFF)
        {
            sum = *(data++) + sum - 0xFF;
        }
        else
        {
            sum += *(data++) ;
        }
    }

    return sum;
}

int UsbControl::upgradeHimax(unsigned char port, char* firmwareData,int length)
{
    auto pos = portMaphandle.find(port);
    if (pos == portMaphandle.end())
    {
        LOG(ERROR) << "can not find the libusb_handle,port: " << port << std::endl;
        return -1;
    }

    /*enter the upgrade status*/
    std::string cmd = std::string("HimaxBoot\r\n");
    std::string resp;
    response(port,cmd,resp,100);


    /*sending code*/
    char* sendData = (char*)malloc(length + 3);
    memset(sendData, 0, length + 3);
    char len;
    if (length == 256)
    {
        len = 0x03;
    }
    else if (length == 128)
    {
        len = 0x01;
    }
    else if (length == 32)
    {
        len = 0x02;
    }
    sendData[0] = len;
    sendData[1] = len >> 4;
    sendData[2] = ~(len >> 4);
    memcpy(sendData + 3, firmwareData, length);
    sendData[3 + length] = checkSum(firmwareData,length);

    cmd = std::string(sendData);
    response(port,cmd, resp,100);
    free(sendData);


    /*sending code finish*/
    cmd = std::string("04");
    write(port, cmd);

    return 0;
}

int UsbControl::resetHimax(unsigned char port)
{
    std::string cmd = std::string("ARG WRITE:RST=\r\n");
    std::string resp;
    int ret = response(port, cmd, resp, 100);
    if (ret < 0)
    {
        return ret;
    }
    return 0;
}