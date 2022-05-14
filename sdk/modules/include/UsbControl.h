#pragma once
#include <string>
#include <map>
#include <mutex>
#include "deviceDataDef.h"
#include "as_datadef.h"
#include "Logger.h"
#include "libusb.h"
class USBControllerPrivate;

class UsbControl
{
private:
    static UsbControl* _usbInstance;

public: 
    UsbControl();
    ~UsbControl();
    int initLibUsb();
    int deInitLibUsb();
    static UsbControl* getSingleInstance();
    int getUSBDeviceList(std::vector<uint8_t>& ports);   
    int modifyFPS(uint8_t port, AS_FPS_E enFps);
    uint8_t getPort(libusb_device*& device);
    int getUsbDevice(uint8_t port, libusb_device*& device);
    int getUsbHandle(uint8_t port, libusb_device_handle*& handle);
    int getCameraType(unsigned char port, AS_CAMERA_TYPE_E* penType);
    int openPort(uint8_t port);
    int closePort(uint8_t port);
    int getSerialNumber(uint8_t port, std::string& serialNumber);
    int setStreamType(uint8_t port, AS_STREAM_OPTION_E enOpt);
    int readInternalParamater(uint8_t port, char* param);
    int readTemperatureAdc(unsigned char port, int* ps32Ito, int* ps32Ntc);
    int writeTemperature(unsigned char port, int s32Tem);
    int configAutoTemperatureCompensation(unsigned char port, bool enbale);
    int setPwmDuty(unsigned char port, int duty);
    int getPwmDuty(unsigned char port, int* duty);
    int setLedTime(unsigned char port, int time);
    int getLedTime(unsigned char port, int* time);
    int resetStream(unsigned char port);
    int setCameraGain(unsigned char port, int amplification);
    int upgradeHimax(unsigned char port, char* firmwareData, int length);
    int resetHimax(unsigned char port);
private:
    std::vector<libusb_device*> libusb_device_List;
    std::map<uint8_t, struct libusb_device*> portMapDevice;
    std::map<uint8_t, struct libusb_device_handle*> portMaphandle;
    struct libusb_device** devs;
    std::mutex Rmutex;
    std::mutex Wmutex;
    bool m_tfStatus;

    int write(uint8_t port, std::string cmd);
    int read(uint8_t port, std::string& Resp, int size);
    int response(uint8_t port, std::string cmd, std::string& Resp, int size);
    int checkPidVid(libusb_device* device, AS_CAMERA_TYPE_E* penType);
    int setTF(unsigned char port, bool enable);
    int adc2Temperature(int adc, int* ps32Tem);
    
  
};
void myToupper(char* str);
char checkSum(char* data,int length);