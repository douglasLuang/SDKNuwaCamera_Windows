#include "VideoControl.h"
#include "Logger.h"
std::string fourcc(unsigned int format)
{
    char formatArray[] = { (char)(format & 0xff), (char)((format >> 8) & 0xff), (char)((format >> 16) & 0xff), (char)((format >> 24) & 0xff), 0 };
    return std::string(formatArray, strlen(formatArray));
}


int init(int deviceID, DeviceResolution* devRes,videoInput &videoinput)
{
    /* set format */
    int width = 640;
    int height = 400;

    /* set resolution */
    if (devRes == NULL) {
        LOG(WARN) << "resolution is NULL. use default resolution 400*640\n";
    }
    else {

        width = devRes->t_width;
        height = devRes->t_height;
        LOG(INFO) << "width: " << width << " height: " << height << std::endl;
    }

    /* init and set up device */
    if (false == videoinput.setupDevice(deviceID, width, height,  VI_MEDIASUBTYPE_YUY2))
    {
        LOG(ERROR) << "open dev  failed!\n";
        return -1;
    }
  
   
    //checkCapabilities      unknown

    return 0;
}


int setFormat(int deviceID,int format,videoInput &videoinput)
{
    return videoinput.setFormat(deviceID, format);
}


int readInternal(int deviceID, void* data, size_t size, videoInput &videoinput)
{  
    if (videoinput.isFrameNew(0))
    {
        if (FALSE == videoinput.getPixels(0, (unsigned char*)data, false, false))
        {
            return -2;
        }
    }
    else
    {
        return -1;
    }
    return 0;
}