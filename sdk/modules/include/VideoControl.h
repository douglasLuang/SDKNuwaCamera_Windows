#pragma once

#include <string>
#include "deviceDataDef.h"
#include "videoInput.h"
#define V4L2MMAP_NBBUFFER 1

std::string fourcc(unsigned int format);

int init(int deviceID, DeviceResolution* devRes, videoInput &videoinput);
int setFormat(int deviceID, int format, videoInput &videoinput);
int readInternal(int deviceID, void* data, size_t size, videoInput &videoinput);