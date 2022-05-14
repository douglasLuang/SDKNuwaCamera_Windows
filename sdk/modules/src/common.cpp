#include <fstream>
#include <stdio.h>
#include "common.h"
#include "Logger.h"

int saveYUVImg(const char* const fileName, void* dataImg, const size_t size)
{
    if (fileName == NULL) {
        LOG(ERROR) << "null ptr!\n";
        return -1;
    }

    FILE* fp;
    fp = fopen(fileName, "wb+");
    if (fp == NULL) {
        LOG(ERROR) << "null ptr!\n";
        return -2;
    }
    fwrite(dataImg, size, 1, fp);
    fflush(fp);
    fclose(fp);

    return 0;
}

int saveCloudDot(const char* const fileName, float* cloudDot, size_t length)
{
    if (fileName == NULL) {
        LOG(ERROR) << "null ptr!\n";
        return -1;
    }

    std::ofstream out(fileName, std::ios::binary);
    if (!out) {
        LOG(ERROR) << "null ptr!\n";
        return -2;
    }
    for (int i = 0, size = length / 3; i < size; i++) {
        size_t index = i * 3;
        out << cloudDot[index + 0] << ";";
        out << cloudDot[index + 1] << ";";
        out << cloudDot[index + 2] << "\n";
    }

    out.close();

    return 0;
}
