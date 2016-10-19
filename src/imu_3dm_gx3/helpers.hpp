#pragma once

namespace imu_3dm_gx3_ros_tool {

static float extract_float(unsigned char* addr) {
    float tmp;

    *((unsigned char*)(&tmp) + 3) = *(addr);
    *((unsigned char*)(&tmp) + 2) = *(addr + 1);
    *((unsigned char*)(&tmp) + 1) = *(addr + 2);
    *((unsigned char*)(&tmp)) = *(addr + 3);

    return tmp;
}

static int extract_int(unsigned char* addr) {
    int tmp;

    *((unsigned char*)(&tmp) + 3) = *(addr);
    *((unsigned char*)(&tmp) + 2) = *(addr + 1);
    *((unsigned char*)(&tmp) + 1) = *(addr + 2);
    *((unsigned char*)(&tmp)) = *(addr + 3);

    return tmp;
}

bool validate_checksum(const unsigned char* data, unsigned short length) {
    unsigned short chksum = 0;
    unsigned short rchksum = 0;

    for (unsigned short i = 0; i < length - 2; i++)
        chksum += data[i];

    rchksum = data[length - 2] << 8;
    rchksum += data[length - 1];

    return (chksum == rchksum);
}

inline void print_bytes(const unsigned char* data, unsigned short length) {
    for (unsigned int i = 0; i < length; i++)
        printf("%2x ", data[i]);
    puts("");
}
}
