#ifndef TYPES_H_
#define TYPES_H_


typedef struct
{
    float range_sc;
    float azimuth_sc;
    float vr;
    float rcs;
    float vr_compensated;
    float x_cc;
    float y_cc;
    float x_seq;
    float y_seq;
} radar_point_t;



#endif