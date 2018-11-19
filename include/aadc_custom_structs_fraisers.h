//
// Created by aadc on 25.10.18.
//

#ifndef AADC_USER_AADC_CUSTOM_STRUCTS_FRAISERS_H
#define AADC_USER_AADC_CUSTOM_STRUCTS_FRAISERS_H


#pragma pack(push,1)

#include <a_utils/base/types.h>

typedef struct
{
    tFloat32 f32Angle;
    tFloat32 f32Distance;
    tInt16 i16Class;
    tInt32 i32Width;
    tInt32 i32Height;

} tLaserSegStruct;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tFloat32 f32pCarLeft;
    tFloat32 f32pCarRight;
    tFloat32 f32pCarCenter;
    tFloat32 f32pPerson;
    tFloat32 f32pChild;
    tFloat32 f32pObsLeft;
    tFloat32 f32pObsRight;
    tFloat32 f32pObsRear;
} tDetectionInfo;
#pragma pack(pop)


#endif //AADC_USER_AADC_CUSTOM_STRUCTS_FRAISERS_H
