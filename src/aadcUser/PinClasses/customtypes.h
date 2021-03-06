// This is a generated file, changes to it may be overwritten in the future.
/**********************************************************************
Copyright (c) 2017, team frAIburg
Licensed under BSD-3.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************/
#ifndef AADCUSER_FRAIBURG_DESCRIPTION_CUSTOMTYPES_H_
#define AADCUSER_FRAIBURG_DESCRIPTION_CUSTOMTYPES_H_

#include "stdafx.h"

#pragma pack(push,1)
typedef struct
{
    tFloat32 x;
    tFloat32 y;
} tPoint;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tFloat32 v;
    tFloat32 s;
} tGoalSpeedDistance;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tFloat32 coeff0;
    tFloat32 coeff1;
    tFloat32 coeff2;
    tFloat32 dist;
    tFloat32 angleOriginRad;
    tFloat32 startx;
    tFloat32 starty;
    tFloat32 endx;
    tFloat32 endy;
    tInt16 alignment;
    tInt16 orientation;
    tInt8 isVertLane;

} tLaneElement;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tInt16 status;
} tPlannerStatus;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tInt32 behaviour_id;
    tInt32 behaviour_next_id;
    tInt32 object_id;
    tInt32 speed_id;
    tInt32 timestamp;
    tBool is_finished;
} tBehaviour;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tInt32 light_id;
    tBool switch_bool;
} tLightCommand;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tFloat32 x;
    tFloat32 y;
    tFloat32 heading;
    tFloat32 accuracy;
} tCrossing;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tInt16 id;
    tFloat32 x;
    tFloat32 y;
    tFloat32 heading;
    tInt16 count;
    tUInt32 ticks;
    tFloat32 dist;
    tBool init;
} tRoadSignData;
#pragma pack(pop)
// From aadc_structs.h
#pragma pack(push,1)
typedef struct
{
    tUInt32 ui32ArduinoTimestamp;
    tBool bValue;
} tBoolSignalValue;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tFloat32 f32Radius;
    tFloat32 f32Angle;
} tPolarCoordiante;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tUInt32 ui32Size;
    tPolarCoordiante tScanArray[360];
} tLaserScannerData;
#pragma pack(pop)


#endif  // AADCUSER_FRAIBURG_DESCRIPTION_CUSTOMTYPES_H_