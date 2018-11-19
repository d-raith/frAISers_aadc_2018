//
// Created by aadc on 27.09.18.
//

#ifndef AADC_USER_ROADSIGNSMAPPIN_H
#define AADC_USER_ROADSIGNSMAPPIN_H
#include "stdafx.h"
#include "aadc_structs.h"
#include "aadc_jury.h"


using namespace adtf_util;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;
using namespace aadc::jury;


/*! Storage structure for the road sign data */
typedef struct _roadSign
{
    /*! road sign */
    tInt16 u16Id;

    /*! init sign */
    tBool bInit;

    /*! location  x*/
    tFloat32 f32X;
    /*! location  y*/
    tFloat32 f32Y;

    /*! sign search radius */
    tFloat32 f32Radius;

    /*! direction (heading) of the road sign */
    tFloat32 f32Direction;

    /*! Number of 16s */
    tInt u16Cnt;

    /*! measurement ticks*/
    tTimeStamp u32ticks;

} RoadSign;

typedef struct _parking
{
    /*! road sign */
    tInt16 u16Id;

    tInt16 status;

    /*! location  x*/
    tFloat32 f32X;
    /*! location  y*/
    tFloat32 f32Y;

    /*! direction (heading) of the road sign */
    tFloat32 f32Direction;

} ParkingSpace;


class RoadSignsMapPin {

public:




    enum RW {
        READ
    };

private:

    cPinReader m_oReader;

    /*! indicate if this pin instance is for reading or writing */
    RW rw;

    /*! Pointer to the clock of the filter. */
    const object_ptr<adtf::services::IReferenceClock>* m_pClock;

    /*! Pointer to the filter object that wants to register the pin. */
    cTriggerFunction* filter;

    /* Indicates if the pin was already registered */
    bool was_registered = false;




    tResult processRoadSignFile(const ISample& sample,
                                vector<RoadSign> *roadSigns,
                                vector<ParkingSpace> *parking);


public:
    /*! Default constructor. */
    RoadSignsMapPin() = default;

    /*! Function to register a pin.
        Parameter p_rw: indicates weather to register the pin for reading or for writing.
        Parameter filter: Pointer to the filter object that wants to register the pin.
        Parameter p_Clock: Pointer to the clock of the filter object. The responsibility
                            for registering the clock remains at the filter!
        Parameter pin_name: Name for registering the pin.
        Return: True if successfull, False if not successfull. */
    tResult registerPin(RW p_rw,
                        cTriggerFunction* filter,
                        const object_ptr<adtf::services::IReferenceClock>* p_clock,
                        const tChar* pin_name);



    /* Function to read data from the pin.
       Pin must have been registered for reading before. */
    tResult readData(vector<RoadSign> *roadSigns,
                     vector<ParkingSpace> *parking);


    static tResult parseRoadSignFile(cDOM& oDOM, vector<RoadSign> *roadSigns,
                              vector<ParkingSpace> *parking, bool convertDirectionToRadian = false);
};



#endif //AADC_USER_ROADSIGNSMAPPIN_H
