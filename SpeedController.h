/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

**********************************************************************/

//THIS CODE WAS POSSIBLE DUE TO THE TEMPLATE PROVIDED BY AUDI. SARIM MEHDI, TONG LIU AND FRANCESCO SOVRANO
//MODIFIED THE CODE FOR THEIR SPECIFIC PURPOSE

#pragma once

//*************************************************************************************************
#define CID_TEMPLATEFILTER_DATA_TRIGGERED_FILTER "speed_controller.filter.user.aadc.cid"

#include "stdafx.h"
// python
#include "ffpython.h"

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace adtf::filter::ant;
using namespace std;
using namespace cv;
using namespace aadc::jury;
using namespace ODReader;


#include "tensorflow/cc/ops/const_op.h"
#include "tensorflow/cc/ops/image_ops.h"
#include "tensorflow/cc/ops/standard_ops.h"
#include "tensorflow/core/framework/graph.pb.h"
#include "tensorflow/core/framework/tensor.h"
#include "tensorflow/core/graph/default_device.h"
#include "tensorflow/core/graph/graph_def_builder.h"
#include "tensorflow/core/lib/core/errors.h"
#include "tensorflow/core/lib/core/stringpiece.h"
#include "tensorflow/core/lib/core/threadpool.h"
#include "tensorflow/core/lib/io/path.h"
#include "tensorflow/core/lib/strings/stringprintf.h"
#include "tensorflow/core/platform/env.h"
#include "tensorflow/core/platform/init_main.h"
#include "tensorflow/core/platform/logging.h"
#include "tensorflow/core/platform/types.h"
#include "tensorflow/core/public/session.h"
#include "tensorflow/core/util/command_line_flags.h"
#include <string.h>

enum SIGNPOST
{ // this should go in header
    NOTHING = -1,
    UNMARKED_INTERSECTION = 0,
    STOP = 1,
    PARKING = 2,
    HAVE_WAY = 3,
    AHEAD_ONLY = 4,
    GIVE_WAY = 5,
    PEDESTRIAN_CROSSING = 6,
    ROUNDABOUT = 7,
    NO_OVERTAKING = 8,
    NO_ENTRY_VEHICULAR_TRAFFIC = 9,
    STARTING_POSITION = 10,
    ONEWAY_STREET = 11,
    ROADWORKS = 12,
    SPEEDLIMIT_50 = 13,
    SPEEDLIMIT_100 = 14
};

typedef vector<float> tuple_t;
typedef vector<tuple_t> list_tuple_t;
typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::milliseconds ms;
typedef std::chrono::duration<float> fsec;
typedef std::chrono::_V2::system_clock::time_point time_point;

/*! Our class derives from this cTriggerFunction */
class cSpeedController : public cTriggerFunction
{
private:
    /*! The output sample writer turn right lights */
    cPinWriter     m_oOutputTurnRight;
    /*! The output sample writer turn left lights */
    cPinWriter     m_oOutputTurnLeft;
    /*! The output sample writer hazard lights */
    cPinWriter     m_oOutputHazard;
    /*! The output sample writer head lights */
    cPinWriter     m_oOutputHeadLight;
    /*! The output sample writer reverse lights */
    cPinWriter     m_oOutputReverseLight;
    /*! The output sample writer brake lights */
    cPinWriter     m_oOutputBrakeLight;
    /*! sample writer of The output driver structure */
    cPinWriter     m_oOutputDriverStruct;
    /*! sample reader The input jury structure */
    cPinReader     m_oInputJuryStruct;
    /*! sample reader List of input maneuvers */
    cPinReader     m_oInputManeuverList;
    /*! pin reader for the road sign file */
    cPinReader m_roadSignMapData;
    /*! The input ultrasonic unit */
    cPinReader m_oInputUltrasonicUnit;

    vector<roadElement> currentRoad;
    vector<tManeuver> listOfManeuvers;
    vector<tManeuver> listOfManeuversForSector;

    tBool loadedManeuverList;

    tBool loadedRoadSignsList;

    tBool executingManeuver;

    tBool getPosition;

    tInt32 m_sectorIterator;
    tInt32 m_maneuverIterator;

    /*! The reference clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;

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

    } roadSign;

    /*! storage for the roadsign data */
    vector<roadSign> m_roadSigns;

    /*! A ddl laser scanner identifier. */
    struct ddlLaserScannerDataId
    {
        tSize size;
        tSize scanArray;
    } m_ddlLSDataId;

    /*! The ls structure sample factory */
    adtf::mediadescription::cSampleCodecFactory m_LSStructSampleFactory;

    /*! The input laser scanner */
    cPinReader m_oInputLaserScanner;

    //Media Descriptions
    struct tJuryStructId
    {
        tSize actionId;
        tSize maneuverEntry;
    } m_ddlJuryStructId;

    /*! The jury structure sample factory */
    cSampleCodecFactory m_juryStructSampleFactory;

    struct tDriverStructId
    {
        tSize stateId;
        tSize maneuverEntry;
    } m_ddlDriverStructId;

    /*! The driver structure sample factory */
    cSampleCodecFactory m_driverStructSampleFactory;

    /*! A signal value identifier. */
    struct tSignalValueId
    {
        tSize timeStamp;
        tSize value;
    } m_ddlSignalValueId;

    /*! The signal value sample factory */
    adtf::mediadescription::cSampleCodecFactory m_SignalValueSampleFactory;

    /*! A bool signal value identifier. */
    struct tBoolSignalValueId
    {
        tSize ui32ArduinoTimestamp;
        tSize bValue;
    } m_ddlBoolSignalValueId;

    /*! The signal value sample factory */
    adtf::mediadescription::cSampleCodecFactory m_BoolSignalValueSampleFactory;

    /*! A ddl ultrasonic structure index. */
    struct tUltraSonic
    {
        tSignalValueId SideLeft;
        tSignalValueId SideRight;
        tSignalValueId RearLeft;
        tSignalValueId RearCenter;
        tSignalValueId RearRight;

    } m_ddlUltrasonicStructIndex;

    adtf::mediadescription::cSampleCodecFactory m_USDataSampleFactory;

    /*! The ddl indices for a tPosition */
    struct tGetPosition
    {
        tSize x;
        tSize y;
        tSize radius;
        tSize speed;
        tSize heading;
    }  m_ddlPositionControllerIndex;

    /*! The reader position */
    adtf::filter::cPinReader m_oReaderPosForController;
    /*! The position sample factory */
    adtf::mediadescription::cSampleCodecFactory m_PositionControllerSampleFactory;

    /*! Media description for variable received telling car to correct itself in the lane.
     * This is kind of like a safety mechanism like the emergency brake. It will become active
     * whenever you are about to go off road or go out of your lane. However, during collision
     * detection (as an example) I will turn this off. */
    struct tLaneCheck
    {
        tSize timeStamp;
        tSize value;
    } m_ddlLaneCheck;

    /*! The signal data sample factory. It is used for reading out the samples */
    adtf::mediadescription::cSampleCodecFactory m_LaneCheckFactory;

    /*! Reader of an InPin. I receive information from the vision about what object it saw in front */
    cPinReader m_oLaneCheck;

    struct tObjectCheck
    {
        tSize timeStamp;
        tSize value;
    } m_ddlObjectCheck;

    /*! The signal data sample factory. It is used for reading out the samples */
    adtf::mediadescription::cSampleCodecFactory m_ObjectCheckFactory;

    /*! Reader of an InPin. */
    cPinReader m_oObjectCheck;

    struct tRearObjectCheck
    {
        tSize timeStamp;
        tSize value;
    } m_ddlRearObjectCheck;

    /*! The signal data sample factory. It is used for reading out the samples */
    adtf::mediadescription::cSampleCodecFactory m_RearObjectCheckFactory;

    /*! Reader of an InPin. */
    cPinReader m_oRearObjectCheck;

    /*! The ddl indices for a tRoadSignExt */
    struct
    {
        tSize id;
        tSize size;
        tSize tvec;
        tSize rvec;
    } m_ddlRoadSignIndex;

    /*! The road sign sample factory */
    adtf::mediadescription::cSampleCodecFactory m_RoadSignSampleFactory;

    /*! Reader of an InPin roadSign. */
    cPinReader m_oReaderRoadSign;

    /*! Media description for speed of car. */
    struct tSpeed
    {
        tSize timeStamp;
        tSize value;
    } m_ddlSpeed;

    /*! The signal data sample factory. It is used for reading out the samples */
    adtf::mediadescription::cSampleCodecFactory m_SpeedFactory;

    /*! Writer to an OutPin. */
    cPinWriter m_oSpeed;

    /*! Media description for steering of car. */
    struct tSteering
    {
        tSize timeStamp;
        tSize value;
    } m_ddlSteering;

    /*! The signal data sample factory. It is used for reading out the samples */
    adtf::mediadescription::cSampleCodecFactory m_SteeringFactory;

    /*! Writer to an OutPin. */
    cPinWriter m_oSteering;

    /*! The maneuver file string */
    cString m_strManeuverFileString;

    /*! The road signs file string */
    cString m_strRoadSignsFileString;

    /*! this is the list with all the loaded sections from the maneuver list*/
    aadc::jury::maneuverList m_sectorList;

    /*! The car will tell the jury whether it is ready, running, error etc. */
    stateCar stateID = statecar_ready; //In the beginning I will always be ready

    /*! If this is true, your car stops. Your car must stop after travelling a certain distance */
    tBool m_Stop = true;

    /*! Don't do lane detection while turning */
    tBool m_ignoreLaneDetection = false;

    /*! I will not go to next maneuver if this bool is true */
    tBool m_evaluatingSign;

    /*! If I see the parking sign and the maneuver right after the current maneuver is parking, then set this bool */
    tBool m_readyToPark;

    /*! If this is true, I will start feeding negative values of velocity and the angle will also be reversed */
    tBool m_reverse;

    /*! This variable stores the car's original speed when it encounters a speed sign */
    tFloat32 m_originalCarSpeed = 0;

    /*! rotation of car about the z-axis */
    tFloat32 m_orientation;

    /*! my steering angle (in raidans) */
    tFloat32 m_steeringAngle;

    /*! my car's speed */
    tFloat32 m_currentSpeed;

    /*! An array of parking spaces */
    tFloat32 listOfParkingSpacesX[8] = {16.0, 16.0, 16.0, 16.0, 18.0, 18.0, 18.0, 18.0};
    tFloat32 listOfParkingSpacesY[8] = {7.75, 7.25, 6.75, 6.25, 8.25, 8.75, 9.25, 9.75};

    tInt32 sizeOfParkingSpacesArray;

    property_variable<tFloat32> m_rangeOfSign = tFloat32(0.50);
    property_variable<tFloat32> m_timeToCheckStopSign = tFloat32(2.0);
    property_variable<tFloat32> m_timeToCheckPedestrian = tFloat32(5.0);
    property_variable<tFloat32> m_speedUpperLimit = tFloat32(1.0);
    property_variable<tFloat32> m_propUSDistanceObs = tFloat32(10.0);

    /*! Vector that stores information of all road signs */
    vector<roadSign> ListOfUnmarkedSigns;
    vector<roadSign> ListOfStopSigns;
    vector<roadSign> ListOfParkingSigns;
    vector<roadSign> ListOfHaveWaySigns;
    vector<roadSign> ListOfStraightSigns;
    vector<roadSign> ListOfGiveWaySigns;
    vector<roadSign> ListOfPedestrianSigns;
    vector<roadSign> ListOfRoundaboutSigns;
    vector<roadSign> ListOfOvertakingSigns;
    vector<roadSign> ListOfNoEntrySigns;
    vector<roadSign> ListOfStartSigns;
    vector<roadSign> ListOfOneWayStreetSigns;
    vector<roadSign> ListOfRoadworksSigns;
    vector<roadSign> ListOf50Signs;
    vector<roadSign> ListOf100Signs;

    /*! array of obnstacles on the map */
    tFloat32 m_allObstaclesX[5] = {0, 0, 0, 0, 0};
    tFloat32 m_allObstaclesY[5] = {0, 0, 0, 0, 0};
    tFloat32 m_allObstaclesRadius[5] = {0, 0, 0, 0, 0};

    /*! iterator for going through array of obstacles */
    tInt32 m_obstacleIterator;

    tInt32 m_currentSign;
    //because it could be that there is another sign right in front of the stop sign, for example,
    // and I don't want my car to evaluate that sign until it is done evaluating the stop sign. So, I
    //store the STOP sign in the above variable and I won't update it until I have evaluated the STOP sign

    tInt16 m_currentManeuver;

    tFloat32 m_currentTime;
    tFloat32 m_originalTime;
    tFloat32 m_previousTimeStamp;

    tFloat32 m_previousPosX;
    tFloat32 m_previousPosY;

    tInt32 m_NumberOfManeuvers;

    tBool m_gettingPosForFirstTime;

    tBool m_recordedTime;
    tBool m_getReferenceTime;

    tBool m_noOvertaking;

    tBool m_lowerTheSpeed;

    tBool emergencyReverse;

    tBool getValueForPullOut; //When I pull out, I want to pull straight out of my current position by exactly 1 meters
    tFloat32 PullOutCoordinateX;
    tFloat32 PullOutCoordinateY;

    list_tuple_t path;

public:

    /*! Default constructor. */
    cSpeedController();

    /*! Destructor. */
    virtual ~cSpeedController() = default;

    /**
    * Overwrites the Configure
    * This is to Read Properties prepare your Trigger Function
    */
    tResult Configure() override;
    /**
    * Overwrites the Process
    * You need to implement the Reading and Writing of Samples within this function
    * MIND: Do Reading until the Readers queues are empty or use the IPinReader::GetLastSample()
    * This FUnction will be called if the Run() of the TriggerFunction was called.
    */
    tResult Process(tTimeStamp tmTimeOfTrigger) override;
    tResult TransmitSpeed(tSignalValue speedSignal);
    tResult TransmitSteering(tSignalValue steeringSignal);

    tResult ToggleHeadLights(tBoolSignalValue lightsSignal);
    tResult ToggleBrakeLights(tBoolSignalValue lightsSignal);
    tResult ToggleReverseLights(tBoolSignalValue lightsSignal);
    tResult ToggleHazardLights(tBoolSignalValue lightsSignal);
    tResult ToggleLeftLights(tBoolSignalValue lightsSignal);
    tResult ToggleRightLights(tBoolSignalValue lightsSignal);

    /*! function which transmits the state
    * \param stateID state to be sent; -1: error, 0: Ready, 1: Running
    * \param i16ManeuverEntry current entry to be sent
    */
    tResult OnSendState(aadc::jury::stateCar stateID, tInt16 i16ManeuverEntry);
    tResult TransmitDriverStruct(tDriverStruct& driverStruct);

    /*! this functions loads the maneuver list given in the properties
    * \result Returns a standard result code.
    */
    tResult LoadManeuverList();

    /*!
     * Process the road sign file.
     *
     * \param   tmTimeOfTrigger The time time of trigger.
     * \param   sample          The sample.
     */
    tResult ProcessRoadSignFile(tInt64 tmTimeOfTrigger, const ISample& sample);

    /*!
     * Parse the road sign file.
     *
     * \param   oDOM    The dom.
     */
    tResult ParseRoadSignFile(cDOM& oDOM);

    void CheckDistanceToObs(std::vector<tPolarCoordiante>& scan);
// Brain stuff
private:
    void LoadBrain();
    void TestBrain();
    std::unique_ptr<tensorflow::Session> session;
public:
    float* GetSpeedVector(tensorflow::Input::Initializer state, float steering_angle, float speed, float speed_upper_limit);
    const float seconds_per_step = 0.1; // seconds
    const float max_steering_angle = 30*M_PI/100; // degrees
    const float max_acceleration = 0.7; // m/s^2
    const float min_speed = 1.0; // m/s
    const float max_speed = 1.4;
    const float speed_limit = 1.0; // m/s
// Path Generator suff
private:
    void LoadPathGenerator();
    void TestPathGenerator();
    ffpython_t python_interpreter;
    float old_car_progress;
    bool plan_new_path = false;
    bool path_ready = false;
    tuple_t test_car_point = {67.20,35.03};
    float test_car_rotation = 1.57f;
    time_point last_controller_call_time = Time::now();

public:
    list_tuple_t GeneratePathFromPoint(tuple_t point, string direction);
    list_tuple_t GeneratePathFromPointAndAction(int action_id, tuple_t point, string direction);
    void SetupSplinesForControlPointGeneration(list_tuple_t path);
    list_tuple_t GetControlPoints(tuple_t point, float rotation, float old_progress);
    tuple_t GetClosestRoadPoint(tuple_t point, float old_progress);
    float GetPathProgress(tuple_t point, float old_progress);
    float GetPathCompletionPercentage(tuple_t point, float old_progress); // in [0,100]
    void SetupPath(tuple_t car_point, string car_direction);
};


//*************************************************************************************************
