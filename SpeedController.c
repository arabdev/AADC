/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

//THIS CODE WAS POSSIBLE DUE TO THE TEMPLATE PROVIDED BY AUDI. SARIM MEHDI, TONG LIU AND FRANCESCO SOVRANO
//MODIFIED THE CODE FOR THEIR SPECIFIC PURPOSE

**********************************************************************/
#include "stdafx.h"
#include "SpeedController.h"

ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_TEMPLATEFILTER_DATA_TRIGGERED_FILTER,
                                    "SpeedController",
                                    cSpeedController,
                                    adtf::filter::pin_trigger({"ultrasonic_struct"}));


cSpeedController::cSpeedController()
{

    //the us struct
    object_ptr<IStreamType> pTypeUSData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tUltrasonicStruct", pTypeUSData, m_USDataSampleFactory))
    {
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tSideLeft") + cString(".ui32ArduinoTimestamp"), m_ddlUltrasonicStructIndex.SideLeft.timeStamp));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tSideLeft") + cString(".f32Value"), m_ddlUltrasonicStructIndex.SideLeft.value));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tSideRight") + cString(".ui32ArduinoTimestamp"), m_ddlUltrasonicStructIndex.SideRight.timeStamp));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tSideRight") + cString(".f32Value"), m_ddlUltrasonicStructIndex.SideRight.value));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tRearLeft") + cString(".ui32ArduinoTimestamp"), m_ddlUltrasonicStructIndex.RearLeft.timeStamp));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tRearLeft") + cString(".f32Value"), m_ddlUltrasonicStructIndex.RearLeft.value));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tRearCenter") + cString(".ui32ArduinoTimestamp"), m_ddlUltrasonicStructIndex.RearCenter.timeStamp));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tRearCenter") + cString(".f32Value"), m_ddlUltrasonicStructIndex.RearCenter.value));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tRearRight") + cString(".ui32ArduinoTimestamp"), m_ddlUltrasonicStructIndex.RearRight.timeStamp));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tRearRight") + cString(".f32Value"), m_ddlUltrasonicStructIndex.RearRight.value));
    }
    else
    {
        LOG_INFO("No mediadescription for tUltrasonicStruct found!");
    }
    Register(m_oInputUltrasonicUnit, "ultrasonic_struct", pTypeUSData);

    object_ptr<IStreamType> pTypeLSData;
    if (ERR_NOERROR == adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tLaserScannerData", pTypeLSData, m_LSStructSampleFactory))
    {
        ////find the indices of the elements for faster access in the proces method.
        LOG_INFO("Found media description for tLaserScannerData!");
        //get all the member indices
        (adtf_ddl::access_element::find_index(m_LSStructSampleFactory, "ui32Size", m_ddlLSDataId.size));
        (adtf_ddl::access_element::find_array_index(m_LSStructSampleFactory, "tScanArray", m_ddlLSDataId.scanArray));
    }
    else {
        LOG_INFO("No media description for tLaserScannerData found!");
    }
    Register(m_oInputLaserScanner, "laser_scanner", pTypeLSData);

    //Get Media Descriptions
    object_ptr<IStreamType> pTypeJuryStruct;

    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tJuryStruct", pTypeJuryStruct, m_juryStructSampleFactory))
    {
     LOG_INFO("Found media description for tJuryStruct!");
        (adtf_ddl::access_element::find_index(m_juryStructSampleFactory, cString("i16ActionID"), m_ddlJuryStructId.actionId));
        (adtf_ddl::access_element::find_index(m_juryStructSampleFactory, cString("i16ManeuverEntry"), m_ddlJuryStructId.maneuverEntry));
    }
    else
    {
        LOG_INFO("No mediadescription for tJuryStruct found!");
    }
    Register(m_oInputJuryStruct, "jury_struct", pTypeJuryStruct);

    object_ptr<IStreamType> pTypeDriverStruct;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tDriverStruct", pTypeDriverStruct, m_driverStructSampleFactory))
    {
        LOG_INFO("Found media description for tDriverStruct!");
        (adtf_ddl::access_element::find_index(m_driverStructSampleFactory, cString("i16StateID"), m_ddlDriverStructId.stateId));
        (adtf_ddl::access_element::find_index(m_driverStructSampleFactory, cString("i16ManeuverEntry"), m_ddlDriverStructId.maneuverEntry));
    }
    else
    {
        LOG_INFO("No mediadescription for tDriverStruct found!");
    }
    Register(m_oOutputDriverStruct, "driver_struct", pTypeDriverStruct);

    object_ptr<IStreamType> pTypeDefault = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_anonymous());
    Register(m_oInputManeuverList, "maneuver_list", pTypeDefault);

    object_ptr<IStreamType> pTypeAnonymous = make_object_ptr<cStreamType>(stream_meta_type_anonymous());
    Register(m_roadSignMapData, "road_sign_map", pTypeAnonymous);

    object_ptr<IStreamType> pTypeBoolSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tBoolSignalValue", pTypeBoolSignalValue, m_BoolSignalValueSampleFactory))
    {
        (adtf_ddl::access_element::find_index(m_BoolSignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlBoolSignalValueId.ui32ArduinoTimestamp));
        (adtf_ddl::access_element::find_index(m_BoolSignalValueSampleFactory, cString("bValue")              , m_ddlBoolSignalValueId.bValue));
    }
    else
    {
        LOG_INFO("No mediadescription for tBoolSignalValue found!");
    }
    Register(m_oOutputHeadLight, "head_light", pTypeBoolSignalValue);
    Register(m_oOutputTurnLeft, "turn_signal_left", pTypeBoolSignalValue);
    Register(m_oOutputTurnRight, "turn_signal_right", pTypeBoolSignalValue);
    Register(m_oOutputBrakeLight, "brake_light", pTypeBoolSignalValue);
    Register(m_oOutputHazard, "hazard_light", pTypeBoolSignalValue);
    Register(m_oOutputReverseLight, "reverse_light", pTypeBoolSignalValue);

    //Add position pin from mediadescription
    adtf::ucom::object_ptr<adtf::streaming::IStreamType> pTypePositionControllerData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tPosition", pTypePositionControllerData, m_PositionControllerSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_PositionControllerSampleFactory, "f32x", m_ddlPositionControllerIndex.x);
        adtf_ddl::access_element::find_index(m_PositionControllerSampleFactory, "f32y", m_ddlPositionControllerIndex.y);
        adtf_ddl::access_element::find_index(m_PositionControllerSampleFactory, "f32radius", m_ddlPositionControllerIndex.radius);
        adtf_ddl::access_element::find_index(m_PositionControllerSampleFactory, "f32speed", m_ddlPositionControllerIndex.speed);
        adtf_ddl::access_element::find_index(m_PositionControllerSampleFactory, "f32heading", m_ddlPositionControllerIndex.heading);
    }
    else
    {
        LOG_WARNING("No mediadescription for tPosition found!");
    }
    Register(m_oReaderPosForController, "position", pTypePositionControllerData);

    //the roadsignext struct
    object_ptr<IStreamType> pTypeRoadSignData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tRoadSignExt", pTypeRoadSignData, m_RoadSignSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_RoadSignSampleFactory, "i16Identifier", m_ddlRoadSignIndex.id);
        adtf_ddl::access_element::find_index(m_RoadSignSampleFactory, "f32Imagesize", m_ddlRoadSignIndex.size);
        adtf_ddl::access_element::find_index(m_RoadSignSampleFactory, "af32TVec", m_ddlRoadSignIndex.tvec);
        adtf_ddl::access_element::find_index(m_RoadSignSampleFactory, "af32RVec", m_ddlRoadSignIndex.rvec);
    }
    else
    {
        LOG_WARNING("No mediadescription for tRoadSignExt found!");
    }

    Register(m_oReaderRoadSign, "road_sign_ext", pTypeRoadSignData);

    object_ptr<IStreamType> pTypeSpeed;
    if (ERR_NOERROR == adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSpeed, m_SpeedFactory))
    {
        adtf_ddl::access_element::find_index(m_SpeedFactory, cString("f32Value"), m_ddlSpeed.value);
        adtf_ddl::access_element::find_index(m_SpeedFactory, cString("ui32ArduinoTimestamp"), m_ddlSpeed.timeStamp);
    }
    else
    {
        LOG_WARNING("No mediadescription for tSignalvalue found!");
    }

    Register(m_oSpeed, "speed", pTypeSpeed);

    object_ptr<IStreamType> pTypeSteering;
    if (ERR_NOERROR == adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSteering, m_SteeringFactory))
    {
        adtf_ddl::access_element::find_index(m_SteeringFactory, cString("f32Value"), m_ddlSteering.value);
        adtf_ddl::access_element::find_index(m_SteeringFactory, cString("ui32ArduinoTimestamp"), m_ddlSteering.timeStamp);
    }
    else
    {
        LOG_WARNING("No mediadescription for tSignalvalue found!");
    }

    Register(m_oSteering, "steering", pTypeSteering);

    object_ptr<IStreamType> pTypeLaneCheck;
    if (ERR_NOERROR == adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeLaneCheck, m_LaneCheckFactory))
    {
        adtf_ddl::access_element::find_index(m_LaneCheckFactory, cString("f32Value"), m_ddlLaneCheck.value);
        adtf_ddl::access_element::find_index(m_LaneCheckFactory, cString("ui32ArduinoTimestamp"), m_ddlLaneCheck.timeStamp);
    }
    else
    {
        LOG_WARNING("No mediadescription for tSignalvalue found!");
    }

    Register(m_oLaneCheck, "Lane_Check" , pTypeLaneCheck);

    object_ptr<IStreamType> pTypeObjectCheck;
    if (ERR_NOERROR == adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeObjectCheck, m_ObjectCheckFactory))
    {
        adtf_ddl::access_element::find_index(m_ObjectCheckFactory, cString("f32Value"), m_ddlObjectCheck.value);
        adtf_ddl::access_element::find_index(m_ObjectCheckFactory, cString("ui32ArduinoTimestamp"), m_ddlObjectCheck.timeStamp);
    }
    else
    {
        LOG_WARNING("No mediadescription for tSignalvalue found!");
    }

    Register(m_oObjectCheck, "Object_Check" , pTypeObjectCheck);

    //register properties
    RegisterPropertyVariable("How far should your car be from a sign (meters)", m_rangeOfSign);
    RegisterPropertyVariable("How long you stop at an intersection waiting for other vehicle", m_timeToCheckStopSign);
    RegisterPropertyVariable("How long you stop at an intersection for pedestrian", m_timeToCheckPedestrian);
    RegisterPropertyVariable("Upper limit of speed in meters per second", m_speedUpperLimit);
    RegisterPropertyVariable("Min distance Obstacle can be from the sides and rear of car", m_propUSDistanceObs);

    m_evaluatingSign = false;
    m_readyToPark = false;
    m_noOvertaking = false;
    m_lowerTheSpeed = false;
    m_reverse = false;

    //simply convenient to initialize all time values here
    m_currentTime = 0;
    m_getReferenceTime = false;
    m_previousTimeStamp = 0;

    //in the beginning, I have no sign information
    m_currentSign = -1;

    //This is simply the rotation of the car (in degrees) from the positive x-axis
    m_orientation = 0;

    //This directly corresponds to the state. So, for example, if the state is 0, the m_currentSpline is
    //0 and so I will use the spline equation of the road that goes from state 0 to state 1.
    //    m_currentSpline = 0;

    m_recordedTime = false;

    m_previousPosX = 0;
    m_previousPosY = 0;

    m_gettingPosForFirstTime = true;

    m_steeringAngle = 0; m_currentSpeed = 0;

    m_obstacleIterator = 0;

    m_currentManeuver = manuever_undefined;

    // tong, load tf resources
    LoadBrain();
    LoadPathGenerator();

    loadedManeuverList = false;

    loadedRoadSignsList = false;

    executingManeuver = false;

    m_sectorIterator = 0; m_maneuverIterator = 0;
    m_NumberOfManeuvers = 0;

    getPosition = false;
    getValueForPullOut = false; PullOutCoordinateX = 9999; PullOutCoordinateY = 9999;
}


//implement the Configure function to read ALL Properties
tResult cSpeedController::Configure()
{
    RETURN_NOERROR;
}


// Reads a model graph definition from disk, and creates a session object you
// can use to run it.
Status LoadGraph(const string& graph_file_name, std::unique_ptr<tensorflow::Session>* session) {
    tensorflow::GraphDef graph_def;
    Status load_graph_status =
            ReadBinaryProto(tensorflow::Env::Default(), graph_file_name, &graph_def);
    if (!load_graph_status.ok()) {
        return tensorflow::errors::NotFound("Failed to load compute graph at '",
                                            graph_file_name, "'");
    }

    // CPU session options
    tensorflow::SessionOptions options = tensorflow::SessionOptions();
    /*tensorflow::ConfigProto* config = &options.config;
  // disabled GPU entirely
  (*config->mutable_device_count())["GPU"] = 0;
  // place nodes somewhere
  config->set_allow_soft_placement(true);
  config->set_log_device_placement(false);*/

    session->reset(tensorflow::NewSession(options));

    Status session_create_status = (*session)->Create(graph_def);
    if (!session_create_status.ok()) {
        return session_create_status;
    }
    return Status::OK();
}

void cSpeedController::LoadPathGenerator()
{
    LOG_INFO("Loading path generator..");
    int argc = 1;
    char* argv[] = {"PathGenerator"};
    PySys_SetArgv(argc, argv);
    python_interpreter.add_path("/home/aadc/AADC/adtf-basic-software/src/aadcUser/Python/PathGenerator/py");
    python_interpreter.load("main");
    string map_file = "/home/aadc/AADC/adtf-basic-software/src/aadcUser/Python/PathGenerator/map/aadc2018#test#track#003.xodr"; // OpenDrive format 1.4
    string maneuvers_file = "/home/aadc/AADC/adtf-basic-software/src/aadcUser/Python/PathGenerator/maneuvers/example.xml";
    string sign_file = "/home/aadc/AADC/adtf-basic-software/src/aadcUser/Python/PathGenerator/map/signs_aadc_testevent2018.xml";
    //YOU DO NOT HAVE ACCESS TO THE MANEUVER LIST FRANCESCO. ADJUST YOUR CODE TO TAKE A VECTOR OF
    //INTEGERS INSTEAD. USE THAT ENUM STRUCT I PUT ON SLACK AS REFERENCE
    //I ALREADY HAVE THE VECTOR OF INTEGERS WHERE EACH INTEGER CORRESPONDS TO A MANEUVER
    //AND NOW I WILL PASS THAT VECTOR TO YOUR CODE HERE

    python_interpreter.call<void>("main", "init", map_file, maneuvers_file, sign_file);
    LOG_INFO("Path generator loaded");
    plan_new_path = true; // we have updated the maneuvers list, thus we have to make a new path plan
    // if maneuvers are updated at runtime, then need to set plan_new_path to true every time you update them
//    path_ready = false; // for debugging
}

void cSpeedController::TestPathGenerator()
{
    // Build test input
    tuple_t car_point = {67.20,35.03}; // car position in map. These are global coordinates.
    float car_rotation = 1.57f; // car rotation is the angle of the car with respect to map, it's like Nord/Sud/Ovest/Est coordinates
    string car_direction = "forward"; // "forward": follow road successors; "backward": follow road predecessor

    // Generate path. A path is a list of roads/splines
    LOG_WARNING("Building path");
    list_tuple_t path = GeneratePathFromPoint(car_point, car_direction);
    LOG_WARNING("Generated path is:");
    for (list_tuple_t::iterator it = path.begin(); it != path.end(); ++it)
    {
        tuple_t info = *it;
        LOG_WARNING(cString::Format("\tU: <%f,%f,%f,%f>, V: <%f,%f,%f,%f>, origin: <%f,%f>, rotation: <%f>, length: <%f>", info[0], info[1], info[2], info[3], info[4], info[5], info[6], info[7], info[8], info[9], info[10], info[11]));
    }

    // Setup splines for control point generation every time you change path!
    LOG_WARNING("Setup splines for control point generation.");
    SetupSplinesForControlPointGeneration(path);
    float old_car_progress = 0; // set to 0 after setup because car is automatically at the start of the path; set -1 if unknown; must be always lower than the real car progress in order to generate good results
    // Update old_car_progress every step using new car_progress. If you don't understand leave it to -1.

    // Get closest road point to car, knowing old car progress may speed up algorithm (a lot!)
    // NB: no need to know ClosestRoadPoint every step, here we use it just for debugging
    tuple_t closest_spline_point = GetClosestRoadPoint(car_point, old_car_progress);
    LOG_WARNING(cString::Format("Closest road point to car <%f,%f>\n", closest_spline_point[0],closest_spline_point[1]));

    // Do the following every step (every 0.1 seconds) beacuse it is used for controller input
    // Get car progress on path, knowing OLD car progress may speed up algorithm (a lot!)
    float car_progress = GetPathProgress(car_point, old_car_progress);
    LOG_WARNING(cString::Format("Car position on road %f\n", car_progress));
    // Generate control points
    list_tuple_t control_points = GetControlPoints(car_point, car_rotation, car_progress);
    LOG_WARNING("Generated control points:");
    for (list_tuple_t::iterator it = control_points.begin(); it != control_points.end(); ++it)
    {
        tuple_t info = *it;
        LOG_WARNING(cString::Format("\tpoint: <%f,%f>", info[0], info[1]));
    }
    // You can feed this control points directly in controller input
}

tuple_t cSpeedController::GetClosestRoadPoint(tuple_t point, float old_progress)
{
    return python_interpreter.call<tuple_t>("main", "get_closest_road_point", point, old_progress);
}

float cSpeedController::GetPathProgress(tuple_t point, float old_progress)
{
    return python_interpreter.call<float>("main", "get_path_progress", point, old_progress);
}

float cSpeedController::GetPathCompletionPercentage(tuple_t point, float old_progress) // in [0,100]
{
    return python_interpreter.call<float>("main", "get_normalized_path_progress", point, old_progress)*100;
}

list_tuple_t cSpeedController::GeneratePathFromPoint(tuple_t point, string direction="forward")
{
    return python_interpreter.call<list_tuple_t>("main", "get_path", point, direction);
}

list_tuple_t cSpeedController::GeneratePathFromPointAndAction(int action_id, tuple_t point, string direction="forward")
{ // be aware that last road of previous action is going to be the first road for this action
    string action;
    switch(action_id)
    {
    case maneuver_left:
    case maneuver_merge_left:
        action = "left";
        break;
    case maneuver_right:
    case maneuver_merge_right:
        action = "right";
        break;
    //case maneuver_pull_out_left:
    //case maneuver_pull_out_right:
     default:
        action = "straight";
        break;
    }

    return python_interpreter.call<list_tuple_t>("main", "get_path_by_action", action, point, direction);
}

void cSpeedController::SetupSplinesForControlPointGeneration(list_tuple_t path)
{
    python_interpreter.call<void>("main", "setup_splines_for_control_point_generation", path);
}

list_tuple_t cSpeedController::GetControlPoints(tuple_t point, float angle, float old_progress)
{
    return python_interpreter.call<list_tuple_t>("main", "get_control_points", point, angle, old_progress);
}

void cSpeedController::LoadBrain()
{ // do it once, when constructing the speed controller
    string graph =
            "/home/aadc/AADC/adtf-basic-software/src/aadcUser/SpeedController/data/frozen_model.pb";
    bool self_test = false;
    string root_dir = "";
    std::vector<Flag> flag_list = {
        Flag("graph", &graph, "graph to be executed"),
        Flag("self_test", &self_test, "run a self test"),
        Flag("root_dir", &root_dir,
        "interpret image and graph file names relative to this directory"),
    };

    // First we load and initialize the model.

    string graph_path = tensorflow::io::JoinPath(root_dir, graph);
    Status load_graph_status = LoadGraph(graph_path, &session);
    if (!load_graph_status.ok()) {
        LOG(ERROR) << load_graph_status;
        LOG_ERROR("Load graph brain not successful");
    }
    else
        LOG_INFO("Brain loaded");
}

float* cSpeedController::GetSpeedVector(tensorflow::Input::Initializer state, float steering_angle, float speed, float speed_upper_limit)
{
    std::initializer_list<float> car_info({steering_angle, speed, speed_limit}); // steering_angle, speed, seconds_per_step,speed_limit
    tensorflow::Input::Initializer concat({car_info});
    // Actually run the image through the model.
    std::vector<Tensor> outputs;
    // attention, the following line may generate a plugin description error if testBrain is misplaced
    Status run_status = session->Run({
                                         {"state", state.tensor}, {"concat", concat.tensor}},
    {"action"}, {}, &outputs);

    if (!run_status.ok()) {
        LOG(ERROR) << "Running model failed: " << run_status;
        return NULL;
    }
    auto output_array = outputs[0].tensor<float, 2>();
    // new steering angle is output[0][0] multiplied by maximum steering angle in degrees or radians, but remember that the input angle must be in radians.
    float new_steering_angle = output_array(0, 0)*max_steering_angle; // output[0][0] in [-1,1]
    // new speed is old_speed + output[0][1] * max. acceleration * seconds_per_step!
    float new_speed = speed + output_array(0, 1)*max_acceleration*seconds_per_step; // output[0][1] in [-1,1]
    float *pair = new float[2]{new_steering_angle, new_speed};
    return pair;
}

void cSpeedController::TestBrain() // how to feed, read me
{
    LOG_WARNING("Building brain test input");
    std::initializer_list<float> control1({0.15381115, -0.12868122,0.0});
    std::initializer_list<float> control2({0.32615253,-0.22782543,0.0});
    std::initializer_list<float> control3({0.49988018,-0.32687855,0.0});
    std::initializer_list<float> control4({0.67505567,-0.42566003,0.0});
    std::initializer_list<float> control5({0.85174059,-0.52398933,0.0});
    std::initializer_list<float> obstacle1({0.0,0.0,0.0});
    std::initializer_list<float> obstacle2({1.03225458,-0.622913,0.38705458});
    std::initializer_list<float> obstacle3({0.0,0.0,0.0});
    std::initializer_list<float> obstacle4({0.0,0.0,0.0});
    std::initializer_list<float> obstacle5({0.0,0.0,0.0});
    tensorflow::Input::Initializer state(
    {
                    {
                        {control1, control2, control3, control4, control5}, // must be 5
                        {obstacle1, obstacle2, obstacle3, obstacle4, obstacle5} // must be 5
                    }
                }
                );

    float steering_angle = 0.4897352883395582f; // in radians
    float speed = 1.4f; // m/s
    float speed_upper_limit = 1.0f; // in [0.7,1.4] m/s

    float* speed_vector = GetSpeedVector(state, steering_angle, speed, speed_upper_limit);
    if (speed_vector == NULL)
        return;

    LOG_WARNING("Test output");
    LOG_WARNING(cString::Format("new steering angle: %f°; new speed: %f m/s", speed_vector[0], speed_vector[1]));
}

void cSpeedController::SetupPath(tuple_t car_point, string car_direction="forward")
{
    // Generate path. A path is a list of roads/splines
    // N.B.: path generation is intended to be done once, not every step!ok nvm it i
    LOG_WARNING(cString::Format("Building new path from point <%f,%f>",car_point[0],car_point[1]));
    list_tuple_t path = GeneratePathFromPoint(car_point, car_direction);
    LOG_WARNING("Generated path is:");
    for (list_tuple_t::iterator it = path.begin(); it != path.end(); ++it)
    {
        tuple_t info = *it;
        LOG_WARNING(cString::Format("\tU: <%f,%f,%f,%f>, V: <%f,%f,%f,%f>, origin: <%f,%f>, rotation: <%f>, length: <%f>", info[0], info[1], info[2], info[3], info[4], info[5], info[6], info[7], info[8], info[9], info[10], info[11]));
    }
    // Setup splines for control point generation, only once! Do it after generating path. Generate path once for all, and not every step.
    LOG_WARNING("Setup splines for control point generation.");
    SetupSplinesForControlPointGeneration(path);//when manure change
    old_car_progress = 0;
}

tResult cSpeedController::Process(tTimeStamp tmTimeOfTrigger)
{
    time_point speedcontroller_t = Time::now();
    try
    {
        emergencyReverse = false;
        object_ptr<const ISample> pReadSample;

        tFloat32 m_timeDifferenceSign;     //how long I evaluate the stop sign

        tSignalValue speedSignal;
        tSignalValue steeringSignal;
        tBoolSignalValue lightsBool;

        tInt32 parkingID;                  //store the parking space ID here (parkingID must always be "extra" value - 1

        lightsBool.bValue = false;
        //The lane check signal receives values from the OpenCV Lane Detection filter. If it is 0, it means I
        //am correctly positioned in the lane. If it is 1, then I must be going out of my lane towards the left lane
        //(which is the right lane according to German traffic rules) and, therefore, I must turn right. Similarly, if the
        //laneCheckSignal is 2, then I must be going offroad (what does it mean going offroad while in right
        //lane? It simply means I am going too much to the right and about to get out of the road completely)
        //and I must turn left
        tSignalValue laneCheckSignal;
        tSignalValue objectCheckSignal;

        //Initialize Position values
        tFloat32 posX = 0, posY = 0, posHeading = 0;
        tFloat32 targetPosX = 9999; tFloat32 targetPosY = 9999;

        //Initialize speed and steering values
        speedSignal.f32Value = 0, steeringSignal.f32Value = 0;

        //A road sign object
        tRoadSignExt roadSign;

        //Initialize the road sign
        roadSign.i16Identifier = -1;

        //initialize the lane check signal
        laneCheckSignal.f32Value = 0;

        //initialize the object check signal
        objectCheckSignal.f32Value = 0;

        //in the beginning I want lane detection to be working
        m_ignoreLaneDetection = false;

        tuple_t car_point;
//        tuple_t car_point=test_car_point; // car point on map, in opendrive map coordinates
//        if (plan_new_path)
//        {
//            LOG_WARNING("Planning new path because maneuvers list has changed.");
//            plan_new_path = false;
//            SetupPath(test_car_point, "forward"); // "forward": follow road successors; "backward": follow road predecessor
//            path_ready = true;
//            //TestPathGenerator();
//        }

        if (IS_OK(m_oReaderPosForController.GetLastSample(pReadSample)))
        {
            //Get Position values
            auto oDecoder = m_PositionControllerSampleFactory.MakeDecoderFor(pReadSample);
            posX = adtf_ddl::access_element::get_value(oDecoder, m_ddlPositionControllerIndex.x);
            posY = adtf_ddl::access_element::get_value(oDecoder, m_ddlPositionControllerIndex.y);
            posHeading = adtf_ddl::access_element::get_value(oDecoder, m_ddlPositionControllerIndex.heading);

            LOG_INFO(cString::Format("%f is the x position of car", posX));
            LOG_INFO(cString::Format("%f is the y position of car", posY));
            LOG_INFO(cString::Format("%f is the current heading of the car", posHeading));

            /*So, what exactly am I doing here and why am I doing here? Well, you see, the problem I noticed
         * with the Positioning Filter provided by AUDI was that sometimes there would be an error and the
         * new position I will receive would be too far away from the car's current position. Our controller
         * relies heavily on position data. So, if such a large error in positioning occurs then it is not
         * hard to see how it could eventually build up and the car would go somewhere else! So, I simply tell
         * the car to make sure that the new position it gets from the positioning filter is not more than
         * 0.5 meters away from its previously recorded position (a safe bet in my opinion). If the new
         * position is more than 0.5, then the car rejects the new positions and continues with the previous
         * one.
         *
         * But what about m_gettingPosForFirstTime. Well, in the beginning I want to get the position. So,
         * I will not use this error-checking logic in the beginning.
         */
            // You should handle this positioning problems using bird eye view homography. You can generate control points directly from view.
            if (m_gettingPosForFirstTime)
            {
                m_previousPosX = posX;
                m_previousPosY = posY;
                m_gettingPosForFirstTime = false;
                getPosition = true;
            }

            //Here I compare the new position I get with the previous one. Notice that doing this for the
            //first time will give me a posDifferenceAdded = 0
            tFloat32 posDifferenceXSquared = (posX - m_previousPosX) * (posX - m_previousPosX);
            tFloat32 posDifferenceYSquared = (posY - m_previousPosY) * (posY - m_previousPosY);
            tFloat32 posDifferenceAdded = pow((posDifferenceXSquared + posDifferenceYSquared), 0.5);

            if (posDifferenceAdded >= 0.5)
            {
                posX = m_previousPosX;
                posY = m_previousPosY;
                LOG_INFO("New position is too far away, keeping old one");
            }
            else
            {
                m_previousPosX = posX;
                m_previousPosY = posY;
                LOG_INFO("This new position makes sense, so okay!");
            }

            car_point = {posX*8,posY*8}; // car point on map, in opendrive map coordinates
            if ((plan_new_path == true) && (loadedRoadSignsList == true))
            {
               plan_new_path = false;
                LOG_WARNING("Planning new path because maneuvers list has changed.");
                SetupPath(car_point, "forward"); // "forward": follow road successors; "backward": follow road predecessor
                path_ready = true;
            }
        }
        else
        {
            LOG_WARNING("I didn't get position!!");
        }


        object_ptr<const ISample> pSample;

        if (IS_OK(m_oInputJuryStruct.GetLastSample(pSample)))
        {
            auto oDecoder = m_juryStructSampleFactory.MakeDecoderFor(*pSample);
            RETURN_IF_FAILED(oDecoder.IsValid());
            tJuryStruct juryInput;
            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlJuryStructId.maneuverEntry, &juryInput.i16ManeuverEntry));
            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlJuryStructId.actionId, &juryInput.i16ActionID));

            tInt8 i8ActionID = juryInput.i16ActionID;
            tInt16 i16entry = juryInput.i16ManeuverEntry;

            switch (aadc::jury::juryAction(i8ActionID))
            {
            case action_getready:
                LOG_INFO(cString::Format("Driver Module: Received Request Ready with maneuver ID %d", i16entry));
                stateID = statecar_ready;
                m_Stop = true;
                break;
            case action_start:
                LOG_INFO(cString::Format("Driver Module: Received Run with maneuver ID %d", i16entry));
                stateID = statecar_running;
                m_Stop = false;
                break;
            case action_stop:
                LOG_INFO(cString::Format("Driver Module: Received Stop with maneuver ID %d", i16entry));
                stateID = statecar_ready;
                m_Stop = true;
                break;
            }
        }
        else
        {
            LOG_WARNING("NO JURY CONNECTED!");
        }

        //Load the Maneuver List here and check if maneuver list is empty or not
        if (!loadedManeuverList)
        {
//            if (IS_OK(m_oInputManeuverList.GetLastSample(pReadSample)))
//            {
//                LOG_INFO("NOW I GET MANEUVER LIST!!!!");
//                std::vector<tChar> data;
//                object_ptr_shared_locked<const ISampleBuffer> pSampleBuffer;
//                data.resize(pSampleBuffer->GetSize());
//                memcpy(data.data(), pSampleBuffer->GetPtr(), pSampleBuffer->GetSize());
//                if (data.size() > 0)
//                {//maneuverlist
//                    m_strManeuverFileString.Set(data.data(), data.size());
//                    LoadManeuverList();
//                }
//            }
//            else
//            {
//                LOG_WARNING("SOMETHING IS WRONG WITH READING MANEUVERS!!");
//            }
            tManeuver man;
            man.id = 0; man.action = maneuver_left; man.extra = 0;
            listOfManeuvers.push_back(man);
            loadedManeuverList = true;
        }

        if (!listOfManeuvers.empty())
        {
            vector<tManeuver>::iterator it;
            for (it = listOfManeuvers.begin(); it != listOfManeuvers.end(); it++)
            {
                int temp_id = it->id;
                int temp_action = it->action;
                int temp_extra = it->extra;
                LOG_INFO(cString::Format("Maneuver ID: %i", temp_id));
                LOG_INFO(cString::Format("Maneuver action: %i", temp_action));
                LOG_INFO(cString::Format("Maneuver extra (0 means no parking): %i", temp_extra));

                if ((temp_id == m_maneuverIterator) && (executingManeuver == false) && (loadedRoadSignsList == true))
                {
                    m_currentManeuver = temp_action;
                    executingManeuver = true;
                    parkingID = temp_extra;
                    GeneratePathFromPointAndAction(temp_action,car_point);
                    LOG_INFO(cString::Format("I am executing this maneuver ID: %i", temp_id));
                    LOG_INFO(cString::Format("I am executing this maneuver action: %i", temp_action));
                    LOG_INFO(cString::Format("I am executing this maneuver extra (0 means no parking): %i", temp_extra));

                    if((m_currentManeuver == maneuver_pull_out_left) || (m_currentManeuver == maneuver_pull_out_right))
                    {
                        m_reverse = true;
                    }
                }
            }
        }
        else
        {
            LOG_WARNING("THE MANEUVER LIST IS EMPTY, PLEASE LOAD A MANEUVER LIST FROM JURY MODULE!");
        }

        //Load the list of road signs and check if it is empty or not and, for troubleshooting purposes,
        //print everything in the list of road signs
        if (!loadedRoadSignsList)
        {
            if (IS_OK(m_roadSignMapData.GetNextSample(pReadSample)))
            {
                // get and store all the road signs
                ProcessRoadSignFile(tmTimeOfTrigger, *pReadSample);
            }
        }
        if (!m_roadSigns.empty())
        {
            vector<_roadSign>::iterator it;
            for (it = m_roadSigns.begin(); it != m_roadSigns.end(); it++)
            {
                tInt16 temp_id = it->u16Id;
                tFloat32 temp_X = it->f32X;
                tFloat32 temp_Y = it->f32Y;
                tFloat32 temp_direction = it->f32Direction;
//                LOG_INFO(cString::Format("Road Sign ID: %i", temp_id));
//                LOG_INFO(cString::Format("Road Sign X: %f", temp_X));
//                LOG_INFO(cString::Format("Road Sign Y: %f", temp_Y));
//                LOG_INFO(cString::Format("Road Sign Direction: %f", temp_direction));
            }
        }
        else
        {
            LOG_WARNING("THE ROAD SIGN LIST IS EMPTY, PLEASE LOAD A ROAD SIGN LIST FROM JURY MODULE!");
        }

        object_ptr<const ISample> pReadSampleLS;

        time_point laserscanner_t = Time::now();
        if(IS_OK(m_oInputLaserScanner.GetLastSample(pReadSampleLS)))
        {
            auto oDecoder = m_LSStructSampleFactory.MakeDecoderFor(*pReadSampleLS);

            //RETURN_IF_FAILED(oDecoder.IsValid());
            tSize numOfScanPoints = 0;
            //RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlLSDataId.size, &numOfScanPoints));

            const tPolarCoordiante* pCoordinates = reinterpret_cast<const tPolarCoordiante*>(oDecoder.GetElementAddress(m_ddlLSDataId.scanArray));

            std::vector<tPolarCoordiante> scan;
            tPolarCoordiante scanPoint;

            for (tSize i = 0; i <numOfScanPoints; i++)
            {
                scanPoint.f32Radius = pCoordinates[i].f32Radius;
                scanPoint.f32Angle = pCoordinates[i].f32Angle;
                scan.push_back(scanPoint);
            }

            //init with some max value
            CheckDistanceToObs(scan);
        }
        LOG_WARNING(cString::Format("LaserScanner takes %f seconds\n", fsec(Time::now() - laserscanner_t).count()));

        time_point roadsignreader_t = Time::now();
        if (IS_OK(m_oReaderRoadSign.GetLastSample(pReadSample)))
        {
            auto oDecoder = m_RoadSignSampleFactory.MakeDecoderFor(*pReadSample);

            RETURN_IF_FAILED(oDecoder.IsValid());

            // retrieve the values (using convenience methods that return a variant)
            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlRoadSignIndex.id, &roadSign.i16Identifier));
        }

        //If I am not evaluating any sign I will get the sign value from the road sign that I see.
        //Otherwise, I will keep the current sign which is a global variable and doesn't reset when the Process
        //function is executed every time unit
        if ((m_roadSigns.empty() == false) && (listOfManeuvers.empty() == false))
        {
            if (!m_evaluatingSign)
            {
                m_currentSign = roadSign.i16Identifier;
                LOG_INFO(cString::Format("The sign in front of car has id %i", roadSign.i16Identifier));
            }
        }
        LOG_WARNING(cString::Format("ReaderRoadSign takes %f seconds\n", fsec(Time::now() - roadsignreader_t).count()));

        //This is where I get the value from the Lane Detection filter. 0 means okay, 1 means turn right
        //and 2 means turn left
        if (IS_OK(m_oLaneCheck.GetLastSample(pReadSample)))
        {
            auto oDecoder = m_LaneCheckFactory.MakeDecoderFor(*pReadSample);

            RETURN_IF_FAILED(oDecoder.IsValid());

            // retrieve the values (using convenience methods that return a variant)
            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlLaneCheck.value, &laneCheckSignal.f32Value));
        }

        //Following is what each number of objectCheckSignal.f32Value corresponds to:
        //1 - Emergency Car
        //2 - Car
        //3 - Adult
        //4 - Child
        time_point objectcheck_t = Time::now();
        if (IS_OK(m_oObjectCheck.GetLastSample(pReadSample)))
        {
            auto oDecoder = m_ObjectCheckFactory.MakeDecoderFor(*pReadSample);

            //RETURN_IF_FAILED(oDecoder.IsValid());

            // retrieve the values (using convenience methods that return a variant)
            //RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlObjectCheck.value, &objectCheckSignal.f32Value));
            oDecoder.GetElementValue(m_ddlObjectCheck.value, &objectCheckSignal.f32Value);
        }
        LOG_WARNING(cString::Format("ObjectCheck takes %f seconds\n", fsec(Time::now() - objectcheck_t).count()));

        //You will notice I am always doing m_orientation = listOfID.... etc. to get some value
        //This value is the angle of rotation of car wrt the positive x axis. So, for example:
        //listOfID0Heading[signIterator] gives me the angle of rotation of car if it was directly
        //facing the sign. What does this mean? So, imagine the car is facing north and it sees a
        //sign to its right which is facing the road. So, ideally, the car makes an angle of 90 degrees
        //wrt the positive x-axis. But, in reality, the car has a slight rotation to the left or right
        //while facing the sign. So, to get accurate value of the car's rotation about the positive x-axis
        //I add this value (which I get from another filter and I store it in posHeading) to the angle value
        //I got from the array element I mentioned previously. The posHeading is basically the value of the heading
        //of the car wrt the vertical axis going straight through the car. It is +ve if the car is slightly rotated
        //towards the left and -ve for right.
        //
        //The posHeading is always added after I get out of this switch block because, imagine I am seeing no
        //sign but I still am driving around with my heading changing continuously, so I still need to keep
        //updating my angle of orientation ert +ve x-axis. Therefore, I will always add the posHeading outside
        //the switch statement as that makes more logical sense

        //A LITTLE DISCUSSION ABOUT HOW I AM HANDLING SIGNS:
        //So, this was something that really troubled yet fascinated me. There were two possible scnearios:
        //I stop right before the sign or I stop ahead of the sign and am unable to see the sign. I wanted
        //to come up with logic that could handle both cases flawlessly. So, here is what I do. I setup a
        //variable that is -1 when the car is initialized, that variable is m_currentSign. Now, if I am not
        //"evaluating" any sign, this m_currentSign will be updated whenever my car sees any sign. So, what
        //happens when I see a sign at an intersection?
        //
        //Well, first of all, I tell the car that I am evaluating a sign, so if my car sees any other sign by
        //mistake (or if it is unavoidable), then that sign won't be registered. So, once I see a sign, I need
        //to make sure that I am within the vicinity of the sign (which is basically a circle of 0.5 m radius
        //with the sign coordinate as center). For this, I iterated between all the signs of that particular
        //type in my map until I come across one which is closest to mine within 0.5 meters. So, now I execute
        //the logic related to that sign. Once I am done executing the logic related to the sign, I don't
        //tell the car I am done evaluating the sign. In fact, I wait until I am outside that circle of 0.5 m
        //radius.
        //
        //So, why wait? I noticed its necessity while coming up with the logic for the STOP sign. You are
        //supposed to wait for 2 seconds once you are close to a STOP sign. Now, once those 2 seconds are up
        //and I have to continue, had I set the m_evaluatingSign back to false right when I was still looking
        //at the STOP sign, it would become true again and I would be stuck right next to that STOP sign since
        //I am right behind it and it is always in my sight. So, it made more sense to wait until the car was
        //outside the circle of 0.5 m radius before telling the car that it was done evaluating that sign

        switch(m_currentSign)
        {
        case NOTHING:
            LOG_INFO("I don't see any sign!");
            break;

        case UNMARKED_INTERSECTION:
        {
            LOG_INFO("Saw the UNMARKED_INTERSECTION sign!");
            vector<_roadSign>::iterator it;
            for (it = ListOfUnmarkedSigns.begin(); it != ListOfUnmarkedSigns.end(); it++)
            {
                tFloat32 temp_X = it->f32X;
                tFloat32 temp_Y = it->f32Y;
                tFloat32 temp_direction = it->f32Direction;
                tFloat32 sqrPosX = pow((posX - temp_X),2);
                tFloat32 sqrPosY = pow((posY - temp_Y),2);
                tFloat32 sqrPosAdded = pow((sqrPosX + sqrPosY),0.5);
                if (sqrPosAdded <= m_rangeOfSign)
                {
                    LOG_INFO(cString::Format("%f is the distance of car from the nearest UNMARKED_INTERSECTION sign", sqrPosAdded));
                    m_orientation = temp_direction;
                    m_evaluatingSign = true;
                    //Check for vehicles in front of you on intersection.
                    if ((objectCheckSignal.f32Value == 1) || (objectCheckSignal.f32Value == 2))
                    {
                        m_Stop = true;
                    }
                    else
                    {
                        m_Stop = false;
                    }
                }
                else
                {
                    m_evaluatingSign = false;
                    m_currentSign = -1;
                }
            }
            break;
        }

        case STOP:
        {
            LOG_INFO("Saw the STOP sign!");
            vector<_roadSign>::iterator it;
            for (it = ListOfStopSigns.begin(); it != ListOfStopSigns.end(); it++)
            {
                tFloat32 temp_X = it->f32X;
                tFloat32 temp_Y = it->f32Y;
                tFloat32 temp_direction = it->f32Direction;
                tFloat32 sqrPosX = pow((posX - temp_X),2);
                tFloat32 sqrPosY = pow((posY - temp_Y),2);
                tFloat32 sqrPosAdded = pow((sqrPosX + sqrPosY),0.5);

                if (sqrPosAdded <= m_rangeOfSign)
                {
                    LOG_INFO(cString::Format("%f is the distance of car from the nearest STOP sign", sqrPosAdded));
                    m_orientation = temp_direction;
                    m_evaluatingSign = true;
                    if (!m_getReferenceTime)
                    {
                        m_previousTimeStamp = tFloat32(laneCheckSignal.ui32ArduinoTimestamp) * (0.000001);
                        m_getReferenceTime = true;
                    }

                    m_timeDifferenceSign = (tFloat32(laneCheckSignal.ui32ArduinoTimestamp) * (0.000001)) - m_previousTimeStamp;
                    if (m_timeDifferenceSign >= m_timeToCheckStopSign)
                    {
                        LOG_INFO("I have stayed at stop sign for too long!");
                        m_Stop = false;
                    }

                    if ((objectCheckSignal.f32Value == 1) || (objectCheckSignal.f32Value == 2))
                    {
                        m_Stop = true;
                    }
                    //Check for vehicle at intersection by stopping for an arbitrary amount of time
                    //if there is a vehicle
                }
                else
                {
                    m_evaluatingSign = false;
                    m_getReferenceTime = false;
                    m_currentSign = -1;
                }
            }
            break;
        }

            //Here I will once again not rely on the vision to park.
        case PARKING:
        {
            LOG_INFO("Saw the PARKING sign!");
            vector<_roadSign>::iterator it;
            for (it = ListOfParkingSigns.begin(); it != ListOfParkingSigns.end(); it++)
            {
                tFloat32 temp_X = it->f32X;
                tFloat32 temp_Y = it->f32Y;
                tFloat32 temp_direction = it->f32Direction;
                tFloat32 sqrPosX = pow((posX - temp_X),2);
                tFloat32 sqrPosY = pow((posY - temp_Y),2);
                tFloat32 sqrPosAdded = pow((sqrPosX + sqrPosY),0.5);

                if (sqrPosAdded <= m_rangeOfSign)
                {
                    LOG_INFO(cString::Format("%f is the distance of car from the nearest PARKING sign", sqrPosAdded));
                    m_orientation = temp_direction;
                    m_evaluatingSign = true;
                    if (m_currentManeuver == maneuver_cross_parking)
                    {
                        m_readyToPark = true;
                    }
                }
                else
                {
                    m_evaluatingSign = false;
                }
            }
            break;
        }

        case HAVE_WAY:
        {
            LOG_INFO("Saw the HAVE_WAY sign!");
            vector<_roadSign>::iterator it;
            for (it = ListOfHaveWaySigns.begin(); it != ListOfHaveWaySigns.end(); it++)
            {
                tFloat32 temp_X = it->f32X;
                tFloat32 temp_Y = it->f32Y;
                tFloat32 temp_direction = it->f32Direction;
                tFloat32 sqrPosX = pow((posX - temp_X),2);
                tFloat32 sqrPosY = pow((posY - temp_Y),2);
                tFloat32 sqrPosAdded = pow((sqrPosX + sqrPosY),0.5);

                if (sqrPosAdded <= m_rangeOfSign)
                {
                    LOG_INFO(cString::Format("%f is the distance of car from the nearest HAVEWAY sign", sqrPosAdded));
                    m_orientation = temp_direction;
                    //Check for vehicles in front of you on intersection.
                    if ((objectCheckSignal.f32Value == 1) || (objectCheckSignal.f32Value == 2))
                    {
                        m_Stop = true;
                    }
                    else
                    {
                        m_Stop = false;
                    }
                }
                else
                {
                    m_evaluatingSign = false;
                    m_currentSign = -1;
                }
            }
            break;
        }

        case AHEAD_ONLY:
        {
            LOG_INFO("Saw the AHEAD_ONLY sign!");
            vector<_roadSign>::iterator it;
            for (it = ListOfStraightSigns.begin(); it != ListOfStraightSigns.end(); it++)
            {
                tFloat32 temp_X = it->f32X;
                tFloat32 temp_Y = it->f32Y;
                tFloat32 temp_direction = it->f32Direction;
                tFloat32 sqrPosX = pow((posX - temp_X),2);
                tFloat32 sqrPosY = pow((posY - temp_Y),2);
                tFloat32 sqrPosAdded = pow((sqrPosX + sqrPosY),0.5);

                if (sqrPosAdded <= m_rangeOfSign)
                {
                    LOG_INFO(cString::Format("%f is the distance of car from the nearest AHEAD ONLY sign", sqrPosAdded));
                    m_orientation = temp_direction;
                }
            }
            break;
        }

        case GIVE_WAY:
        {
            LOG_INFO("Saw the GIVE_WAY sign!");
            vector<_roadSign>::iterator it;
            for (it = ListOfGiveWaySigns.begin(); it != ListOfGiveWaySigns.end(); it++)
            {
                tFloat32 temp_X = it->f32X;
                tFloat32 temp_Y = it->f32Y;
                tFloat32 temp_direction = it->f32Direction;
                tFloat32 sqrPosX = pow((posX - temp_X),2);
                tFloat32 sqrPosY = pow((posY - temp_Y),2);
                tFloat32 sqrPosAdded = pow((sqrPosX + sqrPosY),0.5);

                if (sqrPosAdded <= m_rangeOfSign)
                {
                    LOG_INFO(cString::Format("%f is the distance of car from the nearest GIVE WAY sign", sqrPosAdded));
                    m_orientation = temp_direction;
                    m_evaluatingSign = true;
                    //Check for vehicles in front of you on intersection.
                    if ((objectCheckSignal.f32Value == 1) || (objectCheckSignal.f32Value == 2))
                    {
                        m_Stop = true;
                    }
                    else
                    {
                        m_Stop = false;
                    }
                }
                else
                {
                    m_evaluatingSign = false;
                    m_currentSign = -1;
                }
            }
            break;
        }

            //I ignore lane detection while close to a pedestrian crossing because, as explained in even more
            //detail at the bottom, relying on the map for this small distance is much better than coding
            //some crazy vision related stuff just for a small segment like this.
        case PEDESTRIAN_CROSSING:
        {
            LOG_INFO("Saw the PEDESTRIAN_CROSSING sign!");
            vector<_roadSign>::iterator it;
            for (it = ListOfPedestrianSigns.begin(); it != ListOfPedestrianSigns.end(); it++)
            {
                tFloat32 temp_X = it->f32X;
                tFloat32 temp_Y = it->f32Y;
                tFloat32 temp_direction = it->f32Direction;
                tFloat32 sqrPosX = pow((posX - temp_X),2);
                tFloat32 sqrPosY = pow((posY - temp_Y),2);
                tFloat32 sqrPosAdded = pow((sqrPosX + sqrPosY),0.5);

                if (sqrPosAdded <= m_rangeOfSign)
                {
                    LOG_INFO(cString::Format("%f is the distance of car from the nearest PEDESTRIAN sign", sqrPosAdded));
                    m_orientation = temp_direction;
                    m_evaluatingSign = true;

                    if ((objectCheckSignal.f32Value == 3) || (objectCheckSignal.f32Value == 4))
                    {
                        m_Stop = true;
                        if (!m_getReferenceTime)
                        {
                            m_previousTimeStamp = tFloat32(laneCheckSignal.ui32ArduinoTimestamp) * (0.000001);
                            m_getReferenceTime = true;
                        }

                        m_timeDifferenceSign = (tFloat32(laneCheckSignal.ui32ArduinoTimestamp) * (0.000001)) - m_previousTimeStamp;
                        if (m_timeDifferenceSign >= m_timeToCheckPedestrian)
                        {
                            LOG_INFO("I have stayed at pedestrian sign for too long!");
                            m_Stop = false;
                        }
                    }
                    else
                    {
                        m_Stop = false;
                    }
                    //Check for vehicle at intersection by stopping for an arbitrary amount of time
                    //If there is a vehicle
                }
                else
                {
                    m_evaluatingSign = false;
                    m_getReferenceTime = false;
                    m_currentSign = -1;
                }
            }
            break;
        }

        case ROUNDABOUT:
        {
            LOG_INFO("Saw the ROUNDABOUT sign!");
            vector<_roadSign>::iterator it;
            for (it = ListOfRoundaboutSigns.begin(); it != ListOfRoundaboutSigns.end(); it++)
            {
                tFloat32 temp_X = it->f32X;
                tFloat32 temp_Y = it->f32Y;
                tFloat32 temp_direction = it->f32Direction;
                tFloat32 sqrPosX = pow((posX - temp_X),2);
                tFloat32 sqrPosY = pow((posY - temp_Y),2);
                tFloat32 sqrPosAdded = pow((sqrPosX + sqrPosY),0.5);

                if (sqrPosAdded <= m_rangeOfSign)
                {
                    LOG_INFO(cString::Format("%f is the distance of car from the nearest ROUNDABOUT sign", sqrPosAdded));
                    m_orientation = temp_direction;
                }
            }
            break;
        }

        case NO_OVERTAKING:
        {
            LOG_INFO("Saw the NO_OVERTAKING sign!");
            vector<_roadSign>::iterator it;
            for (it = ListOfOvertakingSigns.begin(); it != ListOfOvertakingSigns.end(); it++)
            {
                tFloat32 temp_X = it->f32X;
                tFloat32 temp_Y = it->f32Y;
                tFloat32 temp_direction = it->f32Direction;
                tFloat32 sqrPosX = pow((posX - temp_X),2);
                tFloat32 sqrPosY = pow((posY - temp_Y),2);
                tFloat32 sqrPosAdded = pow((sqrPosX + sqrPosY),0.5);

                if (sqrPosAdded <= m_rangeOfSign)
                {
                    LOG_INFO(cString::Format("%f is the distance of car from the nearest NO OVERTAKING sign", sqrPosAdded));
                    m_orientation = temp_direction;
                    m_noOvertaking = true;
                    //Here I will just ignore collision detection because that will surely involve
                    //changing lanes. So, if I encounter a slow moving car in my lane, I will simply
                    //let the emergency brake filter do its thing
                }
            }
            break;
        }

        case NO_ENTRY_VEHICULAR_TRAFFIC:
        {
            LOG_INFO("Saw the NO_ENTRY_VEHICULAR_TRAFFIC sign!");
            vector<_roadSign>::iterator it;
            for (it = ListOfNoEntrySigns.begin(); it != ListOfNoEntrySigns.end(); it++)
            {
                tFloat32 temp_X = it->f32X;
                tFloat32 temp_Y = it->f32Y;
                tFloat32 temp_direction = it->f32Direction;
                tFloat32 sqrPosX = pow((posX - temp_X),2);
                tFloat32 sqrPosY = pow((posY - temp_Y),2);
                tFloat32 sqrPosAdded = pow((sqrPosX + sqrPosY),0.5);

                if (sqrPosAdded <= m_rangeOfSign)
                {
                    LOG_INFO(cString::Format("%f is the distance of car from the nearest NO ENTRY VEHICULAR TRAFFIC sign", sqrPosAdded));
                    m_orientation = temp_direction;
                }
            }
            break;
        }

        case STARTING_POSITION:
        {
            LOG_INFO("Saw the STARTING_POSITION sign!");
            vector<_roadSign>::iterator it;
            for (it = ListOfStartSigns.begin(); it != ListOfStartSigns.end(); it++)
            {
                tFloat32 temp_X = it->f32X;
                tFloat32 temp_Y = it->f32Y;
                tFloat32 temp_direction = it->f32Direction;
                tFloat32 sqrPosX = pow((posX - temp_X),2);
                tFloat32 sqrPosY = pow((posY - temp_Y),2);
                tFloat32 sqrPosAdded = pow((sqrPosX + sqrPosY),0.5);
                LOG_INFO(cString::Format("%f is the distance of car from a STARTING sign", sqrPosAdded));
                if (sqrPosAdded <= m_rangeOfSign)
                {
                    LOG_INFO(cString::Format("%f is the distance of car from the nearest STARTING sign", sqrPosAdded));
                    m_orientation = temp_direction;
                }
            }
            break;
        }

        case ONEWAY_STREET:
        {
            LOG_INFO("Saw the ONEWAY_STREET sign!");
            vector<_roadSign>::iterator it;
            for (it = ListOfOneWayStreetSigns.begin(); it != ListOfOneWayStreetSigns.end(); it++)
            {
                tFloat32 temp_X = it->f32X;
                tFloat32 temp_Y = it->f32Y;
                tFloat32 temp_direction = it->f32Direction;
                tFloat32 sqrPosX = pow((posX - temp_X),2);
                tFloat32 sqrPosY = pow((posY - temp_Y),2);
                tFloat32 sqrPosAdded = pow((sqrPosX + sqrPosY),0.5);

                if (sqrPosAdded <= m_rangeOfSign)
                {
                    LOG_INFO(cString::Format("%f is the distance of car from the nearest ONE WAY STREET sign", sqrPosAdded));
                    m_orientation = temp_direction;
                    m_noOvertaking = true;
                    //Same as before where you weren't allowed to change lanes
                }
            }
            break;
        }

        case ROADWORKS:
        {
            LOG_INFO("Saw the ROADWORKS sign!");
            vector<_roadSign>::iterator it;
            for (it = ListOfRoadworksSigns.begin(); it != ListOfRoadworksSigns.end(); it++)
            {
                tFloat32 temp_X = it->f32X;
                tFloat32 temp_Y = it->f32Y;
                tFloat32 temp_direction = it->f32Direction;
                tFloat32 sqrPosX = pow((posX - temp_X),2);
                tFloat32 sqrPosY = pow((posY - temp_Y),2);
                tFloat32 sqrPosAdded = pow((sqrPosX + sqrPosY),0.5);

                if (sqrPosAdded <= m_rangeOfSign)
                {
                    LOG_INFO(cString::Format("%f is the distance of car from the nearest ROADWORKS sign", sqrPosAdded));
                    m_orientation = temp_direction;
                    //Use collision detection to avoid roadworks
                }
            }
            break;
        }

        case SPEEDLIMIT_50:
        {
            LOG_INFO("Saw the SPEEDLIMIT_50 sign!");
            vector<_roadSign>::iterator it;
            for (it = ListOf50Signs.begin(); it != ListOf50Signs.end(); it++)
            {
                tFloat32 temp_X = it->f32X;
                tFloat32 temp_Y = it->f32Y;
                tFloat32 temp_direction = it->f32Direction;
                tFloat32 sqrPosX = pow((posX - temp_X),2);
                tFloat32 sqrPosY = pow((posY - temp_Y),2);
                tFloat32 sqrPosAdded = pow((sqrPosX + sqrPosY),0.5);

                if (sqrPosAdded <= m_rangeOfSign)
                {
                    LOG_INFO(cString::Format("%f is the distance of car from the nearest SPEEDLIMIT 50 sign", sqrPosAdded));
                    m_orientation = temp_direction;
                    m_lowerTheSpeed = true;
                }
            }
            break;
        }

        case SPEEDLIMIT_100:
        {
            LOG_INFO("Saw the SPEEDLIMIT_100 sign!");
            vector<_roadSign>::iterator it;
            for (it = ListOf100Signs.begin(); it != ListOf100Signs.end(); it++)
            {
                tFloat32 temp_X = it->f32X;
                tFloat32 temp_Y = it->f32Y;
                tFloat32 temp_direction = it->f32Direction;
                tFloat32 sqrPosX = pow((posX - temp_X),2);
                tFloat32 sqrPosY = pow((posY - temp_Y),2);
                tFloat32 sqrPosAdded = pow((sqrPosX + sqrPosY),0.5);

                if (sqrPosAdded <= m_rangeOfSign)
                {
                    LOG_INFO(cString::Format("%f is the distance of car from the nearest SPEEDLIMIT 100 sign", sqrPosAdded));
                    m_orientation = temp_direction;
                    m_lowerTheSpeed = false;
                }
            }
            break;
        }
        }

        //This is where I always make sure to get the accurate value of the car's angle wrt
        //+ve x-axis by adding the heading value I get from the marker filter
        //This is for when my car's angle of rotation becomes greater than 360 or lower than 0 degrees which can indeed happen
//        m_orientation = remainder(posHeading+m_steeringAngle,2*M_PI); // in [0,360]
        m_orientation = posHeading;

        //Fool the car into thinking it is looking out of the parking space
        //So, just add 180 degrees to current orientation
        if(m_reverse)
        {
            m_orientation = remainder(m_orientation+M_PI,2*M_PI);
        }

        LOG_INFO(cString::Format("%f is the orientation of my car", m_orientation));

        //********************THIS IS WHERE SARIM'S CONTROLLER STARTS********************//

        //    if (xDiff == 0)
        //    {
        //        angleToTarget = 0;
        //    }
        //    else
        //    {
        //        angleToTarget = atan(yDiff / xDiff);
        //    }

        //    LOG_INFO(cString::Format("%f is the angle my car makes with goal", angleToTarget));

        //    if (angleToTarget > 0)
        //    {
        //        LOG_INFO("steer to your right!");
        //        steeringSignal.f32Value = 30;
        //    }
        //    else if (angleToTarget < 0)
        //    {
        //        LOG_INFO("steer to your left!");
        //        steeringSignal.f32Value = -30;
        //    }

        //********************THIS IS WHERE SARIM'S CONTROLLER ENDS********************//

        //********************THIS IS WHERE FRANCESCO'S'S CONTROLLER STARTS********************//
//        float* speed_vector;
        fsec delta_time = (Time::now() - last_controller_call_time);
        float passed_seconds = delta_time.count();

        if (!path_ready)
            LOG_ERROR("Path is not ready");

//        LOG_WARNING(cString::Format("Seconds passed from last step: %f\n", passed_seconds));
        // Do the following every step (every "seconds_per_step" seconds) beacause it is used for controller input
        list_tuple_t control_points;
        if ( path_ready && (passed_seconds >= seconds_per_step) )
        {
            last_controller_call_time = Time::now();
            LOG_WARNING(cString::Format("Seconds passed from last step: %f\n", passed_seconds));
            float car_rotation = m_orientation;
           // float car_rotation = test_car_rotation;
            // Get car progress on path, knowing OLD car progress may speed up algorithm (a lot!)
            time_point t = Time::now();
            float car_progress = GetPathProgress(car_point, old_car_progress);
            LOG_WARNING(cString::Format("GetPathProgress takes %f seconds\n", fsec(Time::now() - t).count()));
            old_car_progress = car_progress;
            LOG_WARNING(cString::Format("ControlPointGenerator input: car_point <%f,%f>, car_progress <%f>, car_orientation <%f>\n", car_point[0], car_point[1], car_progress, car_rotation));
            // Generate control points
            t = Time::now();
            control_points = GetControlPoints(car_point, car_rotation, car_progress);
            LOG_WARNING(cString::Format("GetControlPoints takes %f seconds\n", fsec(Time::now() - t).count()));
//            // Control points are already ok, just feed them to controller if you don't want bugs
            LOG_WARNING("Generated control points:");
//            vector<std::initializer_list<float>> control;
            for (list_tuple_t::iterator it = control_points.begin(); it != control_points.end(); ++it)
            {
                tuple_t info = *it;
                LOG_WARNING(cString::Format("\tpoint: <%f,%f>, steering: <%f>, angle: <%f>", info[0], info[1], info[2], info[3]));
  //              control.push_back(std::initializer_list<float>{info[0],info[1],0.});
            }

//            LOG_WARNING("Obstacle circles:");
//            vector<std::initializer_list<float>> obstacle;
//            for (int i; i<5; ++i)
//            {
//                LOG_WARNING(cString::Format("\tobstacle: point <%f,%f>, radius <%f>", m_allObstaclesX[i],m_allObstaclesY[i],m_allObstaclesRadius[i]));
//                obstacle.push_back({m_allObstaclesX[i],m_allObstaclesY[i],m_allObstaclesRadius[i]});
//            }

//            tensorflow::Input::Initializer state({
//                                                     {
//                                                         {control[0], control[1], control[2], control[3], control[4]}, // must be 5
//                                                         {obstacle[0], obstacle[1], obstacle[2], obstacle[3], obstacle[4]} // must be 5
//                                                     }
//                                                 });

//            LOG_WARNING(cString::Format("Controller input: steering_angle <%f>, speed <%f>, speed limit <%f>", m_steeringAngle, m_currentSpeed, m_speedUpperLimit));
//            t = Time::now();
//            speed_vector = GetSpeedVector(state, m_steeringAngle, m_currentSpeed, m_speedUpperLimit);
//            LOG_WARNING(cString::Format("GetSpeedVector takes %f seconds\n", fsec(Time::now() - t).count()));
//            float new_angle = speed_vector[0]; // in degress
//            float new_speed = speed_vector[1]; // in m/s

//                //update this global variable with the controller's speed, otherwise you will keep feeding
//                //0 meters per second to car every time this Process loop executes!
//                m_currentSpeed = new_speed;
//                m_currentSpeed = m_currentSpeed < min_speed ? min_speed : m_currentSpeed;
//                m_currentSpeed = m_currentSpeed > max_speed ? max_speed : m_currentSpeed;
//                m_steeringAngle = new_angle;

////                if(m_currentSpeed <= 1.2)
////                {
////                    LOG_WARNING("CALCULATED SPEED BY FRANCESCO'S CONTROLLER IS TOO LOW, SO I MAKE IT 0.8");
////                    m_currentSpeed = 1.2;
////                }

//                LOG_WARNING("Test output");
//                LOG_WARNING(cString::Format("new steering angle: %f", new_angle));
//                LOG_WARNING(cString::Format("new speed: %f m/s", new_speed));
////                speedSignal.f32Value = new_speed < min_speed ? min_speed : new_speed; // actuator ignores speed < 0.5


//                // for debuging
//                /*float test_car_rotation = test_car_rotation + new_angle;
//            float space = new_speed*passed_seconds;
//            float test_x = test_car_point[0];
//            float test_y = test_car_point[1];
//            test_car_point = {test_x+space*cos(test_car_rotation), test_y+space*sin(test_car_rotation)};*/

//        }

        //ALWAYS FEED VALUES TO SPEED AND STEERING ACTUATOR OUTSIDE YOUR CONTROLLER FRANCESCO BECAUSE IN THE BEGINNING I INITIALIZE
        //THE BELOW TWO F32 VALUES TO 0

        //********************THIS IS WHERE FRANCESCO'S CONTROLLER ENDS********************//

        //********************THIS IS WHERE SARIM'S CONTROLLER STARTS**********************//
        //if (path_ready)
        //{
            //float car_progress = GetPathProgress(car_point, old_car_progress);
            //old_car_progress = car_progress;
            //float car_rotation = m_orientation;
            //LOG_WARNING(cString::Format("YOUR ANGLE IS %f", car_rotation));
            //LOG_WARNING(cString::Format("ControlPointGenerator input: car_point <%f,%f>, car_progress <%f>, car_orientation <%f>\n", car_point[0], car_point[1], car_progress, car_rotation));
            //list_tuple_t control_points = GetControlPoints(car_point, car_rotation, car_progress);
            LOG_WARNING("Generated control points:");
            //vector<std::initializer_list<float>> control;
            list_tuple_t::iterator it = control_points.begin();
            tuple_t info = *it;
            targetPosX = info[0]; targetPosY = info[1];
            //check your distance from the target position/ control point you are following
    //        tFloat32 sqrPosXControlPoint = pow((posX - targetPosX), 2);
    //        tFloat32 sqrPosYControlPoint = pow((posY - targetPosY), 2);
            tFloat32 angleToObstacle;
            if ((targetPosX > 0) && (targetPosY == 0)) { angleToObstacle = 0; }
            else if ((targetPosX == 0) && (targetPosY > 0)) { angleToObstacle = 90; }
            else if ((targetPosX < 0) && (targetPosY == 0)) { angleToObstacle = 180; }
            else if ((targetPosX == 0) && (targetPosY < 0)) { angleToObstacle = 270; }

            angleToObstacle = atan(targetPosY/targetPosX);
            if ((targetPosX > 0) && (targetPosY > 0)) { angleToObstacle = angleToObstacle; }
            else if ((targetPosX < 0) && (targetPosY > 0)) { angleToObstacle = 180 + angleToObstacle; }
            else if ((targetPosX < 0) && (targetPosY < 0)) { angleToObstacle = 180 + angleToObstacle; }
            else if ((targetPosX > 0) && (targetPosY < 0)) { angleToObstacle = 360 + angleToObstacle; }

            float degrees_orientation = m_orientation * (180/M_PI);
            if(degrees_orientation < 0)
            {
                degrees_orientation += 360;
            }

            LOG_WARNING(cString::Format("YOUR ANGLE TO OBSTACLE IS %f", angleToObstacle));

            if (angleToObstacle > degrees_orientation) { m_steeringAngle = -50; LOG_WARNING("TURNING LEFT"); }
            else if (angleToObstacle < degrees_orientation) { m_steeringAngle = 50; LOG_WARNING("TURNING RIGHT"); }
            else { steeringSignal.f32Value = 0; }
        }
//        for (list_tuple_t::iterator it = control_points.begin(); it != control_points.end(); ++it)
//        {
//            tuple_t info = *it;
//            LOG_WARNING(cString::Format("\tpoint: <%f,%f>", info[0], info[1]));
//            control.push_back(std::initializer_list<float>{info[0],info[1],0.});
//        }

        //********************THIS IS WHERE SARIM'S CONTROLLER ENDS**********************//

        if(m_Stop)
        {
            LOG_INFO("IT'S TIME TO STOP!!!");
            speedSignal.f32Value = 0;
            steeringSignal.f32Value = 0;
        }
        else
        {
            LOG_INFO("IT'S TIME TO RUN!!!!");
            steeringSignal.f32Value = m_steeringAngle;
            speedSignal.f32Value = m_currentSpeed;
        }

        if (emergencyReverse)
        {
            LOG_INFO("EMERGENCY BRAKE ENGAGED!");
            speedSignal.f32Value = -1.0;
            steeringSignal.f32Value = -steeringSignal.f32Value;
        }
        else
        {
            speedSignal.f32Value = 1.0;
        }

        object_ptr<const ISample> pSampleFromUS;
        // retrieve the values (using convenience methods that return a variant)
        //IMU
        tUltrasonicStruct US_data;

        if (IS_OK(m_oInputUltrasonicUnit.GetLastSample(pSampleFromUS)))
        {
            auto oDecoderUS = m_USDataSampleFactory.MakeDecoderFor(*pSampleFromUS);

            RETURN_IF_FAILED(oDecoderUS.IsValid());

            //we do not need the timestamps here
            RETURN_IF_FAILED(oDecoderUS.GetElementValue(m_ddlUltrasonicStructIndex.SideLeft.value, &US_data.tSideLeft.f32Value));
            RETURN_IF_FAILED(oDecoderUS.GetElementValue(m_ddlUltrasonicStructIndex.SideRight.value, &US_data.tSideRight.f32Value));
            RETURN_IF_FAILED(oDecoderUS.GetElementValue(m_ddlUltrasonicStructIndex.RearLeft.value, &US_data.tRearLeft.f32Value));
            RETURN_IF_FAILED(oDecoderUS.GetElementValue(m_ddlUltrasonicStructIndex.RearCenter.value, &US_data.tRearCenter.f32Value));
            RETURN_IF_FAILED(oDecoderUS.GetElementValue(m_ddlUltrasonicStructIndex.RearRight.value, &US_data.tRearRight.f32Value));
        }

        if (US_data.tSideLeft.f32Value < m_propUSDistanceObs)
        {
            LOG_INFO("ULTRA SONIC SIDE LEFT");
            steeringSignal.f32Value = 30;
        }
        else if (US_data.tSideRight.f32Value < m_propUSDistanceObs)
        {
            LOG_INFO("ULTRA SONIC SIDE RIGHT");
            steeringSignal.f32Value = -30;
        }
        else if (US_data.tRearLeft.f32Value < m_propUSDistanceObs)
        {
            LOG_INFO("ULTRA SONIC SIDE REAR LEFT");
            steeringSignal.f32Value = 30;
        }
        else if (US_data.tRearRight.f32Value < m_propUSDistanceObs)
        {
            LOG_INFO("ULTRA SONIC SIDE REAR LEFT");
            steeringSignal.f32Value = -30;
        }
        else if (US_data.tRearCenter.f32Value < m_propUSDistanceObs)
        {
            LOG_INFO("ULTRA SONIC SIDE REAR CENTER");
            speedSignal.f32Value = 0;
        }

        LOG_INFO(cString::Format("%f is the speed", speedSignal.f32Value));
        LOG_INFO(cString::Format("%f is the steering", steeringSignal.f32Value));

        if(m_Stop)
        {
            speedSignal.f32Value = 0; steeringSignal.f32Value = 0;
        }

        LOG_WARNING(cString::Format("new steering angle: %f", steeringSignal.f32Value));
        LOG_WARNING(cString::Format("new speed: %f m/s", speedSignal.f32Value));

//        speedSignal.f32Value = 0;

        //Send your state to jury at the end
        OnSendState(stateID, m_currentManeuver);

        RETURN_IF_FAILED(TransmitSpeed(speedSignal));
        RETURN_IF_FAILED(TransmitSteering(steeringSignal));
    }
    catch (...)
    {
        auto exc = std::current_exception();
        try
        {
            if (exc) std::rethrow_exception(exc);
        }
        catch(const std::exception& e)
        {
            LOG_ERROR(e.what());
        }
    }
    LOG_WARNING(cString::Format("SpeedController process takes %f seconds\n", fsec(Time::now() - speedcontroller_t).count()));
    RETURN_NOERROR;
}

tResult cSpeedController::OnSendState(stateCar stateID, tInt16 i16ManeuverEntry)
{

    tDriverStruct driverStruct;
    driverStruct.i16StateID = stateID;
    driverStruct.i16ManeuverEntry = i16ManeuverEntry;

    switch (stateID)
    {
    case statecar_ready:
        LOG_INFO(cString::Format("Driver Module: Send state: READY, Maneuver ID %d", i16ManeuverEntry));
        break;
    case statecar_running:
        LOG_INFO(cString::Format("Driver Module: Send state: RUNNING, Maneuver ID %d", i16ManeuverEntry));
        break;
    case statecar_complete:
        LOG_INFO(cString::Format("Driver Module: Send state: COMPLETE, Maneuver ID %d", i16ManeuverEntry));
        break;
    case statecar_error:
        LOG_INFO(cString::Format("Driver Module: Send state: ERROR, Maneuver ID %d", i16ManeuverEntry));
        break;
    case statecar_startup:
        LOG_INFO(cString::Format("Driver Module: Send state: STARTUP, Maneuver ID %d", i16ManeuverEntry));
        break;
    }
    RETURN_IF_FAILED(TransmitDriverStruct(driverStruct));

    RETURN_NOERROR;
}

tResult cSpeedController::TransmitDriverStruct(tDriverStruct& driverStruct)
{
    object_ptr<ISample> pWriteSample;

    RETURN_IF_FAILED(alloc_sample(pWriteSample));
    {
        auto oCodec = m_driverStructSampleFactory.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlDriverStructId.stateId, driverStruct.i16StateID));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlDriverStructId.maneuverEntry, driverStruct.i16ManeuverEntry));
    }

    m_oOutputDriverStruct << pWriteSample << flush << trigger;
    RETURN_NOERROR;
}

//I take the scan of the laser scanner as argument
void cSpeedController::CheckDistanceToObs(std::vector<tPolarCoordiante>& scan)
{
    tPolarCoordiante closestObstacle;
    closestObstacle.f32Angle = 0.0;
    closestObstacle.f32Radius = 99999.9;

    //check for obstacle by iterating through the vector I took in as argument in the beginning
    for (auto element : scan)
    {
        //as laser scanner values start from 270 ---> 0 (straight) ---> 90
        //Notice that the min and max FOV angle are made as property_variable. So, you need
        //convert them to tFloat32 for the comparison to be valid
        //Also, all angles are measured anti-clockwise from +ve x-axis
        //Why am I ORing? So, I check that my angle is from 0 to 60 degrees and then I check if it is
        //from 0 to 300 degrees. So, a truth table for both of these conditions is the correct logic
        if ((element.f32Angle >= tFloat32(40.0)) || (element.f32Angle <= tFloat32(90.0)))
        {
            //zero radius is actually invalid measurement. Zero means no object detected or the object
            //is too close (less than 15 to 20 cm)
            if (element.f32Radius != 0.0 && closestObstacle.f32Radius > element.f32Radius)
            {
                closestObstacle = element;
            }
        }
    }

    //check min distance
    if (closestObstacle.f32Radius <= tFloat32(650.0))
    {
        tFloat32 temp_radius = closestObstacle.f32Radius;
        tFloat32 temp_angle = closestObstacle.f32Angle;
        m_allObstaclesX[m_obstacleIterator] = temp_radius * cos(temp_angle);
        m_allObstaclesY[m_obstacleIterator] = temp_radius * sin(temp_angle);
        m_allObstaclesRadius[m_obstacleIterator] = 0.5f;
        m_obstacleIterator++;
        if (m_obstacleIterator >= 5)
        {
            m_obstacleIterator = 0;
        }
    }

    /*LOG_INFO(cString::Format("%s obstacle found, closest is at %f deg with %f mm dist,"
                                     "FOV (max/min) (%f/%f)", (m_doEmergencyBrake?"  ":"no"),
                                     closestObstacle.f32Angle, closestObstacle.f32Radius,
                                     tFloat32(m_propMaxFovAngle), tFloat32(m_propMinFovAngle)));*/
    //Since this function is invoked periodically, it is advisable to either comment the above LOG_INFO
    //or to disable it in ADTF
}

tResult cSpeedController::LoadManeuverList()
{
    m_sectorList.clear();
    // create dom from string received from pin
    cDOM oDOM;
    oDOM.FromString(m_strManeuverFileString);
    cDOMElementRefList oSectorElems;
    cDOMElementRefList oManeuverElems;

    //read first Sector Elem
    if (IS_OK(oDOM.FindNodes("AADC-Maneuver-List/AADC-Sector", oSectorElems)))
    {
        //iterate through sectors
        for (cDOMElementRefList::iterator itSectorElem = oSectorElems.begin(); itSectorElem != oSectorElems.end(); ++itSectorElem)
        {
            //if sector found
            tSector sector;
            sector.id = (*itSectorElem)->GetAttributeUInt32("id");

            if (IS_OK((*itSectorElem)->FindNodes("AADC-Maneuver", oManeuverElems)))
            {
                //iterate through maneuvers
                for (cDOMElementRefList::iterator itManeuverElem = oManeuverElems.begin(); itManeuverElem != oManeuverElems.end(); ++itManeuverElem)
                {
                    tManeuver man;
                    man.id = (*itManeuverElem)->GetAttributeUInt32("id");
                    man.action = maneuverFromString((*itManeuverElem)->GetAttribute("action").GetPtr());
                    if (man.action == maneuver_cross_parking)
                    {
                        man.extra = (*itManeuverElem)->GetAttributeUInt32("extra");
                    }
                    sector.sector.push_back(man);
                    listOfManeuvers.push_back(man);
                    m_NumberOfManeuvers++;
                }
            }

            m_sectorList.push_back(sector);
        }
    }
    if (oSectorElems.size() > 0)
    {
        loadedManeuverList = true;
        LOG_INFO("DriverFilter: Loaded Maneuver file successfully.");
    }
    else
    {
        LOG_ERROR("DriverFilter: no valid Maneuver Data found!");
        RETURN_ERROR(ERR_INVALID_FILE);
    }


    RETURN_NOERROR;
}

tResult cSpeedController::ProcessRoadSignFile(tInt64 tmTimeOfTrigger, const ISample& sample)
{
    adtf::ucom::ant::object_ptr_shared_locked<const adtf::streaming::ant::ISampleBuffer> pSampleBuffer;
    RETURN_IF_FAILED(sample.Lock(pSampleBuffer));

    adtf_util::cString roadSignFileString;
    roadSignFileString.SetBuffer(pSampleBuffer->GetSize());

    memcpy(roadSignFileString.GetBuffer(), pSampleBuffer->GetPtr(), pSampleBuffer->GetSize());


    cDOM oDOM;
    RETURN_IF_FAILED(oDOM.FromString(roadSignFileString));
    RETURN_IF_FAILED(ParseRoadSignFile(oDOM));

    LOG_INFO(cString::Format("Recieved road sign file from pin with %d signs", m_roadSigns.size()));
    RETURN_NOERROR;
}

tResult cSpeedController::ParseRoadSignFile(cDOM& oDOM)
{
    m_roadSigns.clear();

    cDOMElementRefList oElems;

    if (IS_OK(oDOM.FindNodes("configuration/roadSign", oElems)))
    {
        for (cDOMElementRefList::iterator itElem = oElems.begin(); itElem != oElems.end(); ++itElem)
        {
            roadSign item;
            item.u16Id = tUInt16((*itElem)->GetAttribute("id", "0").AsInt32());
            item.f32X = tFloat32((*itElem)->GetAttribute("x", "0").AsFloat64());
            item.f32Y = tFloat32((*itElem)->GetAttribute("y", "0").AsFloat64());
            item.f32Direction = tFloat32((*itElem)->GetAttribute("direction", "0").AsFloat64());

            switch(item.u16Id)
            {
            case 0:
                ListOfUnmarkedSigns.push_back(item);
                break;
            case 1:
                ListOfStopSigns.push_back(item);
                break;
            case 2:
                ListOfParkingSigns.push_back(item);
                break;
            case 3:
                ListOfHaveWaySigns.push_back(item);
                break;
            case 4:
                ListOfStraightSigns.push_back(item);
                break;
            case 5:
                ListOfGiveWaySigns.push_back(item);
                break;
            case 6:
                ListOfPedestrianSigns.push_back(item);
                break;
            case 7:
                ListOfRoundaboutSigns.push_back(item);
                break;
            case 8:
                ListOfOvertakingSigns.push_back(item);
                break;
            case 9:
                ListOfNoEntrySigns.push_back(item);
                break;
            case 10:
                ListOfStartSigns.push_back(item);
                break;
            case 11:
                ListOfOneWayStreetSigns.push_back(item);
                break;
            case 12:
                ListOfRoadworksSigns.push_back(item);
                break;
            case 13:
                ListOf50Signs.push_back(item);
                break;
            case 14:
                ListOf100Signs.push_back(item);
                break;
            }

            m_roadSigns.push_back(item);
        }
        loadedRoadSignsList = true;
        RETURN_NOERROR;
    }
    else
    {
        RETURN_ERROR(ERR_INVALID_FILE);
    }

    RETURN_NOERROR;
}

//Here you write the outputs to the output pins
tResult cSpeedController::TransmitSpeed(tSignalValue speedSignal)
{
    object_ptr<ISample> pWriteSample;

    //Here you allocate a media sample which has a return type of tResult
    //Just check if tResult is valid or not
    RETURN_IF_FAILED(alloc_sample(pWriteSample))
    {
        auto oCodec = m_SpeedFactory.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSpeed.timeStamp, speedSignal.ui32ArduinoTimestamp));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSpeed.value, speedSignal.f32Value));
    }

    m_oSpeed << pWriteSample << flush << trigger;
    RETURN_NOERROR;
}

//Here you write the outputs to the output pins
tResult cSpeedController::TransmitSteering(tSignalValue steeringSignal)
{
    object_ptr<ISample> pWriteSample;

    //Here you allocate a media sample which has a return type of tResult
    //Just check if tResult is valid or not
    RETURN_IF_FAILED(alloc_sample(pWriteSample))
    {
        auto oCodec = m_SteeringFactory.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSteering.timeStamp, steeringSignal.ui32ArduinoTimestamp));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSteering.value, steeringSignal.f32Value));
    }

    m_oSteering << pWriteSample << flush << trigger;
    RETURN_NOERROR;
}

tResult cSpeedController::ToggleHeadLights(tBoolSignalValue lightsSignal)
{
    object_ptr<ISample> pWriteSample;

    //Here you allocate a media sample which has a return type of tResult
    //Just check if tResult is valid or not
    RETURN_IF_FAILED(alloc_sample(pWriteSample))
    {
        auto oCodec = m_SteeringFactory.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlBoolSignalValueId.ui32ArduinoTimestamp, lightsSignal.ui32ArduinoTimestamp));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlBoolSignalValueId.bValue, lightsSignal.bValue));
    }

    m_oOutputHeadLight << pWriteSample << flush << trigger;
    RETURN_NOERROR;
}

tResult cSpeedController::ToggleBrakeLights(tBoolSignalValue lightsSignal)
{
    object_ptr<ISample> pWriteSample;

    //Here you allocate a media sample which has a return type of tResult
    //Just check if tResult is valid or not
    RETURN_IF_FAILED(alloc_sample(pWriteSample))
    {
        auto oCodec = m_SteeringFactory.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlBoolSignalValueId.ui32ArduinoTimestamp, lightsSignal.ui32ArduinoTimestamp));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlBoolSignalValueId.bValue, lightsSignal.bValue));
    }

    m_oOutputBrakeLight<< pWriteSample << flush << trigger;
    RETURN_NOERROR;
}

tResult cSpeedController::ToggleReverseLights(tBoolSignalValue lightsSignal)
{
    object_ptr<ISample> pWriteSample;

    //Here you allocate a media sample which has a return type of tResult
    //Just check if tResult is valid or not
    RETURN_IF_FAILED(alloc_sample(pWriteSample))
    {
        auto oCodec = m_SteeringFactory.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlBoolSignalValueId.ui32ArduinoTimestamp, lightsSignal.ui32ArduinoTimestamp));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlBoolSignalValueId.bValue, lightsSignal.bValue));
    }

    m_oOutputReverseLight << pWriteSample << flush << trigger;
    RETURN_NOERROR;
}

tResult cSpeedController::ToggleHazardLights(tBoolSignalValue lightsSignal)
{
    object_ptr<ISample> pWriteSample;

    //Here you allocate a media sample which has a return type of tResult
    //Just check if tResult is valid or not
    RETURN_IF_FAILED(alloc_sample(pWriteSample))
    {
        auto oCodec = m_SteeringFactory.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlBoolSignalValueId.ui32ArduinoTimestamp, lightsSignal.ui32ArduinoTimestamp));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlBoolSignalValueId.bValue, lightsSignal.bValue));
    }

    m_oOutputHazard << pWriteSample << flush << trigger;
    RETURN_NOERROR;
}

tResult cSpeedController::ToggleLeftLights(tBoolSignalValue lightsSignal)
{
    object_ptr<ISample> pWriteSample;

    //Here you allocate a media sample which has a return type of tResult
    //Just check if tResult is valid or not
    RETURN_IF_FAILED(alloc_sample(pWriteSample))
    {
        auto oCodec = m_SteeringFactory.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlBoolSignalValueId.ui32ArduinoTimestamp, lightsSignal.ui32ArduinoTimestamp));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlBoolSignalValueId.bValue, lightsSignal.bValue));
    }

    m_oOutputTurnLeft << pWriteSample << flush << trigger;
    RETURN_NOERROR;
}

tResult cSpeedController::ToggleRightLights(tBoolSignalValue lightsSignal)
{
    object_ptr<ISample> pWriteSample;

    //Here you allocate a media sample which has a return type of tResult
    //Just check if tResult is valid or not
    RETURN_IF_FAILED(alloc_sample(pWriteSample))
    {
        auto oCodec = m_SteeringFactory.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlBoolSignalValueId.ui32ArduinoTimestamp, lightsSignal.ui32ArduinoTimestamp));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlBoolSignalValueId.bValue, lightsSignal.bValue));
    }

    m_oOutputTurnRight << pWriteSample << flush << trigger;
    RETURN_NOERROR;
}
