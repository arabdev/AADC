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


/*What will my Emergency Brake filter do? It will take the speed output from the car controller filter,
 * Then, it will let this speed go to the actuator as long as there is no obstacle. But, if there is, then
 * it will simply pass the speed value of 0 to the actuators*/

#include "stdafx.h"
#include "EmergencyBrake.h"


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_TEMPLATEFILTER_DATA_TRIGGERED_FILTER,
                                    "EmergencyBrake",   //label
                                    cEmergencyBrake,    //class
                                    adtf::filter::pin_trigger({"input_speed"}));
/*In the last line you register a trigger to a pin. In our case, it is the input pin, as
 * obvous from the word 'input' in description. Look at this line:
 * Register(m_oReaderSpeed, "input" , pTypeTemplateData);*/


//This is the constructor where you get the media description
cEmergencyBrake::cEmergencyBrake()
{
    //DO NOT FORGET TO LOAD MEDIA DESCRIPTION SERVICE IN ADTF3 AND CHOOSE aadc.description
    //The if and else statements at the bottom simply show my Debug messages
    //IS_OK will check whether tResult is true or not. tResult is what I get from the expression in the
    //brackets and it is the most common return type in ADTF
    //DO NOT FORGET TO LOAD MEDIA DESCRIPTION SERVICE IN ADTF3 AND CHOOSE aadc.description

    object_ptr<IStreamType> pTypeSpeedvalue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSpeedvalue, m_SpeedValueSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_SpeedValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlSpeedValueId.timeStamp);
        adtf_ddl::access_element::find_index(m_SpeedValueSampleFactory, cString("f32Value"), m_ddlSpeedValueId.value);
    }
    else
    {
        LOG_WARNING("No media description for tSignalValue found!");
    }

    //After finding media description for tSignalvalue, you register your pins
    Register(m_oReaderSpeed, "input_speed" , pTypeSpeedvalue);
    Register(m_oWriterSpeed, "output_speed", pTypeSpeedvalue);

    object_ptr<IStreamType> pTypeSteer;
    if (ERR_NOERROR == adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSteer, m_SteerFactory))
    {
        adtf_ddl::access_element::find_index(m_SteerFactory, cString("f32Value"), m_ddlSteer.value);
        adtf_ddl::access_element::find_index(m_SteerFactory, cString("ui32ArduinoTimestamp"), m_ddlSteer.timeStamp);
    }
    else
    {
        LOG_WARNING("No mediadescription for tSignalvalue found!");
    }

    Register(m_oSteer, "steer", pTypeSteer);
    Register(m_oSteerIn, "steer_input", pTypeSteer);

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

    //register properties
    RegisterPropertyVariable("Field of view min angle [deg]", m_propMinFovAngle);
    RegisterPropertyVariable("Field of view max angle [deg]", m_propMaxFovAngle);
    RegisterPropertyVariable("Min distance to obstacle [mm]", m_propMinObstacleDistance);
    //    RegisterPropertyVariable("Min angle of obstacle for which your car steers right [deg]", m_propMinAngleForObs);

    m_doEmergencyBrake = false;
    //    m_steerLeft = false;
    //    m_steerRight = false;
}


//implement the Configure function to read ALL Properties
tResult cEmergencyBrake::Configure()
{
    RETURN_NOERROR;
}

/*In the process function, you take the input pin and read data and decode it
 * and you transmit the decoded data using the output pin. Look at this line:
 * m_oWriter << pWriteSample << flush << trigger;*/
tResult cEmergencyBrake::Process(tTimeStamp tmTimeOfTrigger)
{
    try
    {
        object_ptr<const ISample> pReadSampleSpeed;   //pReadSampleSpeed is the sample you want to read

        tSignalValue speedSignal;

        //check if the input pin receives a sample and whether you can do something with it
        if (IS_OK(m_oReaderSpeed.GetLastSample(pReadSampleSpeed)))
        {
            auto oDecoder = m_SpeedValueSampleFactory.MakeDecoderFor(*pReadSampleSpeed);

            RETURN_IF_FAILED(oDecoder.IsValid());
            //In above two lines, you simply make a decoder for the input data
            //and then check if the decoder is valid

            // retrieve the values (using convenience methods that return a variant)
            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSpeedValueId.value, &speedSignal.f32Value));
            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSpeedValueId.timeStamp, &speedSignal.ui32ArduinoTimestamp));
        }

        object_ptr<const ISample> pReadSampleSteer;

        tSignalValue steerSignal;

        if (IS_OK(m_oSteerIn.GetLastSample(pReadSampleSteer)))
        {
            auto oDecoder = m_SteerFactory.MakeDecoderFor(*pReadSampleSteer);

            RETURN_IF_FAILED(oDecoder.IsValid());
            //In above two lines, you simply make a decoder for the input data
            //and then check if the decoder is valid

            // retrieve the values (using convenience methods that return a variant)
            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSteer.value, &steerSignal.f32Value));
            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSteer.timeStamp, &steerSignal.ui32ArduinoTimestamp));
        }

        object_ptr<const ISample> pReadSampleLS;

        if(IS_OK(m_oInputLaserScanner.GetLastSample(pReadSampleLS)))
        {
            auto oDecoder = m_LSStructSampleFactory.MakeDecoderFor(*pReadSampleLS);

            RETURN_IF_FAILED(oDecoder.IsValid());
            tSize numOfScanPoints = 0;
            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlLSDataId.size, &numOfScanPoints));

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



//        if (m_doEmergencyBrake)
//        {
//            LOG_INFO("There's an obstacle, BREAK!");
//            speedSignal.f32Value = 0;
//        }


        RETURN_IF_FAILED(TransmitSpeed(speedSignal));
        RETURN_IF_FAILED(TransmitSteer(steerSignal));
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
    RETURN_NOERROR;
}

//I take the scan of the laser scanner as argument
void cEmergencyBrake::CheckDistanceToObs(std::vector<tPolarCoordiante>& scan)
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
        if (element.f32Angle >= tFloat32(m_propMinFovAngle) || element.f32Angle <= tFloat32(m_propMaxFovAngle))
        {
            //zero radius is actually invalid measurement. Zero means no object detected or the object
            //is too close (less than 15 to 20 cm)
            if (element.f32Radius != 0.0 && closestObstacle.f32Radius > element.f32Radius)
            {
                closestObstacle = element;
            }
        }

        //        if (element.f32Angle >= 90)
        //        {
        //            m_steerRight = true;
        //        }
        //        else
        //        {
        //            m_steerLeft = true;
        //        }
    }

    //check min distance
    m_doEmergencyBrake = closestObstacle.f32Radius < tFloat32(m_propMinObstacleDistance);

    /*LOG_INFO(cString::Format("%s obstacle found, closest is at %f deg with %f mm dist,"
                                     "FOV (max/min) (%f/%f)", (m_doEmergencyBrake?"  ":"no"),
                                     closestObstacle.f32Angle, closestObstacle.f32Radius,
                                     tFloat32(m_propMaxFovAngle), tFloat32(m_propMinFovAngle)));*/
    //Since this function is invoked periodically, it is advisable to either comment the above LOG_INFO
    //or to disable it in ADTF
}

//Here you write the outputs to the output pins
tResult cEmergencyBrake::TransmitSpeed(tSignalValue speedSignal)
{
    object_ptr<ISample> pWriteSample;

    //Here you allocate a media sample which has a return type of tResult
    //Just check if tResult is valid or not
    RETURN_IF_FAILED(alloc_sample(pWriteSample))
    {
        auto oCodec = m_SpeedValueSampleFactory.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSpeedValueId.timeStamp, speedSignal.ui32ArduinoTimestamp));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSpeedValueId.value, speedSignal.f32Value));
    }

    m_oWriterSpeed <<pWriteSample << flush << trigger;
    RETURN_NOERROR;
}

//Here you write the outputs to the output pins
tResult cEmergencyBrake::TransmitSteer(tSignalValue steerSignal)
{
    object_ptr<ISample> pWriteSample;

    //Here you allocate a media sample which has a return type of tResult
    //Just check if tResult is valid or not
    RETURN_IF_FAILED(alloc_sample(pWriteSample))
    {
        auto oCodec = m_SteerFactory.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSteer.timeStamp, steerSignal.ui32ArduinoTimestamp));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSteer.value, steerSignal.f32Value));
    }

    m_oSteer << pWriteSample << flush << trigger;
    RETURN_NOERROR;
}
