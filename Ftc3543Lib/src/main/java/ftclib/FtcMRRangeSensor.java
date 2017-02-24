/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package ftclib;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import trclib.TrcDbgTrace;
import trclib.TrcFilter;
import trclib.TrcSensor;
import trclib.TrcUtil;

/**
 * This class implements the Modern Range sensor extending TrcAnalogInput. It provides implementation of the abstract
 * methods in TrcAnalogInput.
 */
public class FtcMRRangeSensor extends TrcSensor<FtcMRRangeSensor.DataType>
{
    private static final String moduleName = "FtcMRRangeSensor";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    public enum DataType
    {
        DISTANCE_INCH,
        ULTRASONIC_CM,
        OPTICAL_CM,
        ULTRASONIC_RAW,
        OPTICAL_RAW,
        RAW_LIGHT_DETECTED,
        LIGHT_DETECTED
    }   //enum DataType

    public ModernRoboticsI2cRangeSensor sensor;
    private double distanceInchData = 0.0;
    private long distanceInchTagId = -1;
    private double ultrasonicCmData = 0.0;
    private long ultrasonicCmTagId = -1;
    private double opticalCmData = 0.0;
    private long opticalCmTagId = -1;
    private double ultrasonicRawData = 0.0;
    private long ultrasonicRawTagId = -1;
    private double opticalRawData = 0.0;
    private long opticalRawTagId = -1;
    private double rawLightData = 0.0;
    private double rawLightTagId = -1;
    private double lightData = 0.0;
    private long lightTagId = -1;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     * @param filters specifies an array of filter objects, one for each axis, to filter sensor data. If no filter
     *                is used, this can be set to null.
     */
    public FtcMRRangeSensor(HardwareMap hardwareMap, String instanceName, TrcFilter[] filters)
    {
        super(instanceName, 1, filters);

        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        sensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, instanceName);
    }   //FtcMRRangeSensor

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param filters specifies an array of filter objects, one for each axis, to filter sensor data. If no filter
     *                is used, this can be set to null.
     */
    public FtcMRRangeSensor(String instanceName, TrcFilter[] filters)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, filters);
    }   //FtcMRRangeSensor

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FtcMRRangeSensor(String instanceName)
    {
        this(instanceName, null);
    }   //FtcMRRangeSensor

    /**
     * This method calibrates the sensor.
     */
    public void calibrate()
    {
        calibrate(DataType.DISTANCE_INCH);
    }   //calibrate

    //
    // Implements TrcAnalogInput abstract methods.
    //

    /**
     * This method returns the raw sensor data of the specified type.
     *
     * @param index specifies the data index.
     * @param dataType specifies the data type.
     * @return raw sensor data of the specified index and type.
     */
    @Override
    public SensorData<Double> getRawData(int index, DataType dataType)
    {
        final String funcName = "getRawData";
        SensorData<Double> data = null;
        long currTagId = FtcOpMode.getLoopCounter();

        switch (dataType)
        {
            case DISTANCE_INCH:
                if (currTagId != distanceInchTagId)
                {
                    distanceInchData = sensor.getDistance(DistanceUnit.INCH);
                    distanceInchTagId = currTagId;
                }
                data = new SensorData<>(TrcUtil.getCurrentTime(), distanceInchData);
                break;

            case ULTRASONIC_CM:
                if (currTagId != ultrasonicCmTagId)
                {
                    ultrasonicCmData = sensor.cmUltrasonic();
                    ultrasonicCmTagId = currTagId;
                }
                data = new SensorData<>(TrcUtil.getCurrentTime(), ultrasonicCmData);
                break;

            case OPTICAL_CM:
                if (currTagId != opticalCmTagId)
                {
                    opticalCmData = sensor.cmOptical();
                    opticalCmTagId= currTagId;
                }
                data = new SensorData<>(TrcUtil.getCurrentTime(), opticalCmData);
                break;

            case ULTRASONIC_RAW:
                if (currTagId != ultrasonicRawTagId)
                {
                    ultrasonicRawData = sensor.rawUltrasonic();
                    ultrasonicCmTagId = currTagId;
                }
                data = new SensorData<>(TrcUtil.getCurrentTime(), ultrasonicRawData);
                break;

            case OPTICAL_RAW:
                if (currTagId != opticalRawTagId)
                {
                    opticalRawData = sensor.rawOptical();
                    opticalRawTagId = currTagId;
                }
                data = new SensorData<>(TrcUtil.getCurrentTime(), opticalRawData);
                break;

            case RAW_LIGHT_DETECTED:
                if (currTagId != rawLightTagId)
                {
                    rawLightData = sensor.getRawLightDetected();
                    rawLightTagId = currTagId;
                }
                data = new SensorData<>(TrcUtil.getCurrentTime(), rawLightData);
                break;

            case LIGHT_DETECTED:
                if (currTagId != lightTagId)
                {
                    lightData = sensor.getLightDetected();
                    lightTagId = currTagId;
                }
                data = new SensorData<>(TrcUtil.getCurrentTime(), lightData);
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp:%.3f,value=%f)", data.timestamp, data.value);
        }

        return data;
    }   //getRawData

}   //class FtcMRRangeSensor
