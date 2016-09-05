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

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import hallib.HalUtil;
import trclib.TrcAnalogInput;
import trclib.TrcDbgTrace;
import trclib.TrcFilter;

/**
 * This class implements the Modern Robotics Optical Distance sensor
 * extending TrcAnalogInput. It provides implementation of the abstract
 * methods in TrcAnalogInput.
 */
public class FtcOpticalDistanceSensor extends TrcAnalogInput
{
    private static final String moduleName = "FtcOpticalDistanceSensor";
    private static final boolean debugEnabled = false;
    private TrcDbgTrace dbgTrace = null;

    private OpticalDistanceSensor sensor;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     * @param filters specifies an array of filter objects, one for each axis, to filter
     *                sensor data. If no filter is used, this can be set to null.
     */
    public FtcOpticalDistanceSensor(
            HardwareMap hardwareMap, String instanceName, TrcFilter[] filters)
    {
        super(instanceName, 1, 0, filters);

        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName + "." + instanceName,
                                       false,
                                       TrcDbgTrace.TraceLevel.API,
                                       TrcDbgTrace.MsgLevel.INFO);
        }

        sensor = hardwareMap.opticalDistanceSensor.get(instanceName);
    }   //FtcOpticalDistanceSensor

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param filters specifies an array of filter objects, one for each axis, to filter
     *                sensor data. If no filter is used, this can be set to null.
     */
    public FtcOpticalDistanceSensor(String instanceName, TrcFilter[] filters)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, filters);
    }   //FtcOpticalDistanceSensor

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FtcOpticalDistanceSensor(String instanceName)
    {
        this(instanceName, null);
    }   //FtcOpticalDistanceSensor

    /**
     * This method calibrates the sensor.
     */
    public void calibrate()
    {
        calibrate(DataType.INPUT_DATA);
    }   //calibrate

    //
    // Implements TrcAnalogInput abstract methods.
    //

    /**
     * This method returns the raw sensor data of the specified type.
     *
     * @param index specifies the data index.
     * @return raw sensor data of the specified type.
     */
    @Override
    public SensorData getRawData(int index, DataType dataType)
    {
        final String funcName = "getRawData";
        SensorData data = null;

        //
        // Ultrasonic sensor supports only INPUT_DATA type.
        //
        if (dataType == DataType.INPUT_DATA)
        {
            if (index == 0)
            {
                data = new SensorData(
                        HalUtil.getCurrentTime(), (double)sensor.getRawLightDetected());
            }
            else if (index == 1)
            {
                data = new SensorData(
                        HalUtil.getCurrentTime(), (double)sensor.getLightDetected());
            }
            else
            {
                throw new IllegalArgumentException("Invalid data index.");
            }
        }
        else
        {
            throw new UnsupportedOperationException(
                    "Optical Distance sensor only support INPUT_DATA type.");
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp:%.3f,value=%f)", data.timestamp, data.value);
        }

        return data;
    }   //getRawData

}   //class FtcOpticalDistanceSensor
