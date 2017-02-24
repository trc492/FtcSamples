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
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import trclib.TrcDbgTrace;
import trclib.TrcFilter;
import trclib.TrcSensor;
import trclib.TrcUtil;

/**
 * This class implements a platform dependent ultrasonic sensor extending TrcAnalogInput. It provides implementation
 * of the abstract methods in TrcAnalogInput.
 */
public class FtcUltrasonicSensor extends TrcSensor<FtcUltrasonicSensor.DataType>
{
    private static final String moduleName = "FtcUltrasonicSensor";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    public enum DataType
    {
        ULTRASONIC
    }   //enum DataType

    private UltrasonicSensor sensor;
    private double sensorData = 0.0;
    private long dataTagId = -1;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     * @param filters specifies an array of filter objects, one for each axis, to filter sensor data. If no filter
     *                is used, this can be set to null.
     */
    public FtcUltrasonicSensor(HardwareMap hardwareMap, String instanceName, TrcFilter[] filters)
    {
        super(instanceName, 1, filters);

        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        sensor = hardwareMap.ultrasonicSensor.get(instanceName);
    }   //FtcUltrasonicSensor

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param filters specifies an array of filter objects, one for each axis, to filter sensor data. If no filter
     *                is used, this can be set to null.
     */
    public FtcUltrasonicSensor(String instanceName, TrcFilter[] filters)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, filters);
    }   //FtcUltrasonicSensor

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FtcUltrasonicSensor(String instanceName)
    {
        this(instanceName, null);
    }   //FtcUltrasonicSensor

    //
    // Implements TrcAnalogInput abstract methods.
    //

    /**
     * This method returns the raw sensor data of the specified type.
     *
     * @param index specifies the data index (not used).
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
            case ULTRASONIC:
                if (currTagId != dataTagId)
                {
                    sensorData = sensor.getUltrasonicLevel();
                    dataTagId = currTagId;
                }
                data = new SensorData<>(TrcUtil.getCurrentTime(), sensorData);
                break;
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp:%.3f,value=%f)", data.timestamp, data.value);
        }

        return data;
    }   //getRawData

}   //class FtcUltrasonicSensor
