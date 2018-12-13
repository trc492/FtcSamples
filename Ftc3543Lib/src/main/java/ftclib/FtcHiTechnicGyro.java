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

import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import trclib.TrcDbgTrace;
import trclib.TrcFilter;
import trclib.TrcGyro;
import trclib.TrcUtil;

/**
 * This class implements the HiTechnic gyro extending TrcGyro. It provides implementation of the abstract methods
 * in TrcGyro. It supports only the z axis. It provides rotation rate data but not heading and it does not support
 * built-in calibration.
 */
public class FtcHiTechnicGyro extends TrcGyro
{
    private static final String moduleName = "FtcHiTechnicGyro";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private GyroSensor gyro;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     * @param filters specifies an array of filters to use for filtering sensor noise, one for each axis. Since we
     *                only have 1 axis, the array should have 1 element. If no filters are used, it can be set to
     *                null.
     */
    public FtcHiTechnicGyro(HardwareMap hardwareMap, String instanceName, TrcFilter[] filters)
    {
        super(instanceName, 1, GYRO_HAS_Z_AXIS | GYRO_INTEGRATE, filters);

        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        gyro = hardwareMap.gyroSensor.get(instanceName);
    }   //FtcHiTechnicGyro

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param filters specifies an array of filters to use for filtering sensor noise, one for each axis. Since we
     *                only have 1 axis, the array should have 1 element. If no filters are used, it can be set to
     *                null.
     */
    public FtcHiTechnicGyro(String instanceName, TrcFilter[] filters)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, filters);
    }   //FtcHiTechnicGyro

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FtcHiTechnicGyro(String instanceName)
    {
        this(instanceName, null);
    }   //FtcHiTechnicGyro

    /**
     * This method calibrates the sensor.
     */
    public synchronized void calibrate()
    {
        calibrate(DataType.ROTATION_RATE);
    }   //calibrate

    //
    // Implements TrcGyro abstract methods.
    //

    /**
     * This method returns the raw data of the specified type for the x-axis which is not supported.
     *
     * @param dataType specifies the data type.
     * @return throws UnsupportedOperation exception.
     */
    @Override
    public SensorData<Double> getRawXData(DataType dataType)
    {
        final String funcName = "getRawXData";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=null");
        }

        throw new UnsupportedOperationException("HiTechnic gyro does not support x-axis.");
    }   //getRawXData

    /**
     * This method returns the raw data of the specified type for the y-axis which is not supported.
     *
     * @param dataType specifies the data type.
     * @return throws UnsupportedOperation exception.
     */
    @Override
    public SensorData<Double> getRawYData(DataType dataType)
    {
        final String funcName = "getRawYData";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=null");
        }

        throw new UnsupportedOperationException("HiTechnic gyro does not support y-axis.");
    }   //getRawYData

    /**
     * This method returns the raw data of the specified type for the z-axis.
     *
     * @param dataType specifies the data type.
     * @return raw data of the specified type for the z-axis.
     */
    @Override
    public synchronized SensorData<Double> getRawZData(DataType dataType)
    {
        final String funcName = "getRawZData";
        SensorData<Double> data;
        //
        // HiTechnic gyro supports only rotation rate.
        //
        if (dataType == DataType.ROTATION_RATE)
        {
            data = new SensorData<>(TrcUtil.getCurrentTime(), (double)gyro.rawZ());
        }
        else
        {
            throw new UnsupportedOperationException("HiTechnic gyro only supports rotation rate.");
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp:%.3f,value:%f", data.timestamp, data.value);
        }

        return data;
    }   //getRawZData

}   //class FtcHiTechnicGyro
