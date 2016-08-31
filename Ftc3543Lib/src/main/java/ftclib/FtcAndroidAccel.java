/*
 * Titan Robotics Framework Library
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.net)
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

import android.hardware.Sensor;
import android.hardware.SensorManager;

import hallib.HalUtil;
import trclib.TrcAccelerometer;
import trclib.TrcDbgTrace;
import trclib.TrcFilter;

/**
 * This class implements the Android accelerometer extending
 * TrcAccelerometer. It provides implementation of the abstract methods
 * in TrcAccelerometer. It supports 3 axes: x, y and z. It provides
 * acceleration data for all 3 axes. However, it doesn't provide any
 * velocity or distance data.
 */
public class FtcAndroidAccel extends TrcAccelerometer
{
    private static final String moduleName = "FtcAndroidAccel";
    private static final boolean debugEnabled = false;
    private TrcDbgTrace dbgTrace = null;

    private FtcAndroidSensor sensor = null;
    private int samplingPeriod = SensorManager.SENSOR_DELAY_GAME;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param filters specifies an array of filters to use for filtering
     *                sensor noise, one for each axis. Since we have 3 axes,
     *                the array should have 3 elements. If no filters are
     *                used, it can be set to null.
     */
    public FtcAndroidAccel(String instanceName, TrcFilter[] filters)
    {
        super(instanceName,
              3,
              ACCEL_HAS_X_AXIS | ACCEL_HAS_Y_AXIS | ACCEL_HAS_Z_AXIS |
              ACCEL_INTEGRATE | ACCEL_DOUBLE_INTEGRATE,
              filters);

        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName + "." + instanceName,
                                       false,
                                       TrcDbgTrace.TraceLevel.API,
                                       TrcDbgTrace.MsgLevel.INFO);
        }

        sensor = new FtcAndroidSensor(instanceName, Sensor.TYPE_LINEAR_ACCELERATION, 3);
    }   //FtcAndroidAccel

    /**
     * This method sets the sampling period of the Android accelerometer sensor.
     *
     * @param period specifies the period with SensorManager.SENSOR_DELAY_* constants,
     *               or the number of microseconds.
     */
    public void setSamplingPeriod(int period)
    {
        final String funcName = "setSamplingPeriod";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "period=%d", period);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        samplingPeriod = period;
    }   //setSamplingPeriod

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FtcAndroidAccel(String instanceName)
    {
        this(instanceName, null);
    }   //FtcAndroidAccel

    /**
     * This method enables/disables the sensor.
     *
     * @param enabled specifies true if enabling, false otherwise.
     */
    @Override
    public void setEnabled(boolean enabled)
    {
        sensor.setEnabled(enabled, samplingPeriod);
        super.setEnabled(enabled);
    }   //setEnabled

    /**
     * This method calibrates the sensor. If the sensor is not enabled,
     * it must enable it first before starting calibration. It will disable
     * the sensor if it was disabled before calibration.
     */
    public void calibrate()
    {
        boolean sensorEnabled = sensor.isEnabled();

        if (!sensorEnabled)
        {
            sensor.setEnabled(true);
        }

        calibrate(DataType.ACCELERATION);

        if (!sensorEnabled)
        {
            sensor.setEnabled(false);
        }
    }   //calibrate

    //
    // Implements TrcAccelerometer abstract methods.
    //

    /**
     * This method returns the raw data of the specified type for the x-axis.
     *
     * @param dataType specifies the data type.
     * @return raw data of the specified type for the x-axis.
     */
    @Override
    public SensorData getRawXData(DataType dataType)
    {
        final String funcName = "getRawXData";
        SensorData data = null;

        if (dataType == DataType.ACCELERATION)
        {
            data = new SensorData(HalUtil.getCurrentTime(), sensor.getRawData(0, dataType).value);
        }
        else
        {
            throw new UnsupportedOperationException(
                    "AndroidAccel sensor does not provide velocity or distance data.");
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp:%.3f,value:%f", data.timestamp, data.value);
        }

        return data;
    }   //getRawXData

    /**
     * This method returns the raw data of the specified type for the y-axis.
     *
     * @param dataType specifies the data type.
     * @return raw data of the specified type for the y-axis.
     */
    @Override
    public SensorData getRawYData(DataType dataType)
    {
        final String funcName = "getRawYData";
        SensorData data = null;

        if (dataType == DataType.ACCELERATION)
        {
            data = new SensorData(HalUtil.getCurrentTime(), sensor.getRawData(1, dataType).value);
        }
        else
        {
            throw new UnsupportedOperationException(
                    "AndroidAccel sensor does not provide velocity or distance data.");
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp:%.3f,value:%f", data.timestamp, data.value);
        }

        return data;
    }   //getRawYData

    /**
     * This method returns the raw data of the specified type for the z-axis.
     *
     * @param dataType specifies the data type.
     * @return raw data of the specified type for the z-axis.
     */
    @Override
    public SensorData getRawZData(DataType dataType)
    {
        final String funcName = "getRawZData";
        SensorData data = null;

        if (dataType == DataType.ACCELERATION)
        {
            data = new SensorData(HalUtil.getCurrentTime(), sensor.getRawData(2, dataType).value);
        }
        else
        {
            throw new UnsupportedOperationException(
                    "AndroidAccel sensor does not provide velocity or distance data.");
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp:%.3f,value:%f", data.timestamp, data.value);
        }

        return data;
    }   //getRawZData

}   //class FtcAndroidAccel
