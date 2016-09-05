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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.HardwareMap;

import hallib.HalUtil;
import trclib.TrcDbgTrace;
import trclib.TrcFilter;
import trclib.TrcGyro;

/**
 * This class implements the Modern Robotics gyro extending TrcGyro.
 * It provides implementation of the abstract methods in TrcGyro.
 * The Modern Robotics gyro supports 3 axes: x, y and z. It provides
 * rotation rate data for all 3 axes. However, it only provides heading
 * data for the z-axis and the heading data is wrap-around.
 */
public class FtcMRGyro extends TrcGyro
{
    private static final String moduleName = "FtcMRGyro";
    private static final boolean debugEnabled = false;
    private TrcDbgTrace dbgTrace = null;

    private ModernRoboticsI2cGyro gyro;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     * @param filters specifies an array of filters to use for filtering
     *                sensor noise, one for each axis. Since we have 3 axes,
     *                the array should have 3 elements. If no filters are
     *                used, it can be set to null.
     */
    public FtcMRGyro(HardwareMap hardwareMap, String instanceName, TrcFilter[] filters)
    {
        super(instanceName,
              3,
              GYRO_HAS_X_AXIS | GYRO_HAS_Y_AXIS | GYRO_HAS_Z_AXIS,
              filters);

        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName + "." + instanceName,
                                       false,
                                       TrcDbgTrace.TraceLevel.API,
                                       TrcDbgTrace.MsgLevel.INFO);
        }

        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get(instanceName);
    }   //FtcMRGyro

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param filters specifies an array of filters to use for filtering
     *                sensor noise, one for each axis. Since we have 3 axes,
     *                the array should have 3 elements. If no filters are
     *                used, it can be set to null.
     */
    public FtcMRGyro(String instanceName, TrcFilter[] filters)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, filters);
    }   //FtcMRGyro

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FtcMRGyro(String instanceName)
    {
        this(instanceName, null);
    }   //FtcMRGyro

    /**
     * This method calibrates the sensor.
     */
    public void calibrate()
    {
        final String funcName = "calibrate";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        gyro.calibrate();
        while (gyro.isCalibrating())
        {
            HalUtil.sleep(10);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //calibrate

    //
    // Overriding TrcGyro methods.
    //

    /**
     * This method overrides the TrcGyro class. It doesn't have an x-integrator.
     */
    @Override
    public void resetXIntegrator()
    {
        final String funcName = "resetXIntegrator";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //resetXIntegrator

    /**
     * This method overrides the TrcGyro class. It doesn't have an y-integrator.
     */
    @Override
    public void resetYIntegrator()
    {
        final String funcName = "resetYIntegrator";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //resetYIntegrator

    /**
     * This method overrides the TrcGyro class and calls its own.
     */
    @Override
    public void resetZIntegrator()
    {
        final String funcName = "resetZIntegrator";

        gyro.resetZAxisIntegrator();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //resetZIntegrator

    //
    // Implements TrcGyro abstract methods.
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
        double value = 0.0;

        //
        // MR gyro supports only rotation rate for the x-axis.
        //
        if (dataType == DataType.ROTATION_RATE)
        {
            value = gyro.rawX();
        }
        SensorData data = new SensorData(HalUtil.getCurrentTime(), value);

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
        double value = 0.0;

        //
        // MR gyro supports only rotation rate for the x-axis.
        //
        if (dataType == DataType.ROTATION_RATE)
        {
            value = gyro.rawY();
        }
        SensorData data = new SensorData(HalUtil.getCurrentTime(), value);

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
        double value = 0.0;

        if (dataType == DataType.ROTATION_RATE)
        {
            value = gyro.rawZ();
        }
        else if (dataType == DataType.HEADING)
        {
            value = -gyro.getIntegratedZValue();
        }
        SensorData data = new SensorData(HalUtil.getCurrentTime(), value);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp:%.3f,value:%f", data.timestamp, data.value);
        }

        return data;
    }   //getRawZData

}   //class FtcMRGyro
