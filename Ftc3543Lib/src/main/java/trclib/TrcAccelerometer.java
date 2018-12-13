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

package trclib;

/**
 * This class implements a platform independent accelerometer. Typically, this class is extended by a platform
 * dependent accelerometer class. The platform dependent accelerometer class must implement the abstract methods
 * required by this class. The abstract methods allow this class to get raw data for each accelerometer axis.
 * Depending on the options specified in the constructor, this class may create an integrator. The platform dependent
 * accelerometer can specify how many axes it supports by setting the HAS_AXIS options. If it does not provide
 * velocity or distance data, it can set the INTEGRATE and DOUBLE_INTEGRATE options and let the built-in integrator
 * handle it.
 */
public abstract class TrcAccelerometer extends TrcSensor<TrcAccelerometer.DataType>
{
    //
    // Accelerometer data types.
    //
    public enum DataType
    {
        ACCELERATION,
        VELOCITY,
        DISTANCE
    }   //enum DataType

    /**
     * This abstract method returns the raw data of the specified type for the x-axis.
     *
     * @param dataType specifies the data type.
     * @return raw data of the specified type for the x-axis.
     */
    public abstract SensorData<Double> getRawXData(DataType dataType);

    /**
     * This abstract method returns the raw data of the specified type for the y-axis.
     *
     * @param dataType specifies the data type.
     * @return raw data of the specified type for the y-axis.
     */
    public abstract SensorData<Double> getRawYData(DataType dataType);

    /**
     * This abstract method returns the raw data of the specified type for the z-axis.
     *
     * @param dataType specifies the data type.
     * @return raw data of the specified type for the z-axis.
     */
    public abstract SensorData<Double> getRawZData(DataType dataType);

    //
    // Accelerometer options.
    //
    public static final int ACCEL_HAS_X_AXIS            = (1 << 0);
    public static final int ACCEL_HAS_Y_AXIS            = (1 << 1);
    public static final int ACCEL_HAS_Z_AXIS            = (1 << 2);
    public static final int ACCEL_INTEGRATE             = (1 << 3);
    public static final int ACCEL_DOUBLE_INTEGRATE      = (1 << 4);

    protected final String instanceName;
    private TrcDataIntegrator<DataType> dataIntegrator = null;
    private int xIndex = -1;
    private int yIndex = -1;
    private int zIndex = -1;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param numAxes specifies the number of axes of the gyro.
     * @param options specifies the accelerometer options. Multiple options can be OR'd together.
     *                ACCEL_HAS_X_AXIS - supports x-axis.
     *                ACCEL_HAS_Y_AXIS - supports y-axis.
     *                ACCEL_HAS_Z_AXIS - supports z-axis.
     *                ACCEL_INTEGRATE - do integration on all axes to get velocities.
     *                ACCEL_DOUBLE_INTEGRATE - do double integration on all axes to get distances.
     * @param filters specifies an array of filter objects one for each supported axis. It is assumed that the order
     *                of the filters in the array is x, y and then z. If an axis is specified in the options but no
     *                filter will be used on that axis, the corresponding element in the array should be set to null.
     *                If no filter is used at all, filters can be set to null.
     */
    public TrcAccelerometer(final String instanceName, int numAxes, int options, TrcFilter[] filters)
    {
        super(instanceName, numAxes, filters);
        //
        // Count the number of axes and set up the indices for each axis.
        //
        int axisCount = 0;
        if ((options & ACCEL_HAS_X_AXIS) != 0)
        {
            xIndex = axisCount;
            axisCount++;
        }

        if ((options & ACCEL_HAS_Y_AXIS) != 0)
        {
            yIndex = axisCount;
            axisCount++;
        }

        if ((options & ACCEL_HAS_Z_AXIS) != 0)
        {
            zIndex = axisCount;
            axisCount++;
        }

        if (axisCount != numAxes)
        {
            throw new IllegalArgumentException("numAxes doesn't match the number of axes in options");
        }

        this.instanceName = instanceName;

        //
        // Create the data integrator.
        //
        if ((options & (ACCEL_INTEGRATE | ACCEL_DOUBLE_INTEGRATE)) != 0)
        {
            dataIntegrator = new TrcDataIntegrator<>(
                    instanceName, this, DataType.ACCELERATION, (options & ACCEL_DOUBLE_INTEGRATE) != 0);
        }
    }   //TrcAccelerometer

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param numAxes specifies the number of axes of the gyro.
     * @param options specifies the accelerometer options. Multiple options can be OR'd together.
     *                ACCEL_HAS_X_AXIS - supports x-axis.
     *                ACCEL_HAS_Y_AXIS - supports y-axis.
     *                ACCEL_HAS_Z_AXIS - supports z-axis.
     *                ACCEL_INTEGRATE - do integration on all axes to get velocities.
     *                ACCEL_DOUBLE_INTEGRATE - do double integration on all axes to get distances.
     */
    public TrcAccelerometer(final String instanceName, int numAxes, int options)
    {
        this(instanceName, numAxes, options, null);
    }   //TrcAccelerometer

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method enables/disables the processing of accelerometer data. It is not automatically enabled when the
     * TrcAccelerometer object is created. You need to explicitly enable the it before data processing will start.
     * As part of enabling the accelerometer, calibrate() is also called. calibrate() may be overridden by the
     * platform dependent accelerometer if it is capable of doing its own. Otherwise, calibrate will call the
     * built-in calibrator to do the calibration. Enabling/disabling data processing for the gyro involves
     * enabling/disabling the integrator if it exist.
     *
     * @param enabled specifies true if enabling, false otherwise.
     */
    public void setEnabled(boolean enabled)
    {
        final String funcName = "setEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enabled=%s", Boolean.toString(enabled));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        //
        // Enable/disable integrator.
        //
        if (dataIntegrator != null)
        {
            dataIntegrator.setEnabled(enabled);
        }
    }   //setEnabled

    /**
     * This method inverts the x-axis. This is useful if the orientation of the accelerometer x-axis is such that
     * the data goes the wrong direction.
     *
     * @param inverted specifies true to invert x-axis, false otherwise.
     */
    public void setXInverted(boolean inverted)
    {
        final String funcName = "setXInverted";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "inverted=%s", Boolean.toString(inverted));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        setInverted(xIndex, inverted);
    }   //setXInverted

    /**
     * This method inverts the y-axis. This is useful if the orientation of the accelerometer y-axis is such that
     * the data goes the wrong direction.
     *
     * @param inverted specifies true to invert y-axis, false otherwise.
     */
    public void setYInverted(boolean inverted)
    {
        final String funcName = "setYInverted";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "inverted=%s", Boolean.toString(inverted));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        setInverted(yIndex, inverted);
    }   //setYInverted

    /**
     * This method inverts the z-axis. This is useful if the orientation of the accelerometer z-axis is such that
     * the data goes the wrong direction.
     *
     * @param inverted specifies true to invert z-axis, false otherwise.
     */
    public void setZInverted(boolean inverted)
    {
        final String funcName = "setZInverted";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "inverted=%s", Boolean.toString(inverted));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        setInverted(zIndex, inverted);
    }   //setZInverted

    /**
     * This method sets the scale factor for the data of the x-axis.
     *
     * @param scale specifies the x scale factor.
     */
    public void setXScale(double scale)
    {
        final String funcName = "setXScale";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "scale=%f", scale);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        setScale(xIndex, scale, 0.0);
    }   //setXScale

    /**
     * This method sets the scale factor for the data of the y-axis.
     *
     * @param scale specifies the y scale factor.
     */
    public void setYScale(double scale)
    {
        final String funcName = "setYScale";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "scale=%f", scale);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        setScale(yIndex, scale, 0.0);
    }   //setYScale

    /**
     * This method sets the scale factor for the data of the z-axis.
     *
     * @param scale specifies the z scale factor.
     */
    public void setZScale(double scale)
    {
        final String funcName = "setZScale";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "scale=%f", scale);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        setScale(zIndex, scale, 0.0);
    }   //setZScale

    /**
     * This method returns the acceleration on the x-axis.
     *
     * @return X acceleration.
     */
    public SensorData<Double> getXAcceleration()
    {
        final String funcName = "getXAcceleration";
        SensorData<Double> data = getProcessedData(xIndex, DataType.ACCELERATION);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%f)", data.timestamp, data.value);
        }

        return data;
    }   //getXAcceleration

    /**
     * This method returns the acceleration on the y-axis.
     *
     * @return Y acceleration.
     */
    public SensorData<Double> getYAcceleration()
    {
        final String funcName = "getYAcceleration";
        SensorData<Double> data = getProcessedData(yIndex, DataType.ACCELERATION);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%f)", data.timestamp, data.value);
        }

        return data;
    }   //getYAcceleration

    /**
     * This method returns the acceleration on the z-axis.
     *
     * @return Z acceleration.
     */
    public SensorData<Double> getZAcceleration()
    {
        final String funcName = "getZAcceleration";
        SensorData<Double> data = getProcessedData(zIndex, DataType.ACCELERATION);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%f)", data.timestamp, data.value);
        }

        return data;
    }   //getZAcceleration

    /**
     * This method returns the velocity of the x-axis. If there is an integrator, we call the integrator to get the
     * velocity else we call the platform dependent accelerometer to get the raw velocity value.
     *
     * @return X velocity.
     */
    public SensorData<Double> getXVelocity()
    {
        final String funcName = "getXVelocity";
        SensorData<Double> data;

        if (dataIntegrator != null)
        {
            data = dataIntegrator.getIntegratedData(xIndex);
        }
        else
        {
            data = getRawXData(DataType.VELOCITY);
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%f)", data.timestamp, data.value);
        }

        return data;
    }   //getXVelocity

    /**
     * This method returns the velocity of the y-axis. If there is an integrator, we call the integrator to get the
     * velocity else we call the platform dependent accelerometer to get the raw velocity value.
     *
     * @return Y velocity.
     */
    public SensorData<Double> getYVelocity()
    {
        final String funcName = "getYVelocity";
        SensorData<Double> data;

        if (dataIntegrator != null)
        {
            data = dataIntegrator.getIntegratedData(yIndex);
        }
        else
        {
            data = getRawYData(DataType.VELOCITY);
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%f)", data.timestamp, data.value);
        }

        return data;
    }   //getYVelocity

    /**
     * This method returns the velocity of the z-axis. If there is an integrator, we call the integrator to get the
     * velocity else we call the platform dependent accelerometer to get the raw velocity value.
     *
     * @return Z velocity.
     */
    public SensorData<Double> getZVelocity()
    {
        final String funcName = "getZVelocity";
        SensorData<Double> data;

        if (dataIntegrator != null)
        {
            data = dataIntegrator.getIntegratedData(zIndex);
        }
        else
        {
            data = getRawZData(DataType.VELOCITY);
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%f)", data.timestamp, data.value);
        }

        return data;
    }   //getZVelocity

    /**
     * This method returns the distance of the x-axis. If there is an integrator, we call the integrator to get the
     * distance else we call the platform dependent accelerometer to get the raw distance value.
     *
     * @return X distance.
     */
    public SensorData<Double> getXDistance()
    {
        final String funcName = "getXDistance";
        SensorData<Double> data;

        if (dataIntegrator != null)
        {
            data = dataIntegrator.getDoubleIntegratedData(xIndex);
        }
        else
        {
            data = getRawXData(DataType.DISTANCE);
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%f)", data.timestamp, data.value);
        }

        return data;
    }   //getXDistance

    /**
     * This method returns the distance of the y-axis. If there is an integrator, we call the integrator to get the
     * distance else we call the platform dependent accelerometer to get the raw distance value.
     *
     * @return Y distance.
     */
    public SensorData<Double> getYDistance()
    {
        final String funcName = "getYDistance";
        SensorData<Double> data;

        if (dataIntegrator != null)
        {
            data = dataIntegrator.getDoubleIntegratedData(yIndex);
        }
        else
        {
            data = getRawYData(DataType.DISTANCE);
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%f)", data.timestamp, data.value);
        }

        return data;
    }   //getYDistance

    /**
     * This method returns the distance of the z-axis. If there is an integrator, we call the integrator to get the
     * distance else we call the platform dependent accelerometer to get the raw distance value.
     *
     * @return Z distance.
     */
    public SensorData<Double> getZDistance()
    {
        final String funcName = "getZDistance";
        SensorData<Double> data;

        if (dataIntegrator != null)
        {
            data = dataIntegrator.getDoubleIntegratedData(zIndex);
        }
        else
        {
            data = getRawZData(DataType.DISTANCE);
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%f)", data.timestamp, data.value);
        }

        return data;
    }   //getZDistance

    //
    // The following methods can be overridden by a platform dependent accelerometer class.
    //

    /**
     * This method resets the integrator on the x-axis.
     */
    public void resetXIntegrator()
    {
        final String funcName = "resetXIntegrator";

        if (dataIntegrator != null)
        {
            dataIntegrator.reset(xIndex);
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //resetXIntegrator

    /**
     * This method resets the integrator on the y-axis.
     */
    public void resetYIntegrator()
    {
        final String funcName = "resetYIntegrator";

        if (dataIntegrator != null)
        {
            dataIntegrator.reset(yIndex);
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //resetYIntegrator

    /**
     * This method resets the integrator on the z-axis.
     */
    public void resetZIntegrator()
    {
        final String funcName = "resetZIntegrator";

        if (dataIntegrator != null)
        {
            dataIntegrator.reset(zIndex);
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //resetZIntegrator

    //
    // Implements TrcSensor abstract methods.
    //

    /**
     * This abstract method returns the raw sensor data for the specified axis and type.
     *
     * @param index specifies the axis index.
     * @param dataType specifies the data type.
     * @return raw data for the specified axis.
     */
    @Override
    public SensorData<Double> getRawData(int index, DataType dataType)
    {
        final String funcName = "getRawData";
        SensorData<Double> data = null;

        if (index == xIndex)
        {
            data = getRawXData(dataType);
        }
        else if (index == yIndex)
        {
            data = getRawYData(dataType);
        }
        else if (index == zIndex)
        {
            data = getRawZData(dataType);
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.CALLBK, "index=%d", index);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.CALLBK,
                               "=(timestamp=%.3f,value=%f", data.timestamp, data.value);
        }

        return data;
    }   //getRawData

}   //class TrcAccelerometer
