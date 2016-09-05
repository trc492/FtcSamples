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

import hallib.HalGyro;

/**
 * This class implements a platform independent gyro. Typically, this
 * class is extended by a platform dependent gyro class. The platform
 * dependent gyro class must implement the abstract methods required
 * by this class. The abstract methods allow this class to get raw data
 * for each gyro axis. Depending on the options specified in the
 * constructor, this class creates an integrator or an unwrapper.
 * The platform dependent gyro can specify how many axes it supports by
 * setting the HAS_AXIS options. If it does not provide heading data, it can
 * set the INTEGRATE options and let the built-in integrator handle it. Or
 * if the heading data it provides wrap-around, it can set the UNWRAP_HEADING
 * options to enable the unwrapper to unwrap the heading data.
 */
public abstract class TrcGyro extends TrcSensor implements HalGyro,
                                                           TrcSensorDataSource
{
    //
    // Gyro data types.
    //
    public enum DataType
    {
        ROTATION_RATE,
        HEADING
    }   //enum DataType

    /**
     * This abstract method returns the raw data with the specified type of the x-axis.
     *
     * @param dataType specifies the data type.
     * @return raw data with the specified type of the x-axis.
     */
    public abstract SensorData getRawXData(DataType dataType);

    /**
     * This abstract method returns the raw data with the specified type of the y-axis.
     *
     * @param dataType specifies the data type.
     * @return raw data with the specified type of the y-axis.
     */
    public abstract SensorData getRawYData(DataType dataType);

    /**
     * This abstract method returns the raw data with the specified type of the z-axis.
     *
     * @param dataType specifies the data type.
     * @return raw data with the specified type of the z-axis.
     */
    public abstract SensorData getRawZData(DataType dataType);

    //
    // Gyro options.
    //
    public static final int GYRO_HAS_X_AXIS             = (1 << 0);
    public static final int GYRO_HAS_Y_AXIS             = (1 << 1);
    public static final int GYRO_HAS_Z_AXIS             = (1 << 2);
    public static final int GYRO_INTEGRATE              = (1 << 3);
    public static final int GYRO_UNWRAP_HEADING         = (1 << 4);

    private static final String moduleName = "TrcGyro";
    private static final boolean debugEnabled = false;
    private TrcDbgTrace dbgTrace = null;

    private final String instanceName;
    private TrcDataIntegrator dataIntegrator = null;
    private TrcDataUnwrapper dataUnwrapper = null;
    private int xIndex = -1;
    private int yIndex = -1;
    private int zIndex = -1;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param numAxes specifies the number of axes of the gyro.
     * @param options specifies the gyro options. Multiple options can be OR'd together.
     *                GYRO_HAS_X_AXIS - supports x-axis.
     *                GYRO_HAS_Y_AXIS - supports y-axis.
     *                GYRO_HAS_Z_AXIS - supports z-axis.
     *                GYRO_INTEGRATE - do integration on all axes to get headings.
     *                GYRO_UNWRAP_HEADING - unwrap heading on all axes.
     * @param filters specifies an array of filter objects one for each supported axis.
     *                It is assumed that the order of the filters in the array is x, y
     *                and then z. If an axis is specified in the options but no filter
     *                will be used on that axis, the corresponding element in the array
     *                should be set to null. If no filter is used at all, filters can
     *                be set to null.
     */
    public TrcGyro(final String instanceName, int numAxes, int options, TrcFilter[] filters)
    {
        super(instanceName, numAxes, filters);

        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName + "." + instanceName,
                                       false,
                                       TrcDbgTrace.TraceLevel.API,
                                       TrcDbgTrace.MsgLevel.INFO);
        }

        //
        // Count the number of axes and set up the indices for each axis.
        //
        int axisCount = 0;
        if ((options & GYRO_HAS_X_AXIS) != 0)
        {
            xIndex = axisCount;
            axisCount++;
        }

        if ((options & GYRO_HAS_Y_AXIS) != 0)
        {
            yIndex = axisCount;
            axisCount++;
        }

        if ((options & GYRO_HAS_Z_AXIS) != 0)
        {
            zIndex = axisCount;
            axisCount++;
        }

        if (axisCount != numAxes)
        {
            throw new IllegalArgumentException(
                    "numAxes doesn't match the number of axes in options");
        }

        //
        // Integration of rate and unwrapping of heading are mutually exclusive.
        // If we are doing software integration, the resulting heading is not
        // wrap-around. If we need to unwrap heading, the heading is from the
        // physical sensor and not from the integrator.
        //
        if ((options & GYRO_INTEGRATE) != 0 && (options & GYRO_UNWRAP_HEADING) != 0)
        {
            throw new IllegalArgumentException(
                    "Options Integrate and Unwrap cannot coexist.");
        }

        this.instanceName = instanceName;

        //
        // Create the data integrator.
        //
        if ((options & GYRO_INTEGRATE) != 0)
        {
            dataIntegrator = new TrcDataIntegrator(
                    instanceName, this, DataType.ROTATION_RATE, false);
        }

        //
        // Create the data unwrapper.
        //
        if ((options & GYRO_UNWRAP_HEADING) != 0)
        {
            dataUnwrapper = new TrcDataUnwrapper(instanceName, this, DataType.HEADING);
        }
    }   //TrcGyro

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param numAxes specifies the number of axes of the gyro.
     * @param options specifies the gyro options. Multiple options can be OR'd together.
     *                GYRO_HAS_X_AXIS - supports x-axis.
     *                GYRO_HAS_Y_AXIS - supports y-axis.
     *                GYRO_HAS_Z_AXIS - supports z-axis.
     *                GYRO_INTEGRATE - do integration on all axes to get headings.
     *                GYRO_UNWRAP_HEADING - unwrap heading on all axes.
     */
    public TrcGyro(final String instanceName, int numAxes, int options)
    {
        this(instanceName, numAxes, options, null);
    }   //TrcGyro

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
     * This method enables/disables the processing of gyro data. It is not
     * automatically enabled when the TrcGyro object is created. You need
     * to explicitly enable the it before data processing will start. As
     * part of enabling the gyro, calibrate() is also called. calibrate()
     * may be overridden by the platform dependent gyro if it is capable
     * of doing its own. Otherwise, calibrate will call the built-in
     * calibrator to do the calibration.
     * Enabling/disabling data processing for the gyro involves
     * enabling/disabling the integrator and the unwrapper if
     * they exist.
     *
     * @param enabled specifies true if enabling, false otherwise.
     */
    public void setEnabled(boolean enabled)
    {
        final String funcName = "setEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                                "enabled=%s", Boolean.toString(enabled));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        //
        // Enable/disable integrator.
        //
        if (dataIntegrator != null)
        {
            dataIntegrator.setEnabled(enabled);
        }

        //
        // Enable/disable unwrapper.
        //
        if (dataUnwrapper != null)
        {
            dataUnwrapper.setEnabled(enabled);
        }
    }   //setEnabled

    /**
     * This method inverts the x-axis. This is useful if the orientation of
     * the gyro x-axis is such that the data goes the wrong direction.
     *
     * @param inverted specifies true to invert x-axis, false otherwise.
     */
    public void setXInverted(boolean inverted)
    {
        final String funcName = "setXInverted";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                                "inverted=%s", Boolean.toString(inverted));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        setInverted(xIndex, inverted);
    }   //setXInverted

    /**
     * This method inverts the y-axis. This is useful if the orientation of
     * the gyro y-axis is such that the data goes the wrong direction.
     *
     * @param inverted specifies true to invert y-axis, false otherwise.
     */
    public void setYInverted(boolean inverted)
    {
        final String funcName = "setYInverted";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                                "inverted=%s", Boolean.toString(inverted));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        setInverted(yIndex, inverted);
    }   //setYInverted

    /**
     * This method inverts the z-axis. This is useful if the orientation of
     * the gyro z-axis is such that the data goes the wrong direction.
     *
     * @param inverted specifies true to invert z-axis, false otherwise.
     */
    public void setZInverted(boolean inverted)
    {
        final String funcName = "setZInverted";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                                "inverted=%s", Boolean.toString(inverted));
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

        setScale(xIndex, scale);
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

        setScale(yIndex, scale);
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

        setScale(zIndex, scale);
    }   //setZScale

    /**
     * This method sets the heading value range of the x-axis.
     * The value range is used by the unwrapper to unwrap heading
     * data.
     *
     * @param valueRangeLow specifies the low value of the x-axis range.
     * @param valueRangeHigh specifies the high value of the x-axis range.
     */
    public void setXValueRange(double valueRangeLow, double valueRangeHigh)
    {
        final String funcName = "setXValueRange";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                                "low=%f,high=%f", valueRangeLow, valueRangeHigh);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (dataUnwrapper != null)
        {
            dataUnwrapper.setValueRange(xIndex, valueRangeLow, valueRangeHigh);
        }
    }   //setXValueRange

    /**
     * This method sets the heading value range of the y-axis.
     * The value range is used by the unwrapper to unwrap heading
     * data.
     *
     * @param valueRangeLow specifies the low value of the y-axis range.
     * @param valueRangeHigh specifies the high value of the y-axis range.
     */
    public void setYValueRange(double valueRangeLow, double valueRangeHigh)
    {
        final String funcName = "setYValueRange";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                                "low=%f,high=%f", valueRangeLow, valueRangeHigh);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (dataUnwrapper != null)
        {
            dataUnwrapper.setValueRange(yIndex, valueRangeLow, valueRangeHigh);
        }
    }   //setYValueRange

    /**
     * This method sets the heading value range of the z-axis.
     * The value range is used by the unwrapper to unwrap heading
     * data.
     *
     * @param valueRangeLow specifies the low value of the z-axis range.
     * @param valueRangeHigh specifies the high value of the z-axis range.
     */
    public void setZValueRange(double valueRangeLow, double valueRangeHigh)
    {
        final String funcName = "setZValueRange";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                                "low=%f,high=%f", valueRangeLow, valueRangeHigh);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (dataUnwrapper != null)
        {
            dataUnwrapper.setValueRange(zIndex, valueRangeLow, valueRangeHigh);
        }
    }   //setZValueRange

    /**
     * This method resets the unwrapper on the x-axis.
     */
    public void resetXUnwrapper()
    {
        final String funcName = "resetXUnwrapper";

        if (dataUnwrapper != null)
        {
            dataUnwrapper.reset(xIndex);
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //resetXUnwrapper

    /**
     * This method resets the unwrapper on the y-axis.
     */
    public void resetYUnwrapper()
    {
        final String funcName = "resetYUnwrapper";

        if (dataUnwrapper != null)
        {
            dataUnwrapper.reset(yIndex);
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //resetYUnwrapper

    /**
     * This method resets the unwrapper on the z-axis.
     */
    public void resetZUnwrapper()
    {
        final String funcName = "resetZUnwrapper";

        if (dataUnwrapper != null)
        {
            dataUnwrapper.reset(zIndex);
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //resetZUnwrapper

    /**
     * This method returns the rotation rate on the x-axis.
     *
     * @return X rotation rate.
     */
    public SensorData getXRotationRate()
    {
        final String funcName = "getXRotationRate";
        SensorData data = getData(xIndex, DataType.ROTATION_RATE);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%f)", data.timestamp, data.value);
        }

        return data;
    }   //getXRotationRate

    /**
     * This method returns the rotation rate on the y-axis.
     *
     * @return Y rotation rate.
     */
    public SensorData getYRotationRate()
    {
        final String funcName = "getYRotationRate";
        SensorData data = getData(yIndex, DataType.ROTATION_RATE);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%f)", data.timestamp, data.value);
        }

        return data;
    }   //getYRotationRate

    /**
     * This method returns the rotation rate on the z-axis.
     *
     * @return Z rotation rate.
     */
    public SensorData getZRotationRate()
    {
        final String funcName = "getZRotationRate";
        SensorData data = getData(zIndex, DataType.ROTATION_RATE);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%f)", data.timestamp, data.value);
        }

        return data;
    }   //getZRotationRate

    /**
     * This method returns the heading of the x-axis. If there is an integrator,
     * we call the integrator to get the heading. Else if we have an unwrapper,
     * we call the unwrapper to get the heading else we call the platform dependent
     * gyro to get the raw heading value.
     *
     * @return X heading.
     */
    public SensorData getXHeading()
    {
        final String funcName = "getXHeading";
        SensorData data = null;

        if (dataIntegrator != null)
        {
            data = dataIntegrator.getIntegratedData(xIndex);
        }
        else if (dataUnwrapper != null)
        {
            data = dataUnwrapper.getUnwrappedData(xIndex);
        }
        else
        {
            data = getRawXData(DataType.HEADING);
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%f)", data.timestamp, data.value);
        }

        return data;
    }   //getXHeading

    /**
     * This method returns the heading of the y-axis. If there is an integrator,
     * we call the integrator to get the heading. Else if we have an unwrapper,
     * we call the unwrapper to get the heading else we call the platform dependent
     * gyro to get the raw heading value.
     *
     * @return Y heading.
     */
    public SensorData getYHeading()
    {
        final String funcName = "getYHeading";
        SensorData data = null;

        if (dataIntegrator != null)
        {
            data = dataIntegrator.getIntegratedData(yIndex);
        }
        else if (dataUnwrapper != null)
        {
            data = dataUnwrapper.getUnwrappedData(yIndex);
        }
        else
        {
            data = getRawYData(DataType.HEADING);
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%f)", data.timestamp, data.value);
        }

        return data;
    }   //getYHeading

    /**
     * This method returns the heading of the z-axis. If there is an integrator,
     * we call the integrator to get the heading. Else if we have an unwrapper,
     * we call the unwrapper to get the heading else we call the platform dependent
     * gyro to get the raw heading value.
     *
     * @return Z heading.
     */
    public SensorData getZHeading()
    {
        final String funcName = "getZHeading";
        SensorData data = null;

        if (dataIntegrator != null)
        {
            data = dataIntegrator.getIntegratedData(zIndex);
        }
        else if (dataUnwrapper != null)
        {
            data = dataUnwrapper.getUnwrappedData(zIndex);
        }
        else
        {
            data = getRawZData(DataType.HEADING);
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%f)", data.timestamp, data.value);
        }

        return data;
    }   //getZHeading

    //
    // The following methods can be overridden by a platform dependent gyro class.
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
     * @return raw data for the specified axis and type.
     */
    @Override
    public SensorData getRawData(int index, Object dataType)
    {
        final String funcName = "getRawData";
        SensorData data = null;

        if (index == xIndex)
        {
            data = getRawXData((DataType)dataType);
        }
        else if (index == yIndex)
        {
            data = getRawYData((DataType)dataType);
        }
        else if (index == zIndex)
        {
            data = getRawZData((DataType)dataType);
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.CALLBK, "index=%d", index);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.CALLBK,
                               "=(timestamp=%.3f,value=%f", data.timestamp, data.value);
        }

        return data;
    }   //getRawData

    //
    // Implements TrcSensorDataSource interface.
    //

    /**
     * This method returns the sensor data of the specified index.
     *
     * @param index specifies the data index.
     * @return sensor data of the specified index.
     */
    @Override
    public TrcSensor.SensorData getSensorData(int index)
    {
        final String funcName = "getSensorData";
        TrcSensor.SensorData data = null;

        switch (index)
        {
            case 0:
                data = getXRotationRate();
                break;

            case 1:
                data = getYRotationRate();
                break;

            case 2:
                data = getZRotationRate();
                break;

            case 3:
                data = getXHeading();
                break;

            case 4:
                data = getYHeading();
                break;

            case 5:
                data = getZHeading();
                break;
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "index=%d", index);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(time=%.3f,value=%f)", data.timestamp, data.value);
        }

        return data;
    }   //getSensorData

}   //class TrcGyro
