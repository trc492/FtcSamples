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

import java.util.Locale;

/**
 * This class implements a platform independent value sensor that has one or more axes or data type. Typically,
 * this class is extended by a platform dependent value sensor class. The platform dependent sensor class must
 * implement the abstract methods required by this class. The abstract methods allow this class to get raw data
 * for each axis. This class also allows the caller to register an array of DataProcessors. The data processors
 * will be called one at a time in the array order to filter/process the sensor data.
 */
public abstract class TrcSensor<D>
{
    protected static final String moduleName = "TrcSensor";
    protected static final boolean debugEnabled = false;
    protected static final boolean tracingEnabled = false;
    protected static final boolean useGlobalTracer = false;
    protected static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    protected static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    protected TrcDbgTrace dbgTrace = null;

    /**
     * This class implements the SensorData object that consists of the sensor value as well as a timestamp when the
     * data sample is taken.
     *
     * @param <T> specifies the sensor value type. It could be integer, double, enum or any type.
     */
    public static class SensorData<T>
    {
        public double timestamp;
        public T value;

        /**
         * Constructor: Creates an instance of the object with the given timestamp and data value.
         *
         * @param timestamp specifies the timestamp.
         * @param value specifies the data value.
         */
        public SensorData(double timestamp, T value)
        {
            this.timestamp = timestamp;
            this.value = value;
        }   //SensorData

    }   //class SensorData

    /**
     * This interface will be implemented by sensor classes that provide multiple data types. For example, a 3-axis
     * gyro may provide "rotation rate" as well as "integrated heading" on each of its axes.
     * Normally, sensor classes should extend this class and implement its abstract method which is the same thing.
     * However, since Java supports only single inheritance and if the sensor class already extended another class,
     * it cannot extend this class. In this case, it may just implement the DataSource interface instead.
     *
     * @param <D> specifies the data type enum.
     */
    public interface DataSource<D>
    {
        /**
         * This method returns the selected raw sensor data.
         *
         * @param index specifies the index if the sensor provides some sort of array data (e.g. the axis index of a
         *              3-axis gyro).
         * @param dataType specifies the data type to return (e.g. rotation rate or heading of a gyro axis).
         * @return selected sensor data.
         */
        SensorData<?> getRawData(int index, D dataType);

    }   //interface DataSource

    /**
     * This method returns the selected raw sensor data.
     *
     * @param index specifies the index if the sensor provides some sort of array data (e.g. the axis index of a
     *              3-axis gyro).
     * @param dataType specifies the data type to return (e.g. rotation rate or heading of a gyro axis).
     * @return selected sensor data.
     */
    public abstract SensorData<?> getRawData(int index, D dataType);

    private static final int NUM_CAL_SAMPLES    = 100;
    private static final long CAL_INTERVAL      = 10;   //in msec.

    private final String instanceName;
    private int numAxes;
    private TrcFilter[] filters;
    private int[] signs;
    private double[] scales;
    private double[] offsets;
    private TrcSensorCalibrator<D> calibrator = null;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param numAxes specifies the number of axes.
     * @param filters specifies an array of filter objects, one for each axis, to filter sensor data.
     *                If no filter is used, this can be set to null.
     */
    public TrcSensor(final String instanceName, int numAxes, TrcFilter[] filters)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }
        //
        // Make sure we have at least one axis.
        //
        if (numAxes <= 0)
        {
            throw new IllegalArgumentException("Sensor must have at least one axis.");
        }
        //
        // If no filters are provided, create an array of null filters.
        //
        if (filters == null)
        {
            filters = new TrcFilter[numAxes];
        }
        //
        // Make sure the filter array must have numAxes elements. Even if we don't filter on some axes, we still
        // must have numAxes elements but the elements of those axes can be null.
        //
        if (filters.length != numAxes)
        {
            throw new IllegalArgumentException(
                    String.format(Locale.US, "filters must be an array of %d elements.", numAxes));
        }

        this.instanceName = instanceName;
        this.numAxes = numAxes;
        this.filters = filters;
        signs = new int[numAxes];
        scales = new double[numAxes];
        offsets = new double[numAxes];
        for (int i = 0; i < numAxes; i++)
        {
            signs[i] = 1;
            scales[i] = 1.0;
            offsets[i] = 0.0;
        }
    }   //TrcSensor

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param numAxes specifies the number of axes.
     */
    public TrcSensor(final String instanceName, int numAxes)
    {
        this(instanceName, numAxes, null);
    }   //TrcSensor

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @Override
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method returns the number of axes of the sensor.
     *
     * @return number of axes.
     */
    public int getNumAxes()
    {
        final String funcName = "getNumAxes";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%d", numAxes);
        }

        return numAxes;
    }   //getNumAxes

    /**
     * This method inverts the specified axis of the sensor. This is useful if the orientation of the sensor axis
     * is such that the data goes the wrong direction, if the sensor is mounted up-side-down, for example.
     *
     * @param index specifies the axis index.
     * @param inverted specifies true to invert the axis, false otherwise.
     */
    public void setInverted(int index, boolean inverted)
    {
        final String funcName = "setInverted";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                                "index=%d,inverted=%s", index, Boolean.toString(inverted));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        signs[index] = inverted? -1: 1;
    }   //setInverted

    /**
     * This method sets the scale factor for the data of the specified axis.
     *
     * @param index specifies the axis index.
     * @param scale specifies the scale factor for the axis.
     * @param offset specifies the offset to be subtracted from the scaled data.
     */
    public void setScale(int index, double scale, double offset)
    {
        final String funcName = "setScale";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "index=%d,scale=%f", index, scale);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        scales[index] = scale;
        offsets[index] = offset;
    }   //setScale

    /**
     * This method calibrates the sensor by creating a calibrator if none exist yet. It then calls the calibrator
     * to do the calibration.
     *
     * @param numCalSamples specifies the number of calibration sample to take.
     * @param calInterval specifies the interval between each calibration sample in msec.
     * @param dataType specifies the data type needed calibration.
     */
    protected void calibrate(int numCalSamples, long calInterval, D dataType)
    {
        if (calibrator == null)
        {
            calibrator = new TrcSensorCalibrator<>(instanceName, this, numAxes, dataType);
        }

        calibrator.calibrate(numCalSamples, calInterval);
    }   //calibrate

    /**
     * This method calibrates the sensor by creating a calibrator if none exist yet. It then calls the calibrator
     * to do the calibration.
     *
     * @param dataType specifies the data type needed calibration.
     */
    protected void calibrate(D dataType)
    {
        calibrate(NUM_CAL_SAMPLES, CAL_INTERVAL, dataType);
    }   //calibrate

    /**
     * This method always returns false because the built-in calibrator is synchronous.
     *
     * @return false.
     */
    public boolean isCalibrating()
    {
        final String funcName = "isCalibrating";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=false");
        }
        //
        // The built-in calibrator is synchronous, so we always return false.
        //
        return false;
    }   //isCalibrating

    /**
     * This method returns the processed data for the specified axis and type. The data will go through a filter
     * if a filter is supplied for the axis. The calibration data will be applied to the sensor data if applicable.
     * The sign and scale will also be applied.
     *
     * @param index specifies the axis index.
     * @param dataType specifies the data type object.
     * @return processed sensor data for the axis.
     */
    public SensorData<Double> getProcessedData(int index, D dataType)
    {
        final String funcName = "getProcessedData";
        @SuppressWarnings("unchecked")
        SensorData<Double> data = (SensorData<Double>)getRawData(index, dataType);

        if (data != null)
        {
            double value = data.value;

            if (debugEnabled) dbgTrace.traceInfo(funcName, "raw=%.3f", value);
            if (filters[index] != null)
            {
                value = filters[index].filterData(value);
                if (debugEnabled) dbgTrace.traceInfo(funcName, "filtered=%.3f", value);
            }

            if (calibrator != null)
            {
                value = calibrator.getCalibratedData(index, value);
                if (debugEnabled) dbgTrace.traceInfo(funcName, "calibrated=%.3f", value);
            }

            value *= signs[index]*scales[index] + offsets[index];
            if (debugEnabled) dbgTrace.traceInfo(
                funcName, "scaled=%.3f (scale=%f,offset=%f)", value, scales[index], offsets[index]);
            data.value = value;

            if (debugEnabled)
            {
                dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "index=%d", index);
                dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                                   "=(timestamp=%0.3f,value=%f", data.timestamp, value);
            }
        }

        return data;
    }   //getProcessedData

}   //class TrcSensor
