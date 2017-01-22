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

import java.util.Arrays;

/**
 * This class converts cardinal data to cartesian data for sensors such as gyro or compass. It can handle sensors
 * that have one or more axes. Some value sensors such as the Modern Robotics gyro returns cardinal heading values
 * between 0.0 and 360.0. When the gyro crosses the value range boundary, it wraps around. For example, if the
 * current heading is 0.0 and the gyro turns 1 degree to the left, instead of giving you a value of -1.0, it wraps
 * to the value of 359.0. Similarly, if the current heading is 359.0 and the gyro turns 1, 2, ... degrees to the
 * right, instead of giving you a value of 360.0, 361.0, ... etc, it gives you 0.0, 1.0, ... This is undesirable
 * especially when the heading value is used in PID controlled driving. For example, if the robot wants to go
 * straight and maintain the heading of zero and the robot turned left slightly with a heading of 358.0, instead
 * of turning right 2 degrees to get back to zero heading, the robot will turn left all the way around to get back
 * to zero. This class implements a periodic task that monitor the sensor data. If it crosses the value range
 * boundary, it will keep track of the number of crossovers and will adjust the value so it doesn't wrap in effect
 * converting cardinal heading back to cartesian heading.
 */
public class TrcCardinalConverter<D> implements TrcTaskMgr.Task
{
    private static final String moduleName = "TrcCardinalConverter";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private final String instanceName;
    private TrcSensor<D> sensor;
    private D dataType;
    private int numAxes;
    private double[] cardinalRangeLows;
    private double[] cardinalRangeHighs;
    private TrcSensor.SensorData<Double>[] prevData;
    private int[] numCrossovers;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param sensor specifies the sensor object that needs data unwrapping.
     * @param dataType specifies the data type to be unwrapped.
     */
    public TrcCardinalConverter(final String instanceName, TrcSensor<D> sensor, D dataType)
    {
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        if (sensor == null)
        {
            throw new NullPointerException("sensor cannot be null.");
        }

        this.instanceName = instanceName;
        this.sensor = sensor;
        this.dataType = dataType;
        numAxes = sensor.getNumAxes();

        cardinalRangeLows = new double[numAxes];
        cardinalRangeHighs = new double[numAxes];
        prevData = new TrcSensor.SensorData[numAxes];
        numCrossovers = new int[numAxes];

        for (int i = 0; i < numAxes; i++)
        {
            cardinalRangeLows[i] = 0.0;
            cardinalRangeHighs[i] = 0.0;
            prevData[i] = new TrcSensor.SensorData<>(0.0, 0.0);
            numCrossovers[i] = 0;
        }
    }   //TrcCardinalConverter

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
     * This method enables/disables the converter task. It is not automatically enabled when created. You must
     * explicitly call this method to enable the converter.
     *
     * @param enabled specifies true for enabling the converter, disabling it otherwise.
     */
    public void setEnabled(boolean enabled)
    {
        final String funcName = "setEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enabled=%s", Boolean.toString(enabled));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (enabled)
        {
            reset();
            TrcTaskMgr.getInstance().registerTask(instanceName, this, TrcTaskMgr.TaskType.PRECONTINUOUS_TASK);
        }
        else
        {
            TrcTaskMgr.getInstance().unregisterTask(this, TrcTaskMgr.TaskType.PRECONTINUOUS_TASK);
        }
    }   //setEnabled

    /**
     * This method resets the indexed converter.
     *
     * @param index specifies the axis index.
     */
    public void reset(int index)
    {
        final String funcName = "reset";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        prevData[index] = sensor.getProcessedData(index, dataType);
        numCrossovers[index] = 0;
    }   //reset

    /**
     * This method resets the converter of all axes.
     */
    public void reset()
    {
        final String funcName = "reset";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        for (int i = 0; i < numAxes; i++)
        {
            reset(i);
        }
    }   //reset

    /**
     * This method sets the value range of the indexed converter.
     *
     * @param index specifies the axis index.
     * @param rangeLow specifies the low value of the range.
     * @param rangeHigh specifies the high value of the range.
     */
    public void setCardinalRange(int index, double rangeLow, double rangeHigh)
    {
        final String funcName = "setCardinalRange";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "low=%f,high=%f", rangeLow, rangeHigh);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (rangeLow > rangeHigh)
        {
            throw new IllegalArgumentException("cardinalRangeLow must not be greater than cardinalRangeHigh.");
        }

        cardinalRangeLows[index] = rangeLow;
        cardinalRangeHighs[index] = rangeHigh;
    }   //setCardinalRange

    /**
     * This method returns the converted indexed cartesian data.
     *
     * @param index specifies the axis index.
     * @return converted cartesian data.
     */
    public TrcSensor.SensorData<Double> getCartesianData(int index)
    {
        final String funcName = "getCartesianData";
        TrcSensor.SensorData<Double> data = new TrcSensor.SensorData<>(
                prevData[index].timestamp, prevData[index].value);

        data.value = (cardinalRangeHighs[index] - cardinalRangeLows[index])*numCrossovers[index] +
                     (data.value - cardinalRangeLows[index]);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%f", data.timestamp, data.value);
        }

        return data;
    }   //getCartesianData

    //
    // Implements TrcTaskMgr.Task
    //

    @Override
    public void startTask(TrcRobot.RunMode runMode)
    {
    }   //startTask

    @Override
    public void stopTask(TrcRobot.RunMode runMode)
    {
    }   //stopTask

    @Override
    public void prePeriodicTask(TrcRobot.RunMode runMode)
    {
    }   //prePeriodicTask

    @Override
    public void postPeriodicTask(TrcRobot.RunMode runMode)
    {
    }   //postPeriodicTask

    /**
     * This method is called periodically to check for range crossovers.
     *
     * @param runMode specifies the competition mode that is running.
     */
    @Override
    public void preContinuousTask(TrcRobot.RunMode runMode)
    {
        final String funcName = "preContinuousTask";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "mode=%s", runMode.toString());
        }

        for (int i = 0; i < numAxes; i++)
        {
            TrcSensor.SensorData<Double> data = sensor.getProcessedData(i, dataType);
            if (Math.abs(data.value - prevData[i].value) > (cardinalRangeHighs[i] - cardinalRangeLows[i])/2.0)
            {
                if (data.value > prevData[i].value)
                {
                    numCrossovers[i]--;
                }
                else
                {
                    numCrossovers[i]++;
                }
            }
            prevData[i] = data;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK,
                               "! (numCrossovers=%s)", Arrays.toString(numCrossovers));
        }
    }   //preContinuousTask

    @Override
    public void postContinuousTask(TrcRobot.RunMode runMode)
    {
    }   //postContinuousTask

}   //class TrcCardinalConverter
