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
public class TrcCardinalConverter<D>
{
    private static final String moduleName = "TrcCardinalConverter";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private final String instanceName;
    private final TrcSensor<D> sensor;
    private final D dataType;
    private final int numAxes;
    private final TrcTaskMgr.TaskObject converterTaskObj;
    private final double[] cardinalRangeLows;
    private final double[] cardinalRangeHighs;
    private final TrcSensor.SensorData<Double>[] prevData;
    private final int[] numCrossovers;
    private boolean enabled = false;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param sensor specifies the sensor object that needs data unwrapping.
     * @param dataType specifies the data type to be unwrapped.
     */
    @SuppressWarnings("unchecked")
    public TrcCardinalConverter(final String instanceName, final TrcSensor<D> sensor, final D dataType)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        if (sensor == null)
        {
            throw new NullPointerException("sensor cannot be null.");
        }

        this.instanceName = instanceName;
        this.sensor = sensor;
        this.dataType = dataType;
        numAxes = sensor.getNumAxes();
        converterTaskObj = TrcTaskMgr.getInstance().createTask(
                instanceName + ".converterTask", this::converterTask);

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
     * This method returns the state of the cardinal converter task.
     *
     * @return true if converter task is enabled, false otherwise.
     */
    public synchronized boolean isEnabled()
    {
        final String funcName = "isEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Boolean.toString(enabled));
        }

        return enabled;
    }   //isEnabled

    /**
     * This method enables/disables the converter task. It is not automatically enabled when created. You must
     * explicitly call this method to enable the converter.
     *
     * @param enabled specifies true for enabling the converter, disabling it otherwise.
     */
    public synchronized void setEnabled(boolean enabled)
    {
        final String funcName = "setEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enabled=%b", enabled);
        }

        if (!this.enabled && enabled)
        {
            reset();
            converterTaskObj.registerTask(TrcTaskMgr.TaskType.PRECONTINUOUS_TASK);//STANDALONE_TASK, 100);
        }
        else if (this.enabled && !enabled)
        {
            reset();
            converterTaskObj.unregisterTask(TrcTaskMgr.TaskType.PRECONTINUOUS_TASK);//STANDALONE_TASK);
        }
        this.enabled = enabled;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setEnabled

    /**
     * This method resets the indexed converter.
     *
     * @param index specifies the axis index.
     */
    public synchronized void reset(int index)
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
    public synchronized void reset()
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
    public synchronized void setCardinalRange(int index, double rangeLow, double rangeHigh)
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
        reset(index);
    }   //setCardinalRange

    /**
     * This method returns the converted indexed cartesian data.
     *
     * @param index specifies the axis index.
     * @return converted cartesian data.
     */
    public synchronized TrcSensor.SensorData<Double> getCartesianData(int index)
    {
        final String funcName = "getCartesianData";
        TrcSensor.SensorData<Double> data = new TrcSensor.SensorData<>(
                prevData[index].timestamp, prevData[index].value);

        if (enabled)
        {
            data.value += (cardinalRangeHighs[index] - cardinalRangeLows[index])*numCrossovers[index];
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%f", data.timestamp, data.value);
        }

        return data;
    }   //getCartesianData

    /**
     * This method is called periodically to check for range crossovers.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is running.
     */
    private synchronized void converterTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = "converterTask";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "taskType=%s,runMode=%s", taskType, runMode);
        }

        for (int i = 0; i < numAxes; i++)
        {
            TrcSensor.SensorData<Double> data = sensor.getProcessedData(i, dataType);

            if (data != null)
            {
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
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK,
                "! (numCrossovers=%s)", Arrays.toString(numCrossovers));
        }
    }   //converterTask

}   //class TrcCardinalConverter
