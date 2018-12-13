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
 * This class implements an AnalogTrigger. It monitors the value of the analog sensor against an array of threshold
 * values. If the sensor reading crosses any of the thresholds in the array, it will call a notification handler so
 * that an action could be performed.
 */
public class TrcAnalogTrigger<D>
{
    private static final String moduleName = "TrcAnalogTrigger";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    /**
     * This interface contains the notification handler to be called when the sensor reading crosses a threshold in
     * the array.
     */
    public interface TriggerHandler
    {
        /**
         * This method is called when a threshold has been crossed.
         *
         * @param currZone specifies the zone it is going into.
         * @param prevZone specifies the zone it is coming out of.
         * @param zoneValue specifies the actual sensor value.
         */
        void triggerEvent(int currZone, int prevZone, double zoneValue);

    }   //interface TriggerHandler

    private final String instanceName;
    private final TrcSensor<D> sensor;
    private final int index;
    private final D dataType;
    private final TriggerHandler triggerHandler;
    private final TrcTaskMgr.TaskObject triggerTaskObj;
    private double[] thresholds;
    private boolean enabled = false;
    private int zone = -1;
    private double value = 0.0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param sensor specifies the sensor that is used to detect the trigger.
     * @param index specifies the data index of the sensor to read the sensor value.
     * @param dataType specifies the data type of the sensor to read the sensor value.
     * @param triggerPoints specifies the data value trigger points array.
     * @param triggerHandler specifies the object to handle the trigger event.
     */
    public TrcAnalogTrigger(
        final String instanceName, final TrcSensor<D> sensor, final int index, final D dataType,
        final double[] triggerPoints, final TriggerHandler triggerHandler)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        if (sensor == null || triggerHandler == null)
        {
            throw new NullPointerException("Sensor/TriggerHandler cannot be null");
        }

        setTriggerPoints(triggerPoints);
        this.instanceName = instanceName;
        this.sensor = sensor;
        this.index = index;
        this.dataType = dataType;
        this.triggerHandler = triggerHandler;
        triggerTaskObj = TrcTaskMgr.getInstance().createTask(instanceName + ".triggerTask", this::triggerTask);
    }   //TrcAnalogTrigger

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
     * This method creates and threshold array and calculates all the threshold values. A threshold value is the
     * average of two adjacent trigger points.
     *
     * @param triggerPoints specifies the array of trigger points.
     */
    public synchronized void setTriggerPoints(double[] triggerPoints)
    {
        final String funcName = "setTriggerPoints";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "triggerPts=%s", Arrays.toString(triggerPoints));
        }

        if (triggerPoints == null)
        {
            throw new NullPointerException("TriggerPoints cannot be null");
        }

        if (triggerPoints.length < 2)
        {
            throw new IllegalArgumentException("zoneValues must have at least two elements.");
        }

        thresholds = new double[triggerPoints.length - 1];
        for (int i = 0; i < thresholds.length; i++)
        {
            thresholds[i] = (triggerPoints[i] + triggerPoints[i + 1])/2.0;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Arrays.toString(thresholds));
        }
    }   //setTriggerPoints

    /**
     * This method enables/disables the task that monitors the sensor value.
     *
     * @param enabled specifies true to enable, false to disable.
     */
    public synchronized void setEnabled(boolean enabled)
    {
        final String funcName = "setEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC, "enabled=%b", enabled);
        }

        this.enabled = enabled;

        if (enabled)
        {
            zone = -1;
            value = 0.0;
            triggerTaskObj.registerTask(TrcTaskMgr.TaskType.PRECONTINUOUS_TASK);    //TODO: should use INPUT_TASK
        }
        else
        {
            triggerTaskObj.unregisterTask(TrcTaskMgr.TaskType.PRECONTINUOUS_TASK);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC);
        }
    }   //setEnabled

    /**
     * This method checks if the task is enabled.
     *
     * @return true if enabled, false otherwise.
     */
    public synchronized boolean isEnabled()
    {
        return enabled;
    }   //isActive

    /**
     * This method returns the current zone it is in.
     *
     * @return current zone index.
     */
    public synchronized int getZone()
    {
        return zone;
    }   //getZone

    /**
     * This method returns the last sensor value.
     *
     * @return last sensor value.
     */
    public synchronized double getValue()
    {
        return value;
    }   //getValue

    /**
     * This method is called periodically to check the current sensor value against the threshold array to see it
     * crosses a new threshold.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is running. (e.g. Autonomous, TeleOp, Test).
     */
    private synchronized void triggerTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = "triggerTask";
        TrcSensor.SensorData<Double> data = sensor.getProcessedData(index, dataType);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "taskType=%s,runMode=%s", taskType, runMode);
        }

        if (data != null && data.value != null)
        {
            double sample = (double)data.value;
            int currZone = -1;

            if (sample < thresholds[0])
            {
                currZone = 0;
            }
            else
            {
                for (int i = 0; i < thresholds.length - 1; i++)
                {
                    if (sample >= thresholds[i] && sample < thresholds[i + 1])
                    {
                        currZone = i + 1;
                        break;
                    }
                }

                if (currZone == -1)
                {
                    currZone = thresholds.length;
                }
            }

            if (currZone != zone)
            {
                //
                // We have crossed to another zone, let's notify somebody.
                //
                if (triggerHandler != null)
                {
                    triggerHandler.triggerEvent(currZone, zone, sample);
                }

                if (debugEnabled)
                {
                    dbgTrace.traceInfo(funcName, "%s going to zone %d from zone %d (value=%f)",
                        instanceName, currZone, zone, value);
                }

                zone = currZone;
                value = sample;
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //triggerTask

}   //class TrcAnalogTrigger
