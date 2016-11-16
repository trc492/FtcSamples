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

public class TrcAnalogTrigger implements TrcTaskMgr.Task
{
    private static final String moduleName = "TrcAnalogTrigger";
    private static final boolean debugEnabled = false;
    private TrcDbgTrace dbgTrace = null;

    public interface TriggerHandler
    {
        public void AnalogTriggerEvent(
                TrcAnalogTrigger analogTrigger, int zoneIndex, double zoneValue);
    }   //interface TriggerHandler

    private String instanceName;
    private TrcSensorDataSource sensor;
    private int dataIndex;
    private double[] thresholds;
    private TriggerHandler triggerHandler;
    private boolean enabled = false;
    private int zone = -1;
    private double value = 0.0;

    /**
     * Contructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param sensor specifies the sensor that is used to detect the trigger.
     * @param dataIndex specifies the data index of the sensor to read the sensor value.
     * @param triggerPoints specifies the data value trigger points array.
     * @param triggerHandler specifies the object to handle the trigger event.
     */
    public TrcAnalogTrigger(
            final String instanceName,
            TrcSensorDataSource sensor,
            int dataIndex,
            final double[] triggerPoints,
            TriggerHandler triggerHandler)
    {
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(
                    moduleName + "." + instanceName,
                    false,
                    TrcDbgTrace.TraceLevel.API,
                    TrcDbgTrace.MsgLevel.INFO);
        }

        if (sensor == null || triggerHandler == null)
        {
            throw new NullPointerException("Sensor/TriggerHandler cannot be null");
        }

        setTriggerPoints(triggerPoints);
        this.instanceName = instanceName;
        this.sensor = sensor;
        this.dataIndex = dataIndex;
        this.triggerHandler = triggerHandler;
    }   //TrcAnalogTrigger

    public void setTriggerPoints(double[] triggerPoints)
    {
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
    }   //setTriggerPoints

    public void setEnabled(boolean enabled)
    {
        final String funcName = "setEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.FUNC,
                    "enabled=%s", Boolean.toString(enabled));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC);
        }

        this.enabled = enabled;
        if (enabled)
        {
            TrcTaskMgr.getInstance().registerTask(
                    instanceName, this, TrcTaskMgr.TaskType.PRECONTINUOUS_TASK);
        }
        else
        {
            TrcTaskMgr.getInstance().unregisterTask(this, TrcTaskMgr.TaskType.PRECONTINUOUS_TASK);
        }
    }   //setEnabled

    public boolean isEnabled()
    {
        return enabled;
    }   //isEnabled

    public int getZone()
    {
        return zone;
    }   //getZone

    public double getValue()
    {
        return value;
    }   //getValue

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

    @Override
    public void preContinuousTask(TrcRobot.RunMode runMode)
    {
        final String funcName = "preContinuousTask";
        TrcSensor.SensorData data = sensor.getSensorData(dataIndex);

        if (data.value != null)
        {
            double sample;
            if (data.value instanceof Integer)
            {
                sample = (double)(Integer)data.value;
            }
            else if (data.value instanceof Double)
            {
                sample = (Double)data.value;
            }
            else
            {
                throw new NumberFormatException("Sensor data must be either integer or double.");
            }

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
                    triggerHandler.AnalogTriggerEvent(this, currZone, sample);
                }
                zone = currZone;
                value = sample;

                if (debugEnabled)
                {
                    dbgTrace.traceInfo(
                            funcName, "%s entering zone %d (value=%f)", instanceName, zone, value);
                }
            }
        }
    }   //preContinuousTask

    @Override
    public void postContinuousTask(TrcRobot.RunMode runMode)
    {
    }   //postContinuousTask

}   //class TrcAnalogTrigger
