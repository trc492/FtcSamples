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

import hallib.HalUtil;

/**
 * This class does data integration for sensors that have one or more axes.
 * Some value sensors such as gyros and accelerometers may need to integrate
 * their data to provide heading from gyro rotation rate, and velocity or
 * distance from accelerometer acceleration data. This class uses a periodic
 * task to do integration and optionally double integration.
 */
public class TrcDataIntegrator implements TrcTaskMgr.Task
{
    private static final String moduleName = "TrcDataIntegrator";
    private static final boolean debugEnabled = false;
    private TrcDbgTrace dbgTrace = null;

    private final String instanceName;
    private TrcSensor sensor;
    private Object dataType;
    private int numAxes;
    private TrcSensor.SensorData[] inputData;
    private TrcSensor.SensorData[] integratedData;
    private TrcSensor.SensorData[] doubleIntegratedData;
    private double[] prevTimes;
    private boolean unwindIntegratedData = false;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param sensor specifies the sensor object that needs integration.
     * @param dataType specifies the data type to be integrated.
     * @param doubleIntegration specifies true to do double integration, false otherwise.
     */
    public TrcDataIntegrator(
            final String instanceName,
            TrcSensor sensor,
            Object dataType,
            boolean doubleIntegration)
    {
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(
                    moduleName + "." + instanceName,
                    false,
                    TrcDbgTrace.TraceLevel.API,
                    TrcDbgTrace.MsgLevel.INFO);
        }

        if (sensor == null)
        {
            throw new NullPointerException("sensor cannot be null.");
        }

        this.instanceName = instanceName;
        this.sensor = sensor;
        this.dataType = dataType;
        numAxes = sensor.getNumAxes();

        inputData = new TrcSensor.SensorData[numAxes];
        integratedData = new TrcSensor.SensorData[numAxes];
        doubleIntegratedData = doubleIntegration? new TrcSensor.SensorData[numAxes]: null;
        prevTimes = new double[numAxes];

        for (int i = 0; i < numAxes; i++)
        {
            integratedData[i] = new TrcSensor.SensorData(0.0, 0.0);
            if (doubleIntegratedData != null)
            {
                doubleIntegratedData[i] = new TrcSensor.SensorData(0.0, 0.0);
            }
            prevTimes[i] = 0.0;
        }
    }   //TrcDataIntegrator

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param sensor specifies the sensor object that needs integration.
     * @param dataType specifies the data type to be integrated.
     */
    public TrcDataIntegrator(final String instanceName, TrcSensor sensor, Object dataType)
    {
        this(instanceName, sensor, dataType, false);
    }   //TrcDataProcessor

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
     * This method allows the caller to unwind the integrated data if the
     * data point of all axes are zero.
     *
     * @param unwindData specifies true to reset integrated data if all inputs are zero,
     *                   false otherwise.
     */
    public void setUnwindIntegratedData(boolean unwindData)
    {
        unwindIntegratedData = unwindData;
    }   //setUnwindIntegratedData

    /**
     * This method enables the data integrator. The data integrator is not
     * automatically enabled when created. You must explicitly call this
     * method to enable the data integrator.
     *
     * @param enabled specifies true for enabling the data processor, disabling it otherwise.
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

        if (enabled)
        {
            reset();
            TrcTaskMgr.getInstance().registerTask(
                    instanceName, this, TrcTaskMgr.TaskType.PRECONTINUOUS_TASK);
        }
        else
        {
            TrcTaskMgr.getInstance().unregisterTask(this, TrcTaskMgr.TaskType.PRECONTINUOUS_TASK);
        }
    }   //setEnabled

    /**
     * This method resets the indexed integratedData and doubleIntegratedData.
     *
     * @param index specifies the index.
     */
    public void reset(int index)
    {
        final String funcName = "reset";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        prevTimes[index] = HalUtil.getCurrentTime();
        integratedData[index].value = 0.0;
        if (doubleIntegratedData != null)
        {
            doubleIntegratedData[index].value = 0.0;
        }
    }   //reset

    /**
     * This method resets all integratorData and doubleIntegratedData.
     */
    public void reset()
    {
        final String funcName = "reset";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        for (int i = 0; i < integratedData.length; i++)
        {
            reset(i);
        }
    }   //reset

    /**
     * This method returns the last indexed input data.
     *
     * @param index specifies the index.
     * @return the last indexed input data.
     */
    public TrcSensor.SensorData getInputData(int index)
    {
        final String funcName = "getInputData";
        TrcSensor.SensorData data =
                new TrcSensor.SensorData(inputData[index].timestamp, inputData[index].value);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%f",
                               data != null? data.timestamp: 0.0,
                               data != null? data.value: 0.0);
        }

        return data;
    }   //getInputData

    /**
     * This method returns the last indexed integrated data.
     *
     * @param index specifies the index.
     * @return last indexed integrated data.
     */
    public TrcSensor.SensorData getIntegratedData(int index)
    {
        final String funcName = "getIntegratedData";
        TrcSensor.SensorData data = new TrcSensor.SensorData(
                integratedData[index].timestamp, integratedData[index].value);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%f",
                               data != null? data.timestamp: 0.0,
                               data != null? data.value: 0.0);
        }

        return data;
    }   //getIntegratedData

    /**
     * This method returns the last indexed double integrated data.
     *
     * @param index specifies the index.
     * @return last indexed double integrated data.
     */
    public TrcSensor.SensorData getDoubleIntegratedData(int index)
    {
        final String funcName = "getDoubleIntegratedData";
        TrcSensor.SensorData data = new TrcSensor.SensorData(
                    doubleIntegratedData[index].timestamp, doubleIntegratedData[index].value);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%f",
                               data != null? data.timestamp: 0.0,
                               data != null? data.value: 0.0);
        }

        return data;
    }   //getDoubleIntegratedData

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
     * This method is called periodically to do data integration.
     *
     * @param runMode specifies the competition mode that is running.
     */
    @Override
    public void preContinuousTask(TrcRobot.RunMode runMode)
    {
        final String funcName = "preContinuousTask";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.TASK,
                    "mode=%s", runMode.toString());
        }

        boolean allZeroAxis = true;
        double[] deltaTime = new double[inputData.length];
        for (int i = 0; i < inputData.length; i++)
        {
            //
            // Get sensor data.
            //
            inputData[i] = sensor.getData(i, dataType);
            deltaTime[i] = inputData[i].timestamp - prevTimes[i];
            if ((Double)inputData[i].value != 0.0)
            {
                allZeroAxis = false;
            }
            //
            // Do integration.
            //
            integratedData[i].timestamp = inputData[i].timestamp;
            integratedData[i].value = (Double)integratedData[i].value +
                                      (Double)inputData[i].value*deltaTime[i];
            prevTimes[i] = inputData[i].timestamp;
        }

        //
        // Do double integration if necessary.
        //
        if (doubleIntegratedData != null)
        {
            for (int i = 0; i < inputData.length; i++)
            {
                doubleIntegratedData[i].timestamp = inputData[i].timestamp;
                if (allZeroAxis)
                {
                    integratedData[i].value = 0.0;
                }
                else
                {
                    doubleIntegratedData[i].value =
                            (Double)doubleIntegratedData[i].value +
                            (Double)integratedData[i].value*deltaTime[i];
                }
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //preContinuousTask

    @Override
    public void postContinuousTask(TrcRobot.RunMode runMode)
    {
    }   //postContinuousTask

}   //class TrcDataIntegrator
