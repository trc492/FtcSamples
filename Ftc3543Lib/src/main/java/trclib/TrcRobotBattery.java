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
 * This class monitors the robot battery level and provides methods to get the current battery voltage as well as
 * the lowest voltage it has ever seen during the monitoring session.
 */
public abstract class TrcRobotBattery
{
    protected static final String moduleName = "TrcRobotBattery";
    protected static final boolean debugEnabled = false;
    protected static final boolean tracingEnabled = false;
    protected static final boolean useGlobalTracer = false;
    protected static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    protected static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    protected TrcDbgTrace dbgTrace = null;

    /**
     * This method returns the robot battery voltage.
     *
     * @return robot battery voltage in volts.
     */
    public abstract double getVoltage();

    /**
     * This method returns the robot battery current.
     *
     * @return robot battery current in amps.
     */
    public abstract double getCurrent();

    /**
     * This method returns the robot battery power.
     *
     * @return robot battery power in watts.
     */
    public abstract double getPower();

    private boolean voltageSupported;
    private boolean currentSupported;
    private boolean powerSupported;
    private final TrcTaskMgr.TaskObject robotBatteryTaskObj;
    private double lowestVoltage = 0.0;
    private double highestVoltage = 0.0;
    private double lowestCurrent = 0.0;
    private double highestCurrent = 0.0;
    private double lowestPower = 0.0;
    private double highestPower = 0.0;
    private double totalEnergy = 0.0;
    private double lastTimestamp = 0.0;

    /**
     * Constructor: create an instance of the object.
     *
     * @param voltageSupported specifies true if getVoltage is supported, false otherwise.
     * @param currentSupported specifies true if getCurrent is supported, false otherwise.
     * @param powerSupported specifies true if getPower is supported, false otherwise.
     */
    public TrcRobotBattery(
            boolean voltageSupported, boolean currentSupported, boolean powerSupported)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName, tracingEnabled, traceLevel, msgLevel);
        }

        this.voltageSupported = voltageSupported;
        this.currentSupported = currentSupported;
        this.powerSupported = powerSupported;

        robotBatteryTaskObj = TrcTaskMgr.getInstance().createTask(
            moduleName + ".robotBatteryTask", this::robotBatteryTask);
    }   //TrcRobotBattery

    /**
     * This method enables/disables the battery monitoring task. When the task is enabled, it also clears the
     * lowest voltage.
     *
     * @param enabled specifies true to enable the task, false to disable.
     */
    public synchronized void setEnabled(boolean enabled)
    {
        final String funcName = "setEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enabled=%b", enabled);
        }

        if (enabled)
        {
            if (voltageSupported)
            {
                try
                {
                    lowestVoltage = highestVoltage = getVoltage();
                }
                catch (UnsupportedOperationException e)
                {
                    voltageSupported = false;
                }
            }

            if (currentSupported)
            {
                try
                {
                    lowestCurrent = highestCurrent = getCurrent();
                }
                catch (UnsupportedOperationException e)
                {
                    currentSupported = false;
                }
            }

            if (powerSupported)
            {
                try
                {
                    lowestPower = highestPower = getPower();
                }
                catch (UnsupportedOperationException e)
                {
                    powerSupported = false;
                }
            }

            totalEnergy = 0.0;
            lastTimestamp = TrcUtil.getCurrentTime();
            robotBatteryTaskObj.registerTask(TrcTaskMgr.TaskType.PRECONTINUOUS_TASK);   //TODO: should use INPUT_TASK
        }
        else
        {
            robotBatteryTaskObj.unregisterTask(TrcTaskMgr.TaskType.PRECONTINUOUS_TASK);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setEnabled

    /**
     * This method returns the lowest voltage it has ever seen during the monitoring session.
     *
     * @return lowest battery voltage.
     * @throws UnsupportedOperationException if voltage is not supported by the system. 
     */
    public synchronized double getLowestVoltage()
    {
        if (!voltageSupported)
        {
            throw new UnsupportedOperationException("This system does not support voltage info.");
        }

        return lowestVoltage;
    }   //getLowestVoltage

    /**
     * This method returns the highest voltage it has ever seen during the monitoring session.
     *
     * @return highest battery voltage.
     * @throws UnsupportedOperationException if voltage is not supported by the system. 
     */
    public synchronized double getHighestVoltage()
    {
        if (!voltageSupported)
        {
            throw new UnsupportedOperationException("This system does not support voltage info.");
        }

        return highestVoltage;
    }   //getHighestVoltage

    /**
     * This method returns the lowest current it has ever seen during the monitoring session.
     *
     * @return lowest battery current.
     * @throws UnsupportedOperationException if current is not supported by the system. 
     */
    public synchronized double getLowestCurrent()
    {
        if (!currentSupported)
        {
            throw new UnsupportedOperationException("This system does not support current info.");
        }

        return lowestCurrent;
    }   //getLowestCurrent

    /**
     * This method returns the highest current it has ever seen during the monitoring session.
     *
     * @return highest battery current.
     * @throws UnsupportedOperationException if current is not supported by the system. 
     */
    public synchronized double getHighestCurrent()
    {
        if (!currentSupported)
        {
            throw new UnsupportedOperationException("This system does not support current info.");
        }

        return highestCurrent;
    }   //getHighestCurrent

    /**
     * This method returns the lowest power it has ever seen during the monitoring session.
     *
     * @return lowest battery power.
     * @throws UnsupportedOperationException if power is not supported by the system. 
     */
    public synchronized double getLowestPower()
    {
        if (!powerSupported)
        {
            throw new UnsupportedOperationException("This system does not support power info.");
        }

        return lowestPower;
    }   //getLowestPower

    /**
     * This method returns the highest power it has ever seen during the monitoring session.
     *
     * @return highest battery power.
     * @throws UnsupportedOperationException if power is not supported by the system. 
     */
    public synchronized double getHighestPower()
    {
        if (!powerSupported)
        {
            throw new UnsupportedOperationException("This system does not support power info.");
        }

        return highestPower;
    }   //getHighestPower

    /**
     * This method returns the total energy consumed since the task was enabled.
     *
     * @return total energy consumed in WH (Watt-Hour).
     */
    public synchronized double getTotalEnergy()
    {
        return totalEnergy;
    }   //getTotalEnergy

    /**
     * This method is called periodically to monitor the battery voltage and to keep track of the lowest voltage it
     * has ever seen.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is running.
     */
    private synchronized void robotBatteryTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = "robotBatteryTask";
        double currTime = TrcUtil.getCurrentTime();
        double voltage = 0.0, current = 0.0, power;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "taskType=%s,runMode=%s", taskType, runMode);
        }

        if (voltageSupported)
        {
            voltage = getVoltage();
            if (voltage < lowestVoltage)
            {
                lowestVoltage = voltage;
            }
            else if (voltage > highestVoltage)
            {
                highestVoltage = voltage;
            }
        }

        if (currentSupported)
        {
            current = getCurrent();
            if (current < lowestCurrent)
            {
                lowestCurrent = current;
            }
            else if (current > highestCurrent)
            {
                highestCurrent = current;
            }
        }

        if (powerSupported)
        {
            power = getPower();
            if (power < lowestPower)
            {
                lowestPower = power;
            }
            else if (power > highestPower)
            {
                highestPower = power;
            }
            totalEnergy += power*(currTime - lastTimestamp)/3600.0;
        }
        else if (voltageSupported && currentSupported)
        {
            totalEnergy += voltage*current*(currTime - lastTimestamp)/3600.0;
        }

        lastTimestamp = currTime;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //robotBatteryTask

}   //class TrcRobotBattery
