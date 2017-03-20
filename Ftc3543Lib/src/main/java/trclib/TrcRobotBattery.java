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
public abstract class TrcRobotBattery implements TrcTaskMgr.Task
{
    private static final String moduleName = "TrcRobotBattery";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

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

    private double lowestVoltage = 0.0;
    private double highestVoltage = 0.0;
    private boolean voltageSupported = true;
    private double lowestCurrent = 0.0;
    private double highestCurrent = 0.0;
    private boolean currentSupported = true;
    private double lowestPower = 0.0;
    private double highestPower = 0.0;
    private boolean powerSupported = true;

    /**
     * Constructor: create an instance of the object.
     */
    public TrcRobotBattery()
    {
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName, tracingEnabled, traceLevel, msgLevel);
        }
    }   //TrcRobotBattery

    /**
     * This method enables/disables the battery monitoring task. When the task is enabled, it also clears the
     * lowest voltage.
     *
     * @param enabled specifies true to enable the task, false to disable.
     */
    public void setEnabled(boolean enabled)
    {
        final String funcName = "setEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC, "enabled=%s", Boolean.toString(enabled));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC);
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

            TrcTaskMgr.getInstance().registerTask(moduleName, this, TrcTaskMgr.TaskType.PRECONTINUOUS_TASK);
        }
        else
        {
            TrcTaskMgr.getInstance().unregisterTask(this, TrcTaskMgr.TaskType.PRECONTINUOUS_TASK);
        }
    }   //setEnabled

    /**
     * This method returns the lowest voltage it has ever seen during the monitoring session.
     *
     * @return lowest battery voltage.
     * @throws UnsupportedOperationException if voltage is not supported by the system. 
     */
    public double getLowestVoltage()
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
    public double getHighestVoltage()
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
    public double getLowestCurrent()
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
    public double getHighestCurrent()
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
    public double getLowestPower()
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
    public double getHighestPower()
    {
        if (!powerSupported)
        {
            throw new UnsupportedOperationException("This system does not support power info.");
        }

        return highestPower;
    }   //getHighestPower

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
     * This method is called periodically to monitor the battery voltage and to keep track of the lowest voltage it
     * has ever seen.
     *
     * @param runMode specifies the competition mode that is running.
     */
    @Override
    public void preContinuousTask(TrcRobot.RunMode runMode)
    {
        final String funcName = "preContinuousTask";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "runMode=%s", runMode.toString());
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }

        if (voltageSupported)
        {
            double voltage = getVoltage();
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
            double current = getCurrent();
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
            double power = getPower();
            if (power < lowestPower)
            {
                lowestPower = power;
            }
            else if (power > highestPower)
            {
                highestPower = power;
            }
        }
    }   //preContinuousTask

    @Override
    public void postContinuousTask(TrcRobot.RunMode runMode)
    {
    }   //postContinuousTask

}   //class TrcRobotBattery
