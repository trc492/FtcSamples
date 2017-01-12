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

package ftclib;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorController;

import trclib.TrcDbgTrace;
import trclib.TrcRobot;
import trclib.TrcTaskMgr;

/**
 * This class monitors the robot battery level and provides methods to get the current battery voltage as well as
 * the lowest voltage it has ever seen during the monitoring session.
 */
public class FtcRobotBattery implements TrcTaskMgr.Task
{
    private static final String moduleName = "FtcRobotBattery";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private ModernRoboticsUsbDcMotorController motorController;
    private double lowestVoltage = 0.0;
    private double highestVoltage = 0.0;

    /**
     * Constructor: create an instance of the object.
     *
     * @param motorController specifies the motor control that provides the voltage information.
     */
    public FtcRobotBattery(DcMotorController motorController)
    {
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName, tracingEnabled, traceLevel, msgLevel);
        }

        this.motorController = (ModernRoboticsUsbDcMotorController)motorController;
    }   //FtcRobotBattery

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
            lowestVoltage = highestVoltage = motorController.getVoltage();
            TrcTaskMgr.getInstance().registerTask(moduleName, this, TrcTaskMgr.TaskType.PRECONTINUOUS_TASK);
        }
        else
        {
            TrcTaskMgr.getInstance().unregisterTask(this, TrcTaskMgr.TaskType.PRECONTINUOUS_TASK);
        }
    }   //setEnabled

    /**
     * This method returns the current robot battery voltage.
     *
     * @return current battery voltage.
     */
    public double getCurrentVoltage()
    {
        return motorController.getVoltage();
    }   //getCurrentVoltage

    /**
     * This method returns the lowest voltage it has ever seen during the monitoring session.
     *
     * @return lowest battery voltage.
     */
    public double getLowestVoltage()
    {
        return lowestVoltage;
    }   //getLowestVoltage

    /**
     * This method returns the highest voltage it has ever seen during the monitoring session.
     *
     * @return highest battery voltage.
     */
    public double getHighestVoltage()
    {
        return highestVoltage;
    }   //getHighestVoltage

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

        double voltage = getCurrentVoltage();
        if (voltage < lowestVoltage)
        {
            lowestVoltage = voltage;
        }
        else if (voltage > highestVoltage)
        {
            highestVoltage = voltage;
        }
    }   //preContinuousTask

    @Override
    public void postContinuousTask(TrcRobot.RunMode runMode)
    {
    }   //postContinuousTask

}   //class FtcRobotBattery
