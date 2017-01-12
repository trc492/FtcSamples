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
 * This class implements a platform independent motor controller. Typically, this class is extended by a platform
 * dependent motor controller class. The platform dependent motor controller class must implement the abstract
 * methods required by this class. The abstract methods allow this class to access the physical motor controller
 * independent of the platform. Not all motor controllers are created equal. Some have more features than the other.
 * This class attempts to implement some of the features in software. If the motor controller supports these features
 * in hardware, the subclass should override these methods and access the hardware instead.
 */
public abstract class TrcMotor implements TrcTaskMgr.Task
{
    private static final String moduleName = "TrcMotor";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    /**
     * This method returns the state of the motor controller direction.
     *
     * @return true if the motor direction is inverted, false otherwise.
     */
    public abstract boolean getInverted();

    /**
     * This method returns the motor position by reading the position sensor. The position sensor can be an encoder
     * or a potentiometer.
     *
     * @return current motor position.
     */
    public abstract double getPosition();

    /**
     * This method gets the last set power.
     *
     * @return the last setPower value.
     */
    public abstract double getPower();

    /**
     * This method returns the state of the lower limit switch.
     *
     * @return true if lower limit switch is active, false otherwise.
     */
    public abstract boolean isLowerLimitSwitchActive();

    /**
     * This method returns the state of the upper limit switch.
     *
     * @return true if upper limit switch is active, false otherwise.
     */
    public abstract boolean isUpperLimitSwitchActive();

    /**
     * This method resets the motor position sensor, typically an encoder.
     */
    public abstract void resetPosition();

    /**
     * This method enables/disables motor brake mode. In motor brake mode, set power to 0 would stop the motor very
     * abruptly by shorting the motor wires together using the generated back EMF to stop the motor. When brakMode
     * is false (i.e. float/coast mode), the motor wires are just disconnected from the motor controller so the motor
     * will stop gradually.
     *
     * @param enabled specifies true to enable brake mode, false otherwise.
     */
    public abstract void setBrakeModeEnabled(boolean enabled);

    /**
     * This method inverts the motor direction.
     *
     * @param inverted specifies true to invert motor direction, false otherwise.
     */
    public abstract void setInverted(boolean inverted);

    /**
     * This method sets the output power of the motor controller.
     *
     * @param power specifies the output power for the motor controller in the range of -1.0 to 1.0.
     */
    public abstract void setPower(double power);

    private final String instanceName;
    private boolean speedTaskEnabled = false;
    private double speed = 0.0;
    private double prevTime = 0.0;
    private double prevPos = 0.0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public TrcMotor(final String instanceName)
    {
        this.instanceName = instanceName;

        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }
    }   //TrcMotor

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
     * This method returns the speed of the motor rotation. It keeps track of the rotation speed by using a periodic
     * task to monitor the position sensor value. If the motor controller has hardware monitoring speed, it should
     * override this method and access the hardware instead.
     *
     * @throws UnsupportedOperationException exception.
     */
    public double getSpeed()
    {
        final String funcName = "getSpeed";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%.3f", speed);
        }

        if (speedTaskEnabled)
        {
            return speed;
        }
        else
        {
            throw new UnsupportedOperationException("SpeedTask is not enabled.");
        }
    }   //getSpeed

    /**
     * This method enables/disables the task that monitors the motor speed. To determine the motor speed, the task
     * runs periodically and determines the delta encoder reading over delta time to calculate the speed. Since the
     * task takes up CPU cycle, it should not be enabled if the user doesn't need motor speed info.
     *
     * @param enabled specifies true to enable speed monitor task, disable otherwise.
     */
    public void setSpeedTaskEnabled(boolean enabled)
    {
        final String funcName = "setSpeedTaskEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enabled=%s", Boolean.toString(enabled));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        speedTaskEnabled = enabled;
        TrcTaskMgr taskMgr = TrcTaskMgr.getInstance();
        if (enabled)
        {
            prevTime = TrcUtil.getCurrentTime();
            prevPos = getPosition();
            taskMgr.registerTask(instanceName, this, TrcTaskMgr.TaskType.STOP_TASK);
            taskMgr.registerTask(instanceName, this, TrcTaskMgr.TaskType.PRECONTINUOUS_TASK);
        }
        else
        {
            taskMgr.unregisterTask(this, TrcTaskMgr.TaskType.STOP_TASK);
            taskMgr.unregisterTask(this, TrcTaskMgr.TaskType.PRECONTINUOUS_TASK);
        }
    }   //setSpeedTaskEnabled

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
        final String funcName = "stopTask";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "mode=%s", runMode.toString());
        }

        setSpeedTaskEnabled(false);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
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
     * This task is run periodically to calculate he speed of the motor.
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

        double currTime = TrcUtil.getCurrentTime();
        double currPos = getPosition();
        speed = (currPos - prevPos)/(currTime - prevTime);
        prevTime = currTime;
        prevPos = currPos;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //preContinuousTask

    @Override
    public void postContinuousTask(TrcRobot.RunMode runMode)
    {
    }   //postContinuousTask

}   //class TrcMotor
