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

import android.os.Trace;

/**
 * This class implements a platform independent Enhanced servo. An enhanced servo is a servo with enhanced features.
 * The enhanced servo supports both normal servo as well as continuous servo. It supports limit switches for the
 * continuous servo just like TrcMotor. It simulates a speed controlled motor with a regular servo. It does this
 * by stepping the servo with different step rate to make it seemed to be speed controlled. It also supports doubling
 * two servos to handle bigger load as a single servo, equivalent to connecting two servos with a Y splitter cable
 * except doing it with software instead.
 */
public class TrcEnhancedServo implements TrcTaskMgr.Task
{
    private static final String moduleName = "TrcEnhancedServo";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private static final double SERVO_CONTINUOUS_STOP = 0.5;
    private static final double SERVO_CONTINUOUS_FWD_MAX = 1.0;
    private static final double SERVO_CONTINUOUS_REV_MAX = 0.0;

    private String instanceName;
    private TrcServo servo1 = null;
    private TrcServo servo2 = null;
    private boolean continuousServo = false;
    private boolean servoStepping = false;
    private double targetPosition = 0.0;
    private double currStepRate = 0.0;
    private double prevTime = 0.0;
    private double currPosition = 0.0;
    private double maxStepRate = 0.0;
    private double minPos = 0.0;
    private double maxPos = 1.0;
    //
    // The following is for continuous servo.
    //
    private TrcDigitalInput lowerLimitSwitch = null;
    private TrcDigitalInput upperLimitSwitch = null;

    /**
     * This method is called by different constructors to do common initialization.
     *
     * @param instanceName specifies the instance name.
     * @param servo1 specifies the first physical servo object.
     * @param servo2 specifies the second physical servo object.
     * @param lowerLimitSwitch specifies the lower limit switch object.
     * @param upperLimitSwitch specifies the high limit switch object.
     */
    private void commonInit(
            String instanceName,
            TrcServo servo1, TrcServo servo2,
            TrcDigitalInput lowerLimitSwitch, TrcDigitalInput upperLimitSwitch)
    {
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
        this.servo1 = servo1;
        this.servo2 = servo2;
        this.lowerLimitSwitch = lowerLimitSwitch;
        this.upperLimitSwitch = upperLimitSwitch;
    }   //commonInit

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param servo specifies the physical servo object.
     */
    public TrcEnhancedServo(String instanceName, TrcServo servo)
    {
        if (servo == null)
        {
            throw new NullPointerException("servo cannot be null.");
        }

        commonInit(instanceName, servo, null, null, null);
    }   //TrcEnhancedServo

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param servo1 specifies the first physical servo object.
     * @param servo2 specifies the second physical servo object.
     */
    public TrcEnhancedServo(String instanceName, TrcServo servo1, TrcServo servo2)
    {
        if (servo1 == null || servo2 == null)
        {
            throw new NullPointerException("servo1/servo2 cannot be null.");
        }

        commonInit(instanceName, servo1, servo2, null, null);
    }   //TrcEnhancedServo

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param servo specifies the physical servo object.
     * @param lowerLimitSwitch specifies the lower limit switch object.
     * @param upperLimitSwitch specifies the high limit switch object.
     */
    public TrcEnhancedServo(
            String instanceName, TrcServo servo, TrcDigitalInput lowerLimitSwitch, TrcDigitalInput upperLimitSwitch)
    {
        if (servo1 == null)
        {
            throw new NullPointerException("servo cannot be null.");
        }

        commonInit(instanceName, servo, null, lowerLimitSwitch, upperLimitSwitch);
        continuousServo = true;
    }   //TrcEnhancedServo

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
     * This method enables/disables the stepping feature of the regular servo making it look like a speed controlled
     * motor.
     *
     * @param enabled specifies true to enable stepping, false to disable.
     */
    private void setSteppingEnabled(boolean enabled)
    {
        final String funcName = "setSteppingEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enabled=%s", Boolean.toString(enabled));
        }

        if (enabled && !servoStepping)
        {
            TrcTaskMgr.getInstance().registerTask("ServoSteppingTask", this, TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
            TrcTaskMgr.getInstance().registerTask("ServoSteppingTask", this, TrcTaskMgr.TaskType.STOP_TASK);
        }
        else if (!enabled && servoStepping)
        {
            TrcTaskMgr.getInstance().unregisterTask(this, TrcTaskMgr.TaskType.STOP_TASK);
            TrcTaskMgr.getInstance().unregisterTask(this, TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        }
        servoStepping = enabled;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setSteppingEnabled

    /**
     * This method stops the servo.
     */
    public void stop()
    {
        final String funcName = "stop";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (continuousServo)
        {
            servo1.setPosition(SERVO_CONTINUOUS_STOP);
        }
        else if (servoStepping)
        {
            setSteppingEnabled(false);
        }
    }   //stop

    /**
     * This method sets the servo position.
     *
     * @param position specifies the position to set.
     */
    public void setPosition(double position)
    {
        final String funcName = "setPosition";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "position=%f", position);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (!continuousServo)
        {
            if (servo1 != null)
            {
                servo1.setPosition(position);
            }

            if (servo2 != null)
            {
                servo2.setPosition(position);
            }
        }
    }   //setPosition

    /**
     * This method sets the servo to the specifies position but with the specified steprate in effect controlling
     * the speed to get there.
     *
     * @param position specifies the target position.
     * @param stepRate specifies the stepping rate to get there (degrees/sec).
     */
    public void setPosition(double position, double stepRate)
    {
        final String funcName = "setPosition";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "position=%f,stepRate=%f", position, stepRate);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (!continuousServo)
        {
            this.targetPosition = position;
            this.currStepRate = Math.abs(stepRate);
            this.prevTime = TrcUtil.getCurrentTime();
            this.currPosition = servo1.getPosition();
            setSteppingEnabled(true);
        }
    }   //setPosition

    /**
     * This method sets the stepping mode characteristics.
     *
     * @param maxStepRate specifies the maximum stepping rate.
     * @param minPos specifies the minimum position.
     * @param maxPos specifies the maximum position.
     */
    public void setStepMode(double maxStepRate, double minPos, double maxPos)
    {
        final String funcName = "setStepMode";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                                "maxStepRate=%f,minPos=%f,maxPos=%f", maxStepRate, minPos, maxPos);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (!continuousServo)
        {
            this.maxStepRate = maxStepRate;
            this.minPos = minPos;
            this.maxPos = maxPos;
        }
    }   //setStepMode

    /**
     * This method sets the power just like a regular motor but for a servo. If it is a continuous servo, it will
     * set it running with different speed. If it is a regular servo, it will change its step rate.
     *
     * @param power specifies how fast the servo will turn.
     */
    public void setPower(double power)
    {
        final String funcName = "setPower";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "power=%f", power);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        power = TrcUtil.clipRange(power, -1.0, 1.0);
        if (continuousServo)
        {
            if (lowerLimitSwitch != null && lowerLimitSwitch.isActive() ||
                upperLimitSwitch != null && upperLimitSwitch.isActive())
            {
                //
                // One of the limit switches is hit, so stop!
                //
                servo1.setPosition(SERVO_CONTINUOUS_STOP);
            }
            else
            {
                power = TrcUtil.scaleRange(power, -1.0, 1.0, SERVO_CONTINUOUS_REV_MAX, SERVO_CONTINUOUS_FWD_MAX);
                servo1.setPosition(power);
            }
        }
        else if (!servoStepping)
        {
            setPosition(power > 0.0? maxPos: minPos, Math.abs(power)*maxStepRate);
        }
        else if (power != 0.0)
        {
            targetPosition = power > 0.0? maxPos: minPos;
            currStepRate = Math.abs(power)*maxStepRate;
        }
        else
        {
            setSteppingEnabled(false);
        }
    }   //setPower

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
        stop();
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
    }   //preContinuousTask

    /**
     * This method is called periodically to check whether the servo has reached target. If not, it will calculate
     * the next position to set the servo to according to its step rate.
     *
     * @param runMode specifies the competition mode that is running. (e.g. Autonomous, TeleOp, Test).
     */
    @Override
    public void postContinuousTask(TrcRobot.RunMode runMode)
    {
        final String funcName = "postContinuousTask";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "runMode=%s", runMode.toString());
        }

        if (runMode != TrcRobot.RunMode.DISABLED_MODE)
        {
            double currTime = TrcUtil.getCurrentTime();
            double deltaPos = currStepRate * (currTime - prevTime);

            if (currPosition < targetPosition)
            {
                currPosition += deltaPos;
                if (currPosition > targetPosition)
                {
                    currPosition = targetPosition;
                }
            }
            else if (currPosition > targetPosition)
            {
                currPosition -= deltaPos;
                if (currPosition < targetPosition)
                {
                    currPosition = targetPosition;
                }
            }
            else
            {
                //
                // We have reached target.
                //
                stop();
            }
            prevTime = currTime;

            if (servo1 != null)
            {
                servo1.setPosition(currPosition);
            }

            if (servo2 != null)
            {
                servo2.setPosition(currPosition);
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //postContinuousTask

}   //class TrcEnhancedServo
