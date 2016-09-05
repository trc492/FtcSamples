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

public class TrcEnhancedServo implements TrcTaskMgr.Task
{
    private static final String moduleName = "TrcEnhancedServo";
    private static final boolean debugEnabled = false;
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

    private void commonInit(
            String instanceName,
            TrcServo servo1, TrcServo servo2,
            TrcDigitalInput lowerLimitSwitch, TrcDigitalInput upperLimitSwitch)
    {
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(
                    moduleName + "." + instanceName,
                    false,
                    TrcDbgTrace.TraceLevel.API,
                    TrcDbgTrace.MsgLevel.INFO);
        }

        this.instanceName = instanceName;
        this.servo1 = servo1;
        this.servo2 = servo2;
        this.lowerLimitSwitch = lowerLimitSwitch;
        this.upperLimitSwitch = upperLimitSwitch;
    }   //commonInit

    public TrcEnhancedServo(String instanceName, TrcServo servo)
    {
        if (servo == null)
        {
            throw new NullPointerException("servo cannot be null.");
        }

        commonInit(instanceName, servo, null, null, null);
    }   //TrcEnhancedServo

    public TrcEnhancedServo(String instanceName, TrcServo servo1, TrcServo servo2)
    {
        if (servo1 == null || servo2 == null)
        {
            throw new NullPointerException("servo1/servo2 cannot be null.");
        }

        commonInit(instanceName, servo1, servo2, null, null);
    }   //TrcEnhancedServo

    public TrcEnhancedServo(
            String instanceName,
            TrcServo servo, TrcDigitalInput lowerLimitSwitch, TrcDigitalInput upperLimitSwitch)
    {
        if (servo1 == null)
        {
            throw new NullPointerException("servo cannot be null.");
        }

        commonInit(instanceName, servo, null, lowerLimitSwitch, upperLimitSwitch);
        continuousServo = true;
    }   //TrcEnhancedServo

    public String toString()
    {
        return instanceName;
    }   //toString

    private void setSteppingEnabled(boolean enabled)
    {
        if (enabled && !servoStepping)
        {
            TrcTaskMgr.getInstance().registerTask(
                    "ServoSteppingTask", this, TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
            TrcTaskMgr.getInstance().registerTask(
                    "ServoSteppingTask", this, TrcTaskMgr.TaskType.STOP_TASK);
        }
        else if (!enabled && servoStepping)
        {
            TrcTaskMgr.getInstance().unregisterTask(this, TrcTaskMgr.TaskType.STOP_TASK);
            TrcTaskMgr.getInstance().unregisterTask(this, TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        }
        servoStepping = enabled;
    }   //setSteppingEnabled

    public void stop()
    {
        if (continuousServo)
        {
            servo1.setPosition(SERVO_CONTINUOUS_STOP);
        }
        else if (servoStepping)
        {
            setSteppingEnabled(false);
        }
    }   //stop

    public void setPosition(double position)
    {
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

    public void setPosition(double position, double stepRate)
    {
        if (!continuousServo)
        {
            this.targetPosition = position;
            this.currStepRate = Math.abs(stepRate);
            this.prevTime = HalUtil.getCurrentTime();
            this.currPosition = servo1.getPosition();
            setSteppingEnabled(true);
        }
    }   //setPosition

    public void setStepMode(double maxStepRate, double minPos, double maxPos)
    {
        if (!continuousServo)
        {
            this.maxStepRate = maxStepRate;
            this.minPos = minPos;
            this.maxPos = maxPos;
        }
    }   //setStepMode

    public void setPower(double power)
    {
        power = TrcUtil.limit(power, -1.0, 1.0);
        if (continuousServo)
        {
            if (lowerLimitSwitch != null &&
                lowerLimitSwitch.isActive() ||
                upperLimitSwitch != null &&
                upperLimitSwitch.isActive())
            {
                //
                // One of the limit switches is hit, so stop!
                //
                servo1.setPosition(SERVO_CONTINUOUS_STOP);
            }
            else
            {
                power = TrcUtil.scaleRange(
                        power, -1.0, 1.0,
                        SERVO_CONTINUOUS_REV_MAX, SERVO_CONTINUOUS_FWD_MAX);
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

    @Override
    public void postContinuousTask(TrcRobot.RunMode runMode)
    {
        if (runMode != TrcRobot.RunMode.DISABLED_MODE)
        {
            double currTime = HalUtil.getCurrentTime();
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
    }   //postContinuousTask

}   //class TrcEnhancedServo
