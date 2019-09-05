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

import trclib.TrcRobot.RunMode;
import trclib.TrcTaskMgr.TaskType;

/**
 * This class implements a platform independent Enhanced servo. An enhanced servo is a servo with enhanced features.
 * The enhanced servo supports both normal servo as well as continuous servo. It supports limit switches for the
 * continuous servo just like TrcMotor. It also supports lower and upper limit switches for range calibrating a
 * regular servo. It simulates a speed controlled motor with a regular servo. It does this by stepping the servo
 * with different step rate to make it seemed to be speed controlled. It also supports doubling two servos to handle
 * bigger load as a single servo, equivalent to connecting two servos with a Y splitter cable except doing it with
 * software instead.
 */
public class TrcEnhancedServo
{
    private static final String moduleName = "TrcEnhancedServo";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private static final double SERVO_CONTINUOUS_STOP = 0.5;
    private static final double SERVO_CONTINUOUS_FWD_MAX = 1.0;
    private static final double SERVO_CONTINUOUS_REV_MAX = 0.0;

    private final String instanceName;
    private final boolean continuousServo;
    private final TrcServo servo1;
    private final TrcServo servo2;
    private final TrcDigitalInput lowerLimitSwitch;
    private final TrcDigitalInput upperLimitSwitch;
    private TrcTaskMgr.TaskObject steppingServoTaskObj;
    private TrcTaskMgr.TaskObject stopServoTaskObj;
    private boolean taskEnabled = false;
    private boolean calibrating = false;
    private double physicalRangeMax = 1.0;
    private double logicalRangeLow = -1.0;
    private double logicalRangeHigh = -1.0;
    private double targetPosition = 0.0;
    private double currStepRate = 0.0;
    private double prevTime = 0.0;
    private double currPosition = 0.0;
    private double currPower = 0.0;
    private double maxStepRate = 0.0;
    private double minPos = 0.0;
    private double maxPos = 1.0;

    /**
     * Constructor: Creates an instance of the object.
     * This method is called by different constructors to do common initialization.
     *
     * @param instanceName specifies the instance name.
     * @param continuousServo specifies true if servos are continuous servo.
     * @param servo1 specifies the first physical servo object.
     * @param servo2 specifies the second physical servo object.
     * @param lowerLimitSwitch specifies the lower limit switch object.
     * @param upperLimitSwitch specifies the high limit switch object.
     */
    public TrcEnhancedServo(
        String instanceName, boolean continuousServo, TrcServo servo1, TrcServo servo2,
        TrcDigitalInput lowerLimitSwitch, TrcDigitalInput upperLimitSwitch)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
        this.continuousServo = continuousServo;
        this.servo1 = servo1;
        this.servo2 = servo2;
        this.lowerLimitSwitch = lowerLimitSwitch;
        this.upperLimitSwitch = upperLimitSwitch;

        TrcTaskMgr taskMgr = TrcTaskMgr.getInstance();
        steppingServoTaskObj = taskMgr.createTask(instanceName + ".enhancedServoTask", this::steppingServoTask);
        stopServoTaskObj = taskMgr.createTask(instanceName + ".stopServoTask", this::stopServoTask);
    }   //TrcEnhancedServo

    /**
     * Constructor: Creates an instance of the object.
     * This method is called by different constructors to do common initialization.
     *
     * @param instanceName specifies the instance name.
     * @param continuousServo specifies true if servos are continuous servo.
     * @param servo specifies the physical servo object.
     * @param lowerLimitSwitch specifies the lower limit switch object.
     * @param upperLimitSwitch specifies the high limit switch object.
     */
    public TrcEnhancedServo(
        String instanceName, boolean continuousServo, TrcServo servo,
        TrcDigitalInput lowerLimitSwitch, TrcDigitalInput upperLimitSwitch)
    {
        this(instanceName, continuousServo, servo, null, lowerLimitSwitch, upperLimitSwitch);
    }   //TrcEnhancedServo

    /**
     * Constructor: Creates an instance of the object.
     * This method is called by different constructors to do common initialization.
     *
     * @param instanceName specifies the instance name.
     * @param continuousServo specifies true if servos are continuous servo.
     * @param servo specifies the physical servo object.
     * @param lowerLimitSwitch specifies the lower limit switch object.
     */
    public TrcEnhancedServo(
        String instanceName, boolean continuousServo, TrcServo servo, TrcDigitalInput lowerLimitSwitch)
    {
        this(instanceName, continuousServo, servo, null, lowerLimitSwitch, null);
    }   //TrcEnhancedServo

    /**
     * Constructor: Creates an instance of the object.
     * This method is called by different constructors to do common initialization.
     *
     * @param instanceName specifies the instance name.
     * @param servo1 specifies the first physical servo object.
     * @param servo2 specifies the second physical servo object.
     * @param lowerLimitSwitch specifies the lower limit switch object.
     * @param upperLimitSwitch specifies the high limit switch object.
     */
    public TrcEnhancedServo(
        String instanceName, TrcServo servo1, TrcServo servo2,
        TrcDigitalInput lowerLimitSwitch, TrcDigitalInput upperLimitSwitch)
    {
        this(instanceName, false, servo1, servo2, lowerLimitSwitch, upperLimitSwitch);
    }   //TrcEnhancedServo

    /**
     * Constructor: Creates an instance of the object.
     * This method is called by different constructors to do common initialization.
     *
     * @param instanceName specifies the instance name.
     * @param servo1 specifies the first physical servo object.
     * @param servo2 specifies the second physical servo object.
     * @param lowerLimitSwitch specifies the lower limit switch object.
     */
    public TrcEnhancedServo(
        String instanceName, TrcServo servo1, TrcServo servo2, TrcDigitalInput lowerLimitSwitch)
    {
        this(instanceName, false, servo1, servo2, lowerLimitSwitch, null);
    }   //TrcEnhancedServo

    /**
     * Constructor: Creates an instance of the object.
     * This method is called by different constructors to do common initialization.
     *
     * @param instanceName specifies the instance name.
     * @param servo1 specifies the first physical servo object.
     * @param servo2 specifies the second physical servo object.
     */
    public TrcEnhancedServo(
        String instanceName, TrcServo servo1, TrcServo servo2)
    {
        this(instanceName, false, servo1, servo2, null, null);
    }   //TrcEnhancedServo

    /**
     * Constructor: Creates an instance of the object.
     * This method is called by different constructors to do common initialization.
     *
     * @param instanceName specifies the instance name.
     * @param servo specifies the physical servo object.
     * @param lowerLimitSwitch specifies the lower limit switch object.
     * @param upperLimitSwitch specifies the high limit switch object.
     */
    public TrcEnhancedServo(
        String instanceName, TrcServo servo, TrcDigitalInput lowerLimitSwitch, TrcDigitalInput upperLimitSwitch)
    {
        this(instanceName, false, servo, null, lowerLimitSwitch, upperLimitSwitch);
    }   //TrcEnhancedServo

    /**
     * Constructor: Creates an instance of the object.
     * This method is called by different constructors to do common initialization.
     *
     * @param instanceName specifies the instance name.
     * @param servo specifies the physical servo object.
     * @param lowerLimitSwitch specifies the lower limit switch object.
     */
    public TrcEnhancedServo(
        String instanceName, TrcServo servo, TrcDigitalInput lowerLimitSwitch)
    {
        this(instanceName, false, servo, null, lowerLimitSwitch, null);
    }   //TrcEnhancedServo

    /**
     * Constructor: Creates an instance of the object.
     * This method is called by different constructors to do common initialization.
     *
     * @param instanceName specifies the instance name.
     * @param servo specifies the physical servo object.
     */
    public TrcEnhancedServo(String instanceName, TrcServo servo)
    {
        this(instanceName, false, servo, null, null, null);
    }   //TrcEnhancedServo

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @Override
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method enables/disables the enhanced servo task for performing step rate speed control or zero
     * calibration.
     *
     * @param enabled specifies true to enable task, false to disable.
     */
    private synchronized void setTaskEnabled(boolean enabled)
    {
        final String funcName = "setTaskEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC, "enabled=%b", enabled);
        }

        if (enabled)
        {
            steppingServoTaskObj.registerTask(TaskType.POSTCONTINUOUS_TASK);    //TODO: should use OUTPUT_TASK
            stopServoTaskObj.registerTask(TrcTaskMgr.TaskType.STOP_TASK);
        }
        else
        {
            steppingServoTaskObj.unregisterTask(TaskType.POSTCONTINUOUS_TASK);
            stopServoTaskObj.unregisterTask(TrcTaskMgr.TaskType.STOP_TASK);
            calibrating = false;
        }
        taskEnabled = enabled;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC);
        }
    }   //setTaskEnabled

    /**
     * This method checks if the task is enabled.
     *
     * @return true if task is enabled, false otherwise.
     */
    private synchronized boolean isTaskEnabled()
    {
        final String funcName = "isTaskEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC, "=%s", taskEnabled);
        }

        return taskEnabled;
    }   //isTaskEnabled

    /**
     * This method performs range calibration on a regular servo.
     *
     * @param physicalRangeMin specifies the desired physical range minimum (typically 0.0).
     * @param physicalRangeMax specifies the desired physical range maximum (typically 180.0).
     * @param stepRate specifies the step rate to use for the calibration.
     */
    public void rangeCalibrate(double physicalRangeMin, double physicalRangeMax, double stepRate)
    {
        final String funcName = "rangeCalibrate";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "stepRate=%f", stepRate);
        }

        if (!continuousServo && lowerLimitSwitch != null & upperLimitSwitch != null)
        {
            this.physicalRangeMax = physicalRangeMax;
            servo1.setPhysicalRange(physicalRangeMin, physicalRangeMax);
            servo1.setLogicalRange(0.0, 1.0);
            if (servo2 != null)
            {
                servo2.setPhysicalRange(physicalRangeMin, physicalRangeMax);
                servo2.setLogicalRange(0.0, 1.0);
            }

            logicalRangeLow = logicalRangeHigh = -1.0;
            setPosition(physicalRangeMin, stepRate);
            calibrating = true;
       }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //rangeCalibrate

    /**
     * This method stops the servo.
     */
    public void stop()
    {
        final String funcName = "stop";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (continuousServo)
        {
            servo1.setPosition(SERVO_CONTINUOUS_STOP);
        }
        else
        {
            setTaskEnabled(false);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //stop

    /**
     * This method returns the target position set by setPosition.
     *
     * @return target position.
     */
    public double getPosition()
    {
        final String funcName = "getPosition";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", targetPosition);
        }

        return targetPosition;
    }   //getPosition

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
        }

        if (!continuousServo)
        {
            targetPosition = position;

            if (servo1 != null)
            {
                servo1.setPosition(position);
            }

            if (servo2 != null)
            {
                servo2.setPosition(position);
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setPosition

    /**
     * This method sets the servo to the specifies position but with the specified step rate in effect controlling
     * the speed to get there.
     *
     * @param position specifies the target position.
     * @param stepRate specifies the stepping rate to get there (degrees/sec).
     */
    public synchronized void setPosition(double position, double stepRate)
    {
        final String funcName = "setPosition";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "position=%f,stepRate=%f", position, stepRate);
        }

        if (!continuousServo)
        {
            this.targetPosition = position;
            this.currStepRate = Math.abs(stepRate);
            this.prevTime = TrcUtil.getCurrentTime();
            this.currPosition = servo1.getPosition();
            setTaskEnabled(true);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setPosition

    /**
     * This method sets the stepping mode characteristics.
     *
     * @param maxStepRate specifies the maximum stepping rate.
     * @param minPos specifies the minimum position.
     * @param maxPos specifies the maximum position.
     */
    public synchronized void setStepMode(double maxStepRate, double minPos, double maxPos)
    {
        final String funcName = "setStepMode";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                                "maxStepRate=%f,minPos=%f,maxPos=%f", maxStepRate, minPos, maxPos);
        }

        if (!continuousServo)
        {
            this.maxStepRate = maxStepRate;
            this.minPos = minPos;
            this.maxPos = maxPos;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
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
        }

        power = TrcUtil.clipRange(power, -1.0, 1.0);
        if (continuousServo)
        {
            if (lowerLimitSwitch != null && lowerLimitSwitch.isActive() && power < 0.0 ||
                upperLimitSwitch != null && upperLimitSwitch.isActive() && power > 0.0)
            {
                //
                // One of the limit switches is hit, so stop!
                //
                servo1.setPosition(SERVO_CONTINUOUS_STOP);
            }
            else
            {
                power = TrcUtil.scaleRange(
                        power, -1.0, 1.0, SERVO_CONTINUOUS_REV_MAX, SERVO_CONTINUOUS_FWD_MAX);
                servo1.setPosition(power);
            }
        }
        else if (!isTaskEnabled() || calibrating)
        {
            //
            // Not in stepping mode, so start stepping mode.
            // If we are calibrating, cancel calibration.
            //
            calibrating = false;
            setPosition(power > 0.0? maxPos: minPos, Math.abs(power)*maxStepRate);
        }
        else if (power != 0.0)
        {
            //
            // We are already in stepping mode, just change the stepping parameters.
            //
            targetPosition = power > 0.0? maxPos: minPos;
            currStepRate = Math.abs(power)*maxStepRate;
        }
        else
        {
            //
            // We are stopping.
            //
            stop();
        }
        currPower = power;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setPower

    /**
     * This method returns the last set power value.
     *
     * @return last power set to the motor.
     */
    public double getPower()
    {
        final String funcName = "getPower";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", currPower);
        }

        return currPower;
    }   //getPower

    /**
     * This method is called periodically to check whether the servo has reached target. If not, it will calculate
     * the next position to set the servo to according to its step rate.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is running. (e.g. Autonomous, TeleOp, Test).
     */
    private synchronized void steppingServoTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = "steppingServoTask";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "taskType=%s,runMode=%s", taskType, runMode);
        }

        if (runMode != RunMode.DISABLED_MODE)
        {
            if (calibrating)
            {
                if (logicalRangeLow == -1.0 && lowerLimitSwitch.isActive())
                {
                    //
                    // We finished calibrating the low range, now start calibrating the high range.
                    //
                    logicalRangeLow = servo1.toLogicalPosition(currPosition);
                    setPosition(physicalRangeMax, currStepRate);
                }
                else if (logicalRangeHigh == -1.0 && upperLimitSwitch.isActive())
                {
                    //
                    // We finished calibrating the high range, we are done calibrating.
                    // Note: stop() will set calibrating to false.
                    //
                    logicalRangeHigh = servo1.toLogicalPosition(currPosition);
                    servo1.setLogicalRange(logicalRangeLow, logicalRangeHigh);
                    if (servo2 != null)
                    {
                        servo2.setLogicalRange(logicalRangeLow, logicalRangeHigh);
                    }
                    stop();
                }
                else if (currPosition == targetPosition)
                {
                    //
                    // Somehow, we reached the end and did not trigger a limit switch. Let's abort.
                    // Note: stop() will set calibrating to false.
                    //
                    stop();
                }
            }

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
            else if (!calibrating)
            {
                //
                // We have reached target and we are not calibrating, so we are done.
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
    }   //steppingServoTask

    /**
     * This method is called when the competition mode is about to end so it will stop the servo if necessary.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is running. (e.g. Autonomous, TeleOp, Test).
     */
    private void stopServoTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = "stopServoTask";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "taskType=%s,runMode=%s", taskType, runMode);
        }
        //
        // Note: stop() will set calibrating to false.
        //
        stop();

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //stopServoTask

}   //class TrcEnhancedServo
