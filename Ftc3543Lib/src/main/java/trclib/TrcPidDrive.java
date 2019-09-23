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

import trclib.TrcTaskMgr.TaskType;

/**
 * This class implements a PID controlled robot drive. A PID controlled robot drive consist of a robot drive base
 * and three PID controllers, one for the X direction, one for the Y direction and one for turn. If the robot drive
 * base is incapable of moving in the X direction, the X PID controller will be null. In addition, it has stall
 * detection support which will detect motor stall condition. The motors on a drive base could stall if the robot
 * runs into an obstacle in low power or the robot is very close to target and doesn't have enough power to overcome
 * steady state error. When stall condition is detected, PID drive will be aborted so that the robot won't get stuck
 * waiting forever trying to reach target.
 */
public class TrcPidDrive
{
    private static final String moduleName = "TrcPidDrive";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;
    private TrcDbgTrace msgTracer = null;
    private TrcRobotBattery battery = null;
    private boolean tracePidInfo = false;
    private TrcPose2D oldReferenceFrame = null;

    /**
     * This interface provides a stuck wheel notification handler. It is useful for detecting drive base motor
     * malfunctions. A stuck wheel could happen if the motor is malfunctioning, the motor power wire is unplugged,
     * the motor encoder is malfunctioning or the motor encoder wire is unplugged.
     */
    public interface StuckWheelHandler
    {
        /**
         * This method is called when a stuck wheel is detected.
         *
         * @param pidDrive specifies this TrcPidDrive instance.
         * @param index specifies which wheel in the DriveBase is stuck.
         */
        void stuckWheel(TrcPidDrive pidDrive, int index);

    }   //interface StuckWheelHandler

    /**
     * Turn mode specifies how PID controlled drive is turning the robot.
     */
    public enum TurnMode
    {
        IN_PLACE,
        PIVOT_FORWARD,
        PIVOT_BACKWARD,
        CURVE
    }   //enum TurnMode

    private static final double DEF_BEEP_FREQUENCY = 880.0; //in Hz
    private static final double DEF_BEEP_DURATION = 0.2;    //in seconds

    private final String instanceName;
    private final TrcDriveBase driveBase;
    private final TrcPidController xPidCtrl;
    private final TrcPidController yPidCtrl;
    private final TrcPidController turnPidCtrl;
    private final TrcTaskMgr.TaskObject driveTaskObj;
    private final TrcTaskMgr.TaskObject stopTaskObj;
    private TrcWarpSpace warpSpace = null;
    private boolean warpSpaceEnabled = true;
    private StuckWheelHandler stuckWheelHandler = null;
    private double stuckTimeout = 0.0;
    private TurnMode turnMode = TurnMode.IN_PLACE;
    private TrcTone beepDevice = null;
    private double beepFrequency = DEF_BEEP_FREQUENCY;
    private double beepDuration = DEF_BEEP_DURATION;
    private double stallTimeout = 0.0;
    private TrcEvent notifyEvent = null;
    private double expiredTime = 0.0;
    private double manualX = 0.0;
    private double manualY = 0.0;
    private boolean active = false;
    private boolean holdTarget = false;
    private boolean turnOnly = false;
    private boolean maintainHeading = false;
    private boolean canceled = false;
    private String owner = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param driveBase specifies the drive base object.
     * @param xPidCtrl specifies the PID controller for the X direction.
     * @param yPidCtrl specifies the PID controller for the Y direction.
     * @param turnPidCtrl specifies the PID controller for turn.
     */
    public TrcPidDrive(
        String instanceName, TrcDriveBase driveBase,
        TrcPidController xPidCtrl, TrcPidController yPidCtrl, TrcPidController turnPidCtrl)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
        this.driveBase = driveBase;
        this.xPidCtrl = xPidCtrl;
        this.yPidCtrl = yPidCtrl;
        this.turnPidCtrl = turnPidCtrl;
        TrcTaskMgr taskMgr = TrcTaskMgr.getInstance();
        driveTaskObj = taskMgr.createTask(instanceName + ".driveTask", this::driveTask);
        stopTaskObj = taskMgr.createTask(instanceName + ".stopTask", this::stopTask);

        if (turnPidCtrl != null && turnPidCtrl.hasAbsoluteSetPoint())
        {
            warpSpace = new TrcWarpSpace(instanceName, 0.0, 360.0);
        }
    }   //TrcPidDrive

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
     * This method sets the message tracer for logging trace messages.
     *
     * @param tracer specifies the tracer for logging messages.
     * @param tracePidInfo specifies true to enable tracing of PID info, false otherwise.
     * @param battery specifies the battery object to get battery info for the message.
     */
    public synchronized void setMsgTracer(TrcDbgTrace tracer, boolean tracePidInfo, TrcRobotBattery battery)
    {
        this.msgTracer = tracer;
        this.tracePidInfo = tracePidInfo;
        this.battery = battery;
    }   //setMsgTracer

    /**
     * This method sets the message tracer for logging trace messages.
     *
     * @param tracer specifies the tracer for logging messages.
     * @param tracePidInfo specifies true to enable tracing of PID info, false otherwise.
     */
    public void setMsgTracer(TrcDbgTrace tracer, boolean tracePidInfo)
    {
        setMsgTracer(tracer, tracePidInfo, null);
    }   //setMsgTracer

    /**
     * This method sets the message tracer for logging trace messages.
     *
     * @param tracer specifies the tracer for logging messages.
     */
    public void setMsgTracer(TrcDbgTrace tracer)
    {
        setMsgTracer(tracer, false, null);
    }   //setMsgTracer

    /**
     * This method checks if warpspace processing is enabled in a PID controlled turn.
     *
     * @return true if warpspace processing is enabled, false otherwise.
     */
    public boolean isWarpSpaceEnabled()
    {
        return warpSpaceEnabled;
    }   //isWarpSpaceEnabled

    /**
     * This method enables/disables warpspace processing when doing a PID controlled turn.
     *
     * @param enabled specifies true to enable warpspace processing, false to disable.
     */
    public void setWarpSpaceEnabled(boolean enabled)
    {
        warpSpaceEnabled = enabled;
    }   //setWarpSpaceEnabled

    /**
     * This method returns the X PID controller if any.
     *
     * @return X PID controller.
     */
    public TrcPidController getXPidCtrl()
    {
        final String funcName = "getXPidCtrl";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s",
                    xPidCtrl != null? xPidCtrl.toString(): "null");
        }

        return xPidCtrl;
    }   //getXPidCtrl

    /**
     * This method returns the Y PID controller if any.
     *
     * @return Y PID controller.
     */
    public TrcPidController getYPidCtrl()
    {
        final String funcName = "getYPidCtrl";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s",
                    yPidCtrl != null? yPidCtrl.toString(): "null");
        }

        return yPidCtrl;
    }   //getYPidCtrl

    /**
     * This method returns the Turn PID controller if any.
     *
     * @return Turn PID controller.
     */
    public TrcPidController getTurnPidCtrl()
    {
        final String funcName = "getTurnPidCtrl";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s",
                    turnPidCtrl != null? turnPidCtrl.toString(): "null");
        }

        return turnPidCtrl;
    }   //getTurnPidCtrl

    /**
     * This method sets a stuck wheel handler to enable stuck wheel detection.
     *
     * @param stuckWheelHandler specifies the stuck wheel handler.
     * @param stuckTimeout specifies the stuck timeout in seconds.
     */
    public synchronized void setStuckWheelHandler(StuckWheelHandler stuckWheelHandler, double stuckTimeout)
    {
        final String funcName = "setStuckWheelHandler";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                    "stuckWheelHandler=%s,timeout=%f", stuckWheelHandler, stuckTimeout);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.stuckWheelHandler = stuckWheelHandler;
        this.stuckTimeout = stuckTimeout;
    }   //setStuckWheelHandler

    /**
     * This methods sets the turn mode. Supported modes are in-place (default), pivot and curve.
     *
     * @param turnMode specifies the turn mode to set to.
     */
    public void setTurnMode(TurnMode turnMode)
    {
        final String funcName = "setTurnMode";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "turnMode=%s", turnMode);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.turnMode = turnMode;
    }   //setTurnMode

    /**
     * This method returns the current turn mode.
     *
     * @return current turn mode.
     */
    public TurnMode getTurnMode()
    {
        final String funcName = "getTurnMode";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", turnMode);
        }

        return turnMode;
    }   //getTurnMode

    /**
     * This method sets the beep device and the beep tones so that it can play beeps when motor stalled or if the
     * limit switches are activated/deactivated.
     *
     * @param beepDevice specifies the beep device object.
     * @param beepFrequency specifies the beep frequency.
     * @param beepDuration specifies the beep duration.
     */
    public synchronized void setBeep(TrcTone beepDevice, double beepFrequency, double beepDuration)
    {
        final String funcName = "setBeep";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                    "beep=%s,freq=%.0f,duration=%.3f", beepDevice.toString(), beepFrequency, beepDuration);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.beepDevice = beepDevice;
        this.beepFrequency = beepFrequency;
        this.beepDuration = beepDuration;
    }   //setBeep

    /**
     * This method sets the beep device so that it can play beeps at default frequency and duration when motor
     * stalled or if the limit switches are activated/deactivated.
     *
     * @param beepDevice specifies the beep device object.
     */
    public void setBeep(TrcTone beepDevice)
    {
        setBeep(beepDevice, DEF_BEEP_FREQUENCY, DEF_BEEP_DURATION);
    }   //setBeep

    /**
     * This method sets the stall timeout which is the minimum elapsed time for the wheels to be motionless to be
     * considered stalled.
     *
     * @param stallTimeout specifies stall timeout in seconds.
     */
    public void setStallTimeout(double stallTimeout)
    {
        final String funcName = "setStallTimeout";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "timeout=%.3f", stallTimeout);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.stallTimeout = stallTimeout;
    }   //setStallTimeout

    /**
     * This method starts a PID operation by setting the PID targets.
     *
     * @param owner specifies the ID string of the caller requesting exclusive access.
     * @param xTarget specifies the X target position.
     * @param yTarget specifies the Y target position.
     * @param turnTarget specifies the target heading.
     * @param holdTarget specifies true for holding the target position at the end, false otherwise.
     * @param event specifies an event object to signal when done.
     * @param timeout specifies a timeout value in seconds. If the operation is not completed without the specified
     *                timeout, the operation will be canceled and the event will be signaled. If no timeout is
     *                specified, it should be set to zero.
     */
    public synchronized void setTarget(String owner, double xTarget, double yTarget, double turnTarget,
        boolean holdTarget, TrcEvent event, double timeout)
    {
        final String funcName = "setTarget";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API, "x=%f,y=%f,turn=%f,hold=%s,event=%s,timeout=%.3f",
                    xTarget, yTarget, turnTarget, Boolean.toString(holdTarget), event.toString(), timeout);
        }

        if (driveBase.validateOwnership(owner))
        {
            this.owner = owner;
            double xError = 0.0, yError = 0.0, turnError = 0.0;

            if (xPidCtrl != null)
            {
                xPidCtrl.setTarget(xTarget);
                xError = xPidCtrl.getError();
            }

            if (yPidCtrl != null)
            {
                yPidCtrl.setTarget(yTarget);
                yError = yPidCtrl.getError();
            }

            if (turnPidCtrl != null)
            {
                turnPidCtrl.setTarget(turnTarget, warpSpaceEnabled? warpSpace: null);
                turnError = turnPidCtrl.getError();
            }

            if (event != null)
            {
                event.clear();
            }
            this.notifyEvent = event;

            this.expiredTime = timeout;
            if (timeout != 0)
            {
                this.expiredTime += TrcUtil.getCurrentTime();
            }

            this.holdTarget = holdTarget;
            this.turnOnly = xError == 0.0 && yError == 0.0 && turnError != 0.0;
            driveBase.resetStallTimer();

            // Cache the reference pose to reset it after the pid operation
            oldReferenceFrame = driveBase.getReferencePose();
            driveBase.setReferencePose();

            setTaskEnabled(true);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setTarget

    /**
     * This method starts a PID operation by setting the PID targets.
     *
     * @param xTarget    specifies the X target position.
     * @param yTarget    specifies the Y target position.
     * @param turnTarget specifies the target heading.
     * @param holdTarget specifies true for holding the target position at the end, false otherwise.
     * @param event      specifies an event object to signal when done.
     * @param timeout    specifies a timeout value in seconds. If the operation is not completed without the specified
     *                   timeout, the operation will be canceled and the event will be signaled. If no timeout is
     *                   specified, it should be set to zero.
     */
    public synchronized void setTarget(double xTarget, double yTarget, double turnTarget, boolean holdTarget,
                                       TrcEvent event, double timeout)
    {
        setTarget(null, xTarget, yTarget, turnTarget, holdTarget, event, timeout);
    }   //setTarget

    /**
     * This method starts a PID operation by setting the PID targets.
     *
     * @param xTarget specifies the X target position.
     * @param yTarget specifies the Y target position.
     * @param turnTarget specifies the target heading.
     * @param holdTarget specifies true for holding the target position at the end, false otherwise.
     * @param event specifies an event object to signal when done.
     */
    public void setTarget(double xTarget, double yTarget, double turnTarget, boolean holdTarget, TrcEvent event)
    {
        setTarget(xTarget, yTarget, turnTarget, holdTarget, event, 0.0);
    }   //setTarget

    /**
     * This method starts a PID operation by setting the PID targets.
     *
     * @param yTarget specifies the Y target position.
     * @param turnTarget specifies the target heading.
     * @param holdTarget specifies true for holding the target position at the end, false otherwise.
     * @param event specifies an event object to signal when done.
     * @param timeout specifies a timeout value in seconds. If the operation is not completed without the specified
     *                timeout, the operation will be canceled and the event will be signaled. If no timeout is
     *                specified, it should be set to zero.
     */
    public void setTarget(double yTarget, double turnTarget, boolean holdTarget, TrcEvent event, double timeout)
    {
        setTarget(0.0, yTarget, turnTarget, holdTarget, event, timeout);
    }   //setTarget

    /**
     * This method starts a PID operation by setting the PID targets.
     *
     * @param yTarget specifies the Y target position.
     * @param turnTarget specifies the target heading.
     * @param holdTarget specifies true for holding the target position at the end, false otherwise.
     * @param event specifies an event object to signal when done.
     */
    public void setTarget(double yTarget, double turnTarget, boolean holdTarget, TrcEvent event)
    {
        setTarget(0.0, yTarget, turnTarget, holdTarget, event, 0.0);
    }   //setTarget

    /**
     * This method allows a mecanum drive base to drive and maintain a fixed heading.
     *
     * @param xPower specifies the X drive power.
     * @param yPower specifies the Y drive power.
     * @param headingTarget specifies the heading to maintain.
     */
    public synchronized void driveMaintainHeading(double xPower, double yPower, double headingTarget)
    {
        final String funcName = "driveMaintainHeading";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                    "xPower=%f,yPower=%f,heading=%f", xPower, yPower, headingTarget);
        }

        if (xPidCtrl != null)
        {
            manualX = xPower;
            manualY = yPower;
            if (turnPidCtrl != null)
            {
                turnPidCtrl.setTarget(headingTarget);
            }
            maintainHeading = true;
            setTaskEnabled(true);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //driveMaintainHeading

    /**
     * This method checks if a PID drive operation is currently active.
     *
     * @return true if PID drive is active, false otherwise.
     */
    public boolean isActive()
    {
        final String funcName = "isActive";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", active);
        }

        return active;
    }   //isActive

    /**
     * This method cancels an active PID drive operation and stops all motors.
     */
    public synchronized void cancel()
    {
        final String funcName = "cancel";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (active && driveBase.validateOwnership(owner))
        {
            stopPid();
            canceled = true;
            if (notifyEvent != null)
            {
                notifyEvent.cancel();
                notifyEvent = null;
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //cancel

    /**
     * This method checks if a PID drive operation was canceled.
     *
     * @return true if PID drive is active, false otherwise.
     */
    public boolean isCanceled()
    {
        final String funcName = "isCanceled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Boolean.toString(canceled));
        }

        return canceled;
    }   //isCanceled

    /**
     * This method stops the PID drive operation and reset the states.
     */
    private synchronized void stopPid()
    {
        final String funcName = "stopPid";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC);
        }

        setTaskEnabled(false);
        driveBase.stop(owner);

        if (xPidCtrl != null)
        {
            xPidCtrl.reset();
        }

        if (yPidCtrl != null)
        {
            yPidCtrl.reset();
        }

        if (turnPidCtrl != null)
        {
            turnPidCtrl.reset();
        }

        // reset the reference pose
        driveBase.setReferencePose(oldReferenceFrame);

        holdTarget = false;
        turnOnly = false;
        maintainHeading = false;
        canceled = false;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC);
        }
    }   //stopPid

    /**
     * This method enables/disables the PID drive task. It assumes the caller has the synchronized lock.
     *
     * @param enabled specifies true to enable PID drive task, false to disable.
     */
    private void setTaskEnabled(boolean enabled)
    {
        final String funcName = "setTaskEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC, "enabled=%b", enabled);
        }

        if (enabled)
        {
            driveTaskObj.registerTask(TaskType.OUTPUT_TASK);
            stopTaskObj.registerTask(TaskType.STOP_TASK);
        }
        else
        {
            driveTaskObj.unregisterTask(TaskType.OUTPUT_TASK);
            stopTaskObj.unregisterTask(TaskType.STOP_TASK);
        }
        active = enabled;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC);
        }
    }   //setTaskEnabled

    /**
     * This method is called periodically to execute the PID drive operation.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is about to end (e.g. Autonomous, TeleOp, Test).
     */
    private synchronized void driveTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = "driveTask";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "taskType=%s,runMode=%s", taskType, runMode);
        }

        double xPower = turnOnly || xPidCtrl == null? 0.0: xPidCtrl.getOutput();
        double yPower = turnOnly || yPidCtrl == null? 0.0: yPidCtrl.getOutput();
        double turnPower = turnPidCtrl == null? 0.0: turnPidCtrl.getOutput();

        boolean expired = expiredTime != 0.0 && TrcUtil.getCurrentTime() >= expiredTime;
        boolean stalled = stallTimeout != 0.0 && driveBase.isStalled(stallTimeout);
        boolean xOnTarget = xPidCtrl == null || xPidCtrl.isOnTarget();
        boolean yOnTarget = yPidCtrl == null || yPidCtrl.isOnTarget();
        boolean turnOnTarget = turnPidCtrl == null || turnPidCtrl.isOnTarget();
        boolean onTarget = turnOnTarget && (turnOnly || xOnTarget && yOnTarget);

        if (stuckWheelHandler != null)
        {
            for (int i = 0; i < driveBase.getNumMotors(); i++)
            {
                if(driveBase.isMotorStalled(i, stuckTimeout))
                {
                    stuckWheelHandler.stuckWheel(this, i);
                }
            }
        }

        if (stalled || expired)
        {
            if (beepDevice != null)
            {
                beepDevice.playTone(beepFrequency, beepDuration);
            }

            if (msgTracer != null)
            {
                msgTracer.traceInfo(funcName, "%s: Stalled=%s, Expired=%s", instanceName, stalled, expired);
            }
        }

        if (maintainHeading && driveBase.supportsHolonomicDrive())
        {
            driveBase.holonomicDrive(owner, manualX, manualY, turnPower, false, 0.0);
        }
        else if (expired || stalled || onTarget)
        {
            if (expired || stalled || !holdTarget)
            {
                stopPid();
                if (notifyEvent != null)
                {
                    notifyEvent.set(true);
                    notifyEvent = null;
                }
            }
            // If we come here, both onTarget and holdTarget are true.
            // We will stop the drive base but not stopping PID.
            else
            {
                driveBase.stop(owner);
            }
        }
        // If we come here, we are not on target yet, keep driving.
        else if (xPidCtrl != null && driveBase.supportsHolonomicDrive())
        {
            driveBase.holonomicDrive(owner, xPower, yPower, turnPower);
        }
        else if (turnOnly)
        {
            switch (turnMode)
            {
                case IN_PLACE:
                    driveBase.arcadeDrive(owner, 0.0, turnPower, false);
                    break;

                case PIVOT_FORWARD:
                case CURVE:
                    if (turnPower < 0.0)
                    {
                        driveBase.tankDrive(owner, 0.0, -turnPower);
                    }
                    else
                    {
                        driveBase.tankDrive(owner, turnPower, 0.0);
                    }
                    break;

                case PIVOT_BACKWARD:
                    if (turnPower < 0.0)
                    {
                        driveBase.tankDrive(owner, turnPower, 0.0);
                    }
                    else
                    {
                        driveBase.tankDrive(owner, 0.0, -turnPower);
                    }
                    break;
            }
        }
        else if (turnMode == TurnMode.IN_PLACE)
        {
            // We are still in an in-place turn.
            driveBase.arcadeDrive(owner, yPower, turnPower, false);
        }
        else
        {
            driveBase.curveDrive(owner, yPower, turnPower, false);
        }

        if (msgTracer != null && tracePidInfo)
        {
            double currTime = TrcUtil.getCurrentTime();
            if (xPidCtrl != null) xPidCtrl.printPidInfo(msgTracer, currTime, battery);
            if (yPidCtrl != null) yPidCtrl.printPidInfo(msgTracer, currTime, battery);
            if (turnPidCtrl != null) turnPidCtrl.printPidInfo(msgTracer, currTime, battery);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //driveTask

    /**
     * This method is called when the competition mode is about to end to stop the PID drive operation if any.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is about to end (e.g. Autonomous, TeleOp, Test).
     */
    private void stopTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = "stopTask";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "taskType=%s,runMode=%s", taskType, runMode);
        }

        stopPid();

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //stopTask

}   //class TrcPidDrive
