/*
 * Copyright (c) 2019 Titan Robotics Club (http://www.titanrobotics.com)
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
 * This class implements absolute target drive. It keeps track of the robot's absolute target position in order to
 * minimize cumulative error as a result of doing multiple segments of relative target drive. For example, if you
 * did a Y setTarget of 24 inches with a 2-inch tolerance and the robot only went 22 inches and you did another Y
 * setTarget of 12 inches and the robot only went 10 inches, the overall distance traveled would be 32 inches instead
 * of the intended 36 inches. The cumulative error would be 4 inches. Using this class, it keeps track of the absolute
 * target position. On the first segment, the absolute Y target was 24 inches and the robot went 22 inches. On the
 * second segment, the cumulative absolute Y target became 24 + 12 = 36 inches so the set target will be 36 - current
 * Y position = 36 - 22 = 14 inches. So even with a 2-inch error, the robot will travel 12 inches instead of 14 and
 * the overall distance traveled will be 22 + 12 = 34 instead of the intended overall target of 36. The error is
 * now just 2 inches instead of 4. No matter how many segment the robot travels, the overall error is still just
 * the 2-inch tolerance.
 *
 * @param <StateType> The type of states used in the state machine.
 */

public class TrcAbsTargetDrive<StateType>
{
    private static final String moduleName = "TrcAbsTargetDrive";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private final String instanceName;
    private final TrcDriveBase driveBase;
    private final TrcPidDrive pidDrive;
    private final TrcEvent event;
    private final TrcStateMachine<StateType> sm;
    private final boolean useAbsTarget;
    private TrcPose2D absTargetPose;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param driveBase specifies the drive base from which to get the robot odometry.
     * @param pidDrive specifies the PidDrive object to use for PID controlled drive.
     * @param event specifies the event to signal at the end of the drive.
     * @param sm specifies the state machine to advance to the next state at the end of the drive.
     * @param useAbsTarget specifies true to use absolute target (deltas added to absolute target) instead of deltas
     *                    are relative to current pose.
     */
    public TrcAbsTargetDrive(
            String instanceName, TrcDriveBase driveBase, TrcPidDrive pidDrive, TrcEvent event,
            TrcStateMachine<StateType> sm, boolean useAbsTarget)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                    TrcDbgTrace.getGlobalTracer():
                    new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
        this.driveBase = driveBase;
        this.pidDrive = pidDrive;
        this.event = event;
        this.sm = sm;
        this.useAbsTarget = useAbsTarget;
        absTargetPose = driveBase.getAbsolutePose();
    }   //TrcAbsTargetDrive

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param driveBase specifies the drive base from which to get the robot odometry.
     * @param pidDrive specifies the PidDrive object to use for PID controlled drive.
     * @param event specifies the event to signal at the end of the drive.
     * @param sm specifies the state machine to advance to the next state at the end of the drive.
     */
    public TrcAbsTargetDrive(
            String instanceName, TrcDriveBase driveBase, TrcPidDrive pidDrive, TrcEvent event,
            TrcStateMachine<StateType> sm)
    {
        this(instanceName, driveBase, pidDrive, event, sm, true);
    }   //TrcAbsTargetDrive

    /**
     * This method returns the instance name and the current target pose.
     *
     * @return instance name and current target pose.
     */
    @Override
    public String toString()
    {
        return instanceName + ": " + absTargetPose.toString();
    }   //toString

    /**
     * This method sets the PID controlled relative drive targets.
     *
     * @param xDelta specifies the X target relative to the current X position.
     * @param yDelta specifies the Y target relative to the current Y position.
     * @param turnDelta specifies the turn target relative to the current heading.
     * @param nextState specifies the next state the state machine should advance to at the end of the drive.
     */
    public void setTarget(double xDelta, double yDelta, double turnDelta, StateType nextState)
    {
        final String funcName = "setTarget";
        TrcPose2D newTargetPose = absTargetPose.translatePose(xDelta, yDelta);
        newTargetPose.heading += turnDelta;
        double xTarget, yTarget, turnTarget;

        if (debugEnabled)
        {
            dbgTrace.traceInfo(funcName, "xDelta=%.1f, yDelta=%.1f, turnDelta=%.1f, CurrPose:%s",
                    xDelta, yDelta, turnDelta, absTargetPose);
        }

        if (useAbsTarget)
        {
            TrcPose2D relativePose = newTargetPose.relativeTo(driveBase.getAbsolutePose());
            xTarget = relativePose.x;
            yTarget = relativePose.y;
        }
        else
        {
            xTarget = xDelta;
            yTarget = yDelta;
        }
        turnTarget = pidDrive.getTurnPidCtrl().hasAbsoluteSetPoint()? newTargetPose.heading: turnDelta;

        if (debugEnabled)
        {
            dbgTrace.traceInfo(funcName, "xTarget=%.1f, yTarget=%.1f, turnTarget=%.1f, NewPose:%s",
                    xTarget, yTarget, turnTarget, newTargetPose);
        }

        absTargetPose = newTargetPose;
        pidDrive.setTarget(xTarget, yTarget, turnTarget, false, event);
        sm.waitForSingleEvent(event, nextState);
    }   //setTarget

    /**
     * This method sets the PID controlled relative drive targets.
     *
     * @param xTarget specifies the X target relative to the current X position.
     * @param yTarget specifies the Y target relative to the current Y position.
     * @param nextState specifies the next state the state machine should advance to at the end of the drive.
     */
    public void setXYTarget(double xTarget, double yTarget, StateType nextState)
    {
        setTarget(xTarget, yTarget, 0.0, nextState);
    }   //setXYTarget

    /**
     * This method sets the PID controlled relative drive targets.
     *
     * @param xTarget specifies the X target relative to the current X position.
     * @param nextState specifies the next state the state machine should advance to at the end of the drive.
     */
    public void setXTarget(double xTarget, StateType nextState)
    {
        setTarget(xTarget, 0.0, 0.0, nextState);
    }   //setXTarget

    /**
     * This method sets the PID controlled relative drive targets.
     *
     * @param yTarget specifies the Y target relative to the current Y position.
     * @param nextState specifies the next state the state machine should advance to at the end of the drive.
     */
    public void setYTarget(double yTarget, StateType nextState)
    {
        setTarget(0.0, yTarget, 0.0, nextState);
    }   //setYTarget

    /**
     * This method sets the PID controlled relative drive targets.
     *
     * @param turnTarget specifies the turn target relative to the current heading.
     * @param nextState specifies the next state the state machine should advance to at the end of the drive.
     */
    public void setTurnTarget(double turnTarget, StateType nextState)
    {
        setTarget(0.0, 0.0, turnTarget, nextState);
    }   //setXYTarget

}   //TrcAbsTargetDrive
