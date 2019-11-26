/*
 * Copyright (c) 2018 Titan Robotics Club (http://www.titanrobotics.com)
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

package samples;

import trclib.TrcEvent;
import trclib.TrcPidController;
import trclib.TrcPidController.PidCoefficients;
import trclib.TrcPidDrive;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

/**
 * This class implements a generic PID control drive command. It is agnostic to the PID controller sensors.
 * The caller provides the PID drive object that has all PID controllers which means the caller controls
 * what sensors are controlling the X, Y and turn PID controllers. For example, the caller can provide a PID
 * drive object that uses the encoders to control the X and Y PID controllers and a gyro for the turn PID
 * controller. The caller can also use the encoders to control the X and Y PID controllers but a camera to
 * control the turn PID controller.
 */
public class CmdPidDrive implements TrcRobot.RobotCommand
{
    private static final boolean debugXPid = true;
    private static final boolean debugYPid = true;
    private static final boolean debugTurnPid = true;

    private enum State
    {
        DO_DELAY,
        DO_PID_DRIVE,
        DONE
    }   //enum State

    private static final String moduleName = "CmdPidDrive";

    private final K9Robot robot;
    private final TrcPidDrive pidDrive;
    private final double delay;
    private final double xDistance;
    private final double yDistance;
    private final double heading;
    private final double drivePowerLimit;
    private final boolean useSensorOdometry;
    private final boolean tuneMode;
    private final TrcEvent event;
    private final TrcTimer timer;
    private final TrcStateMachine<State> sm;
    private final TrcPidController xPidCtrl;
    private final TrcPidController yPidCtrl;
    private final TrcPidController turnPidCtrl;

    private TrcPidController tunePidCtrl = null;
    private PidCoefficients savedPidCoeff = null;
    private Boolean savedWarpSpaceEnabled = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param pidDrive specifies the PID drive object to be used for PID controlled drive.
     * @param delay specifies delay in seconds before PID drive starts. 0 means no delay.
     * @param xDistance specifies the target distance for the X direction.
     * @param yDistance specifies the target distance for the Y direction.
     * @param heading specifies the target heading.
     * @param drivePowerLimit specifies the power limit to be applied for the PID controlled drive.
     * @param useSensorOdometry specifies true to use the sensor odometry, false to use drive base odometry.
     * @param tuneMode specifies true if in tune mode which allows getting PID constants from the robot object
     *        for PID tuning, false otherwise.
     */
    public CmdPidDrive(
            K9Robot robot, TrcPidDrive pidDrive, double delay, double xDistance, double yDistance, double heading,
            double drivePowerLimit, boolean useSensorOdometry, boolean tuneMode)
    {
        robot.globalTracer.traceInfo(
                moduleName,
                "pidDrive=%s, delay=%.3f, xDist=%.1f, yDist=%.1f, heading=%.1f, powerLimit=%.1f, " +
                       "useSensorOdometry=%s, tuneMode=%s",
                pidDrive, delay, xDistance, yDistance, heading, drivePowerLimit, useSensorOdometry, tuneMode);

        this.robot = robot;
        this.pidDrive = pidDrive;
        this.delay = delay;
        this.xDistance = xDistance;
        this.yDistance = yDistance;
        this.heading = heading;
        this.drivePowerLimit = drivePowerLimit;
        this.useSensorOdometry = useSensorOdometry;
        this.tuneMode = tuneMode;

        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);

        xPidCtrl = pidDrive.getXPidCtrl();
        yPidCtrl = pidDrive.getYPidCtrl();
        turnPidCtrl = pidDrive.getTurnPidCtrl();

        sm.start(State.DO_DELAY);
    }   //CmdPidDrive

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param pidDrive specifies the PID drive object to be used for PID controlled drive.
     * @param delay specifies delay in seconds before PID drive starts. 0 means no delay.
     * @param xDistance specifies the target distance for the X direction.
     * @param yDistance specifies the target distance for the Y direction.
     * @param heading specifies the target heading.
     * @param useSensorOdometry specifies true to use the sensor odometry, false to use drive base odometry.
     */
    public CmdPidDrive(
            K9Robot robot, TrcPidDrive pidDrive, double delay, double xDistance, double yDistance, double heading,
            boolean useSensorOdometry)
    {
        this(robot, pidDrive, delay, xDistance, yDistance, heading, 1.0,
             useSensorOdometry, false);
    }   //CmdPidDrive

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param pidDrive specifies the PID drive object to be used for PID controlled drive.
     * @param delay specifies delay in seconds before PID drive starts. 0 means no delay.
     * @param xDistance specifies the target distance for the X direction.
     * @param yDistance specifies the target distance for the Y direction.
     * @param heading specifies the target heading.
     */
    public CmdPidDrive(
            K9Robot robot, TrcPidDrive pidDrive, double delay, double xDistance, double yDistance, double heading)
    {
        this(robot, pidDrive, delay, xDistance, yDistance, heading, 1.0,
             false, false);
    }   //CmdPidDrive

    //
    // Implements the TrcRobot.RobotCommand interface.
    //

    /**
     * This method checks if the current RobotCommand  is running.
     *
     * @return true if the command is running, false otherwise.
     */
    public boolean isActive()
    {
        return sm.isEnabled();
    }   //isActive

    /**
     * This method cancels the command if it is active.
     */
    @Override
    public void cancel()
    {
        if (pidDrive.isActive()) pidDrive.cancel();

        if (xPidCtrl != null) xPidCtrl.restoreOutputLimit();
        if (yPidCtrl != null) yPidCtrl.restoreOutputLimit();
        if (turnPidCtrl != null) turnPidCtrl.restoreOutputLimit();

        if (savedPidCoeff != null)
        {
            tunePidCtrl.setPidCoefficients(savedPidCoeff);
            savedPidCoeff = null;
        }

        if (savedWarpSpaceEnabled != null)
        {
            pidDrive.setWarpSpaceEnabled(savedWarpSpaceEnabled);
            savedWarpSpaceEnabled = null;
        }

        sm.stop();
    }   //cancel

    /**
     * This method must be called periodically by the caller to drive the command sequence forward.
     *
     * @param elapsedTime specifies the elapsed time in seconds since the start of the robot mode.
     * @return true if the command sequence is completed, false otherwise.
     */
    @Override
    @SuppressWarnings("unused")
    public boolean cmdPeriodic(double elapsedTime)
    {
        State state = sm.checkReadyAndGetState();

        if (state == null)
        {
            robot.dashboard.displayPrintf(1, "State: disabled or waiting...");
        }
        else
        {
            robot.dashboard.displayPrintf(1, "State: %s", state);

            switch (state)
            {
                case DO_DELAY:
                    //
                    // Do delay if any.
                    //
                    if (delay == 0.0)
                    {
                        sm.setState(State.DO_PID_DRIVE);
                        //
                        // Intentionally falling through to DO_PID_DRIVE.
                        //
                    }
                    else
                    {
                        timer.set(delay, event);
                        sm.waitForSingleEvent(event, State.DO_PID_DRIVE);
                        break;
                    }

                case DO_PID_DRIVE:
                    //
                    // Drive the set distance and heading.
                    //
                    if (tuneMode)
                    {
                        //
                        // We are in tune mode, we are tuning PID. We tune PID one direction at a time.
                        // Read PID constants from the robot and change the corresponding PID controller with them.
                        //
                        if (xDistance != 0.0 && (tunePidCtrl = xPidCtrl) != null ||
                            yDistance != 0.0 && (tunePidCtrl = yPidCtrl) != null ||
                            heading != 0.0 && (tunePidCtrl = turnPidCtrl) != null)
                        {
                            savedPidCoeff = tunePidCtrl.getPidCoefficients();
                            tunePidCtrl.setPidCoefficients(robot.tunePidCoeff);

                            robot.globalTracer.traceInfo("PidTuning", "%s: Kp=%f, Ki=%f, Kd=%f, Kf=%f",
                                    tunePidCtrl, robot.tunePidCoeff.kP, robot.tunePidCoeff.kI, robot.tunePidCoeff.kD,
                                    robot.tunePidCoeff.kF);
                        }
                        //
                        // Do not optimize turning if we are tuning PID.
                        //
                        savedWarpSpaceEnabled = pidDrive.isWarpSpaceEnabled();
                        pidDrive.setWarpSpaceEnabled(false);
                    }
                    //
                    // Set power limits for each direction if applicable.
                    //
                    if (xDistance != 0.0 && xPidCtrl != null)
                    {
                        xPidCtrl.saveAndSetOutputLimit(drivePowerLimit);
                    }

                    if (yDistance != 0.0 && yPidCtrl != null)
                    {
                        yPidCtrl.saveAndSetOutputLimit(drivePowerLimit);
                    }

                    if (heading != 0.0 && turnPidCtrl != null)
                    {
                        turnPidCtrl.saveAndSetOutputLimit(drivePowerLimit);
                    }

                    if (useSensorOdometry)
                    {
                        pidDrive.setSensorTarget(xDistance, yDistance, heading, event);
                    }
                    else
                    {
                        pidDrive.setRelativeTarget(xDistance, yDistance, heading, event);
                    }

                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done, restore everything.
                    //
                    cancel();
                    break;
            }

            robot.traceStateInfo(elapsedTime, state.toString(), xDistance, yDistance, heading);
        }

        if (pidDrive.isActive() && (debugXPid || debugYPid || debugTurnPid))
        {
            if (robot.battery != null)
            {
                robot.globalTracer.traceInfo("Battery", "Voltage=%5.2fV (%5.2fV)",
                        robot.battery.getVoltage(), robot.battery.getLowestVoltage());
            }

            robot.globalTracer.traceInfo(moduleName, "RobotPose: %s", robot.driveBase.getAbsolutePose());

            if (debugXPid && xPidCtrl != null)
            {
                xPidCtrl.printPidInfo(robot.globalTracer, elapsedTime);
            }

            if (debugYPid && yPidCtrl != null)
            {
                yPidCtrl.printPidInfo(robot.globalTracer, elapsedTime);
            }

            if (debugTurnPid && turnPidCtrl != null)
            {
                turnPidCtrl.printPidInfo(robot.globalTracer, elapsedTime);
            }
        }

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //class CmdPidDrive
