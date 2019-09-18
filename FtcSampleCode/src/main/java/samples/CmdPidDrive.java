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
    private static final boolean debugXPid = false;
    private static final boolean debugYPid = false;
    private static final boolean debugTurnPid = false;

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
    private final boolean tuneMode;
    private final TrcPidController xPidCtrl;
    private final TrcPidController yPidCtrl;
    private final TrcPidController turnPidCtrl;
    private final TrcEvent event;
    private final TrcTimer timer;
    private final TrcStateMachine<State> sm;

    private TrcPidController tunePidCtrl = null;
    private PidCoefficients savedPidCoeff = null;
    private Boolean savedWarpSpaceEnabled = null;
    private Double savedXOutputLimit = null;
    private Double savedYOutputLimit = null;
    private Double savedTurnOutputLimit = null;

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
     * @param tuneMode specifies true if in tune mode which allows getting PID constants from the robot object
     *        for PID tuning, false otherwise.
     */
    public CmdPidDrive(
            K9Robot robot, TrcPidDrive pidDrive, double delay, double xDistance, double yDistance, double heading,
            double drivePowerLimit, boolean tuneMode)
    {
        robot.globalTracer.traceInfo(
                moduleName,
                "pidDrive=%s, delay=%.3f, xDist=%.1f, yDist=%.1f, heading=%.1f, powerLimit=%.1f, testMode=%s",
                pidDrive, delay, xDistance, yDistance, heading, drivePowerLimit, tuneMode);

        this.robot = robot;
        this.pidDrive = pidDrive;
        this.delay = delay;
        this.xDistance = xDistance;
        this.yDistance = yDistance;
        this.heading = heading;
        this.drivePowerLimit = drivePowerLimit;
        this.tuneMode = tuneMode;

        xPidCtrl = pidDrive.getXPidCtrl();
        yPidCtrl = pidDrive.getYPidCtrl();
        turnPidCtrl = pidDrive.getTurnPidCtrl();

        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
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
    */
    public CmdPidDrive(
            K9Robot robot, TrcPidDrive pidDrive, double delay, double xDistance, double yDistance, double heading)
    {
        this(robot, pidDrive, delay, xDistance, yDistance, heading, 1.0, false);
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
        if (pidDrive.isActive())
        {
            pidDrive.cancel();
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
            robot.dashboard.displayPrintf(1, "State: Disabled");
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
                        if (xDistance != 0.0 && (tunePidCtrl = pidDrive.getXPidCtrl()) != null ||
                            yDistance != 0.0 && (tunePidCtrl = pidDrive.getYPidCtrl()) != null ||
                            heading != 0.0 && (tunePidCtrl = pidDrive.getTurnPidCtrl()) != null)
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
                        savedXOutputLimit = xPidCtrl.getOutputLimit();
                        xPidCtrl.setOutputLimit(drivePowerLimit);
                    }

                    if (yDistance != 0.0 && yPidCtrl != null)
                    {
                        savedYOutputLimit = yPidCtrl.getOutputLimit();
                        yPidCtrl.setOutputLimit(drivePowerLimit);
                    }

                    if (heading != 0.0 && turnPidCtrl != null)
                    {
                        savedTurnOutputLimit = turnPidCtrl.getOutputLimit();
                        turnPidCtrl.setOutputLimit(drivePowerLimit);
                    }

                    pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done, restore everything.
                    //
                    if (tunePidCtrl != null)
                    {
                        tunePidCtrl.printPidInfo(robot.globalTracer);
                    }
                    if (savedXOutputLimit != null) xPidCtrl.setOutputLimit(savedXOutputLimit);
                    if (savedYOutputLimit != null) yPidCtrl.setOutputLimit(savedYOutputLimit);
                    if (savedTurnOutputLimit != null) turnPidCtrl.setOutputLimit(savedTurnOutputLimit);
                    if (savedPidCoeff != null) tunePidCtrl.setPidCoefficients(savedPidCoeff);
                    if (savedWarpSpaceEnabled != null) pidDrive.setWarpSpaceEnabled(savedWarpSpaceEnabled);
                    sm.stop();
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

            if (debugXPid && robot.encoderXPidCtrl != null)
            {
                pidDrive.getXPidCtrl().printPidInfo(robot.globalTracer, elapsedTime);
            }

            if (debugYPid)
            {
                pidDrive.getYPidCtrl().printPidInfo(robot.globalTracer, elapsedTime);
            }

            if (debugTurnPid)
            {
                pidDrive.getTurnPidCtrl().printPidInfo(robot.globalTracer, elapsedTime);
            }
        }

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //class CmdPidDrive
