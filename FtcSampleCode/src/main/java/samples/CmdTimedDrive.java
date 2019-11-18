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
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

/**
 * This class implements a generic timed drive command. The command drives the robot in the given direction
 * for the given amount of time.
 */
public class CmdTimedDrive implements TrcRobot.RobotCommand
{
    private enum State
    {
        DO_DELAY,
        DRIVE_BY_TIME,
        DONE
    }   //enum State

    private static final String moduleName = "CmdTimedDrive";

    private K9Robot robot;
    private double delay;
    private double driveTime;
    private double xDrivePower;
    private double yDrivePower;
    private double turnPower;
    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param delay specifies delay in seconds before timed drive starts. 0 means no delay.
     * @param driveTime specifies the amount of drive time in seconds.
     * @param xDrivePower specifies the motor power in the X direction.
     * @param yDrivePower specifies the motor power in the Y direction.
     * @param turnPower specifies the motor power for turning.
     */
    public CmdTimedDrive(
            K9Robot robot, double delay, double driveTime, double xDrivePower, double yDrivePower, double turnPower)
    {
        robot.globalTracer.traceInfo(
                moduleName, "delay=%.3f, time=%.1f, xPower=%.1f, yPower=%.1f, turnPower=%.1f",
                delay, driveTime, xDrivePower, yDrivePower, turnPower);

        this.robot = robot;
        this.delay = delay;
        this.driveTime = driveTime;
        this.xDrivePower = xDrivePower;
        this.yDrivePower = yDrivePower;
        this.turnPower = turnPower;
        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.DO_DELAY);
    }   //CmdTimedDrive

    //
    // Implements the TrcRobot.AutoStrategy interface.
    //

    /**
     * This method checks if the current RobotCommand  is running.
     *
     * @return true if the command is running, false otherwise.
     */
    @Override
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
        robot.driveBase.stop();
        sm.stop();
    }   //cancel

    /**
     * This method must be called periodically by the caller to drive the command sequence forward.
     *
     * @param elapsedTime specifies the elapsed time in seconds since the start of the robot mode.
     * @return true if the command sequence is completed, false otherwise.
     */
    @Override
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
                        sm.setState(State.DRIVE_BY_TIME);
                        //
                        // Intentionally falling through to DRIVE_BY_TIME.
                        //
                    }
                    else
                    {
                        timer.set(delay, event);
                        sm.waitForSingleEvent(event, State.DRIVE_BY_TIME);
                        break;
                    }

                case DRIVE_BY_TIME:
                    //
                    // Drive the robot with the given power for a set amount of time.
                    //
                    if (robot.driveBase.supportsHolonomicDrive())
                    {
                        robot.driveBase.holonomicDrive(xDrivePower, yDrivePower, turnPower);
                    }
                    else
                    {
                        robot.driveBase.arcadeDrive(yDrivePower, turnPower);
                    }
                    timer.set(driveTime, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done.
                    //
                    cancel();
                    break;
            }
            robot.traceStateInfo(elapsedTime, state.toString(), 0.0, 0.0, 0.0);
        }

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //class CmdTimedDrive
