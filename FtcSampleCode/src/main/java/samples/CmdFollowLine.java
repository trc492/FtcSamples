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
 * This class implements an autonomous routine that drives forward to find the line. Then it will drive the robot
 * following the line using PID controlled drive.
 */
public class CmdFollowLine implements TrcRobot.RobotCommand
{
    private enum State
    {
        DO_DELAY,
        FIND_LINE,
        TURN_TO_LINE,
        FOLLOW_LINE,
        DONE
    }   //enum State

    private static final String moduleName = "CmdFollowLine";

    private final K9Robot robot;
    private final double delay;
    private final FtcAutoK9.Alliance alliance;
    private final TrcEvent event;
    private final TrcTimer timer;
    private final TrcStateMachine<State> sm;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param delay specifies delay in seconds before PID drive starts. 0 means no delay.
     * @param alliance specifies the alliance we are on.
     */
    public CmdFollowLine(K9Robot robot, double delay, FtcAutoK9.Alliance alliance)
    {
        robot.globalTracer.traceInfo(moduleName,
                "delay=%.3f, alliance=%s", delay, alliance);

        this.robot = robot;
        this.delay = delay;
        this.alliance = alliance;

        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.DO_DELAY);
    }   //CmdFollowLine

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
        if (robot.pidDrive.isActive())
        {
            robot.pidDrive.cancel();
        }

        if (robot.pidLineFollow.isActive())
        {
            robot.pidLineFollow.cancel();
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
                        sm.setState(State.FIND_LINE);
                        //
                        // Intentionally falling through to DO_PID_DRIVE.
                        //
                    }
                    else
                    {
                        timer.set(delay, event);
                        sm.waitForSingleEvent(event, State.FIND_LINE);
                        break;
                    }

                case FIND_LINE:
                    //
                    // Go forward slowly for 3 ft to find the line.
                    // If line is detected, PID drive will be interrupted.
                    //
                    robot.colorTrigger.setEnabled(true);
                    // Drive slowly, limit to half power.
                    robot.encoderYPidCtrl.setOutputRange(-0.5, 0.5);
                    robot.pidDrive.setTarget(36.0, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.TURN_TO_LINE);
                    break;

                case TURN_TO_LINE:
                    //
                    // We have past the line slightly, so turn left or right 90 degree
                    // slowly to find the edge of the line. If the line is detected,
                    // PID turn will be interrupted.
                    //
                    robot.gyroPidCtrl.setOutputRange(-0.5, 0.5);
                    robot.pidDrive.setTarget(
                            0.0,
                            alliance == FtcAutoK9.Alliance.RED_ALLIANCE? -90.0: 90.0,
                            false, event);
                    sm.waitForSingleEvent(event, State.FOLLOW_LINE);
                    break;

                case FOLLOW_LINE:
                    //
                    // Slowly follow the line for 5 ft.
                    //
                    robot.colorTrigger.setEnabled(false);
                    robot.encoderYPidCtrl.setOutputRange(-0.3, 0.3);
                    robot.colorPidCtrl.setOutputRange(-0.3, 0.3);
                    //
                    // Follow right edge if red alliance.
                    // Follow left edge if blue alliance.
                    //
                    robot.colorPidCtrl.setInverted(alliance == FtcAutoK9.Alliance.RED_ALLIANCE);
                    robot.pidLineFollow.setTarget(60.0, K9Robot.COLOR_LINE_EDGE_DEADBAND, false, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done, restore everything.
                    //
                    sm.stop();
                    break;
            }

            robot.traceStateInfo(elapsedTime, state.toString(), 0.0, 0.0, robot.targetHeading);
        }

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //class CmdFollowLine
