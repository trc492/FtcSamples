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
 * This class implements an autonomous routine that drives forward a specified distance and then turn a specified
 * degrees using PID controlled drive.
 */
public class CmdDriveAndTurn implements TrcRobot.RobotCommand
{
    private static final boolean debugYPid = false;
    private static final boolean debugTurnPid = false;

    private enum State
    {
        DO_DELAY,
        DO_PID_DRIVE,
        DO_PID_TURN,
        DONE
    }   //enum State

    private static final String moduleName = "CmdDriveAndTurn";

    private final K9Robot robot;
    private final double delay;
    private final double yDistance;
    private final double turnDegrees;
    private final TrcEvent event;
    private final TrcTimer timer;
    private final TrcStateMachine<State> sm;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param delay specifies delay in seconds before PID drive starts. 0 means no delay.
     * @param yDistance specifies the target distance in inches for the Y direction.
     * @param turnDegrees specifies the degrees to turn.
     */
    public CmdDriveAndTurn(K9Robot robot, double delay, double yDistance, double turnDegrees)
    {
        robot.globalTracer.traceInfo(moduleName,
                "delay=%.3f, yDist=%.1f, degrees=%.1f", delay, yDistance, turnDegrees);

        this.robot = robot;
        this.delay = delay;
        this.yDistance = yDistance;
        this.turnDegrees = turnDegrees;

        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.DO_DELAY);
    }   //CmdDriveAndTurn

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
                    // Drive the set distance and maintain the same heading.
                    //
                    robot.pidDrive.setTarget(yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DO_PID_TURN);
                    break;

                case DO_PID_TURN:
                    //
                    // Update the robot target heading with the degrees to turn and turn to the new heading.
                    //
                    robot.targetHeading += turnDegrees;
                    robot.pidDrive.setTarget(0.0, robot.targetHeading, false, event);
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

            robot.traceStateInfo(elapsedTime, state.toString(), 0.0, yDistance, robot.targetHeading);
        }

        if (robot.pidDrive.isActive() && (debugYPid || debugTurnPid))
        {
            if (robot.battery != null)
            {
                robot.globalTracer.traceInfo("Battery", "Voltage=%5.2fV (%5.2fV)",
                        robot.battery.getVoltage(), robot.battery.getLowestVoltage());
            }

            if (debugYPid)
            {
                robot.pidDrive.getYPidCtrl().printPidInfo(robot.globalTracer, elapsedTime);
            }

            if (debugTurnPid)
            {
                robot.pidDrive.getTurnPidCtrl().printPidInfo(robot.globalTracer, elapsedTime);
            }
        }

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //class CmdDriveAndTurn
