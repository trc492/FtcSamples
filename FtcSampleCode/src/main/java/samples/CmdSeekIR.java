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
 * This class implements an autonomous routine that drives forward to the IR beacon using PID controlled drive.
 */
public class CmdSeekIR implements TrcRobot.RobotCommand
{
    private enum State
    {
        DO_DELAY,
        SEEK_IR,
        DONE
    }   //enum State

    private static final String moduleName = "CmdSeekIR";

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
    public CmdSeekIR(K9Robot robot, double delay, FtcAutoK9.Alliance alliance)
    {
        robot.globalTracer.traceInfo(moduleName, "delay=%.3f, alliance=%s", delay, alliance);

        this.robot = robot;
        this.delay = delay;
        this.alliance = alliance;

        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.DO_DELAY);
    }   //CmdSeekIR

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
                        sm.setState(State.SEEK_IR);
                        //
                        // Intentionally falling through to DO_PID_DRIVE.
                        //
                    }
                    else
                    {
                        timer.set(delay, event);
                        sm.waitForSingleEvent(event, State.SEEK_IR);
                        break;
                    }

                case SEEK_IR:
                    //
                    // Go towards IR beacon until IR strength reaches 0.8.
                    //
                    robot.pidSeekIr.setSensorTarget(0.0, 0.8, 0.0, event);
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
        }

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //class CmdSeekIR
