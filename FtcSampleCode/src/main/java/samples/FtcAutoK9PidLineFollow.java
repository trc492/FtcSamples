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

package samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import ftclib.FtcOpMode;
import trclib.TrcAnalogTrigger;
import trclib.TrcEvent;
import trclib.TrcPidController;
import trclib.TrcPidDrive;
import trclib.TrcRobot;
import trclib.TrcStateMachine;

@Autonomous(name="Auto: K9Bot PID Line Following", group="3543AutoSamples")
@Disabled
public class FtcAutoK9PidLineFollow extends FtcOpMode
{
    //
    // State machine states.
    //
    private enum State
    {
        FIND_LINE,
        TURN_TO_LINE,
        FOLLOW_LINE,
        DONE
    }

    private K9Robot robot;
    //
    // Event and state machine.
    //
    private TrcEvent event;
    private TrcStateMachine sm;

    //
    // Implements FtcOpMode abstract methods.
    //

    @Override
    public void initRobot()
    {
        robot = new K9Robot(TrcRobot.RunMode.AUTO_MODE);
        //
        // State machine.
        //
        event = new TrcEvent("driveEvent");
        sm = new TrcStateMachine("lineFollow");
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void startMode(TrcRobot.RunMode prevMode)
    {
        robot.startMode(TrcRobot.RunMode.AUTO_MODE);
        //
        // Start state machine at state FIND_LINE.
        //
        sm.start(State.FIND_LINE);
    }   //startMode

    @Override
    public void stopMode(TrcRobot.RunMode nextMode)
    {
        robot.stopMode(TrcRobot.RunMode.AUTO_MODE);
    }   //stopMode

    @Override
    public void runContinuous(double elapsedTime)
    {
        robot.drivePidCtrl.displayPidInfo(1);
        robot.turnPidCtrl.displayPidInfo(3);
        robot.lightPidCtrl.displayPidInfo(5);
        //
        // Run state machine.
        //
        if (sm.isReady())
        {
            State state = (State)sm.getState();
            robot.dashboard.displayPrintf(7, "State: %s", state.toString());
            switch (state)
            {
                case FIND_LINE:
                    //
                    // Go forward slowly for 3 ft to find the line.
                    // If line is detected, PID drive will be interrupted.
                    //
                    robot.lightTrigger.setTaskEnabled(true);
                    robot.drivePidCtrl.setOutputRange(-0.5, 0.5);
                    robot.pidDrive.setTarget(36.0, 0.0, false, event);
                    sm.addEvent(event);
                    sm.waitForEvents(State.TURN_TO_LINE);
                    break;

                case TURN_TO_LINE:
                    //
                    // We have past the line slightly, so turn right 90 degree
                    // slowly to find the edge of the line. If the line is detected,
                    // PID turn will be interrupted.
                    //
                    robot.turnPidCtrl.setOutputRange(-0.5, 0.5);
                    robot.pidDrive.setTarget(0.0, 90.0, false, event);
                    sm.addEvent(event);
                    sm.waitForEvents(State.FOLLOW_LINE);
                    break;

                case FOLLOW_LINE:
                    //
                    // Follow the line for 5 ft.
                    //
                    robot.lightTrigger.setTaskEnabled(false);
                    robot.drivePidCtrl.setOutputRange(-0.3, 0.3);
                    robot.lightPidCtrl.setOutputRange(-0.3, 0.3);
                    robot.lineFollowDrive.setTarget(60.0, K9Robot.LIGHT_THRESHOLD, false, event);
                    sm.addEvent(event);
                    sm.waitForEvents(State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done, restore everything and stop!
                    //
                    robot.drivePidCtrl.setOutputRange(-1.0, 1.0);
                    robot.turnPidCtrl.setOutputRange(-1.0, 1.0);
                    robot.lightPidCtrl.setOutputRange(-1.0, 1.0);
                    sm.stop();
                    break;
            }
        }
    }   //runContinuous

}   //class FtcAutoK9PidLineFollow
