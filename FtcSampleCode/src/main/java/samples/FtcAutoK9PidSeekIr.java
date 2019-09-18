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
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;

@Autonomous(name="Auto: K9Bot PID Seek IR", group="3543AutoSamples")
@Disabled
public class FtcAutoK9PidSeekIr extends FtcOpMode
{
    //
    // State machine states.
    //
    private enum State
    {
        SEEK_IR,
        DONE
    }

    private K9Robot robot;
    //
    // Event and state machine.
    //
    private TrcEvent event;
    private TrcStateMachine<State> sm;

    //
    // Implements FtcOpMode abstract methods.
    //

    @Override
    public void initRobot()
    {
        robot = new K9Robot();
        //
        // State machine.
        //
        event = new TrcEvent("seekIrEvent");
        sm = new TrcStateMachine("seekIrSM");
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        robot.startMode();
        //
        // Start state machine at state FIND_LINE.
        //
        sm.start(State.SEEK_IR);
    }   //startMode

    @Override
    public void stopMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        robot.stopMode();
    }   //stopMode

    @Override
    public void runContinuous(double elapsedTime)
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
                case SEEK_IR:
                    //
                    // Go towards IR beacon until IR strength reaches 0.8.
                    //
                    robot.pidSeekIr.setTarget(0.8, 0.0, false, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done, stop!
                    //
                    sm.stop();
                    break;
            }

            robot.irDrivePidCtrl.displayPidInfo(8);
            robot.irTurnPidCtrl.displayPidInfo(10);
        }
    }   //runContinuous

}   //class FtcAutoK9PidSeekIr
