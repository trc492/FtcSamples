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
        event = new TrcEvent("seekIrEvent");
        sm = new TrcStateMachine("seekIrSM");
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void startMode()
    {
        robot.startMode(TrcRobot.RunMode.AUTO_MODE);
        //
        // Start state machine at state FIND_LINE.
        //
        sm.start(State.SEEK_IR);
    }   //startMode

    @Override
    public void stopMode()
    {
        robot.stopMode(TrcRobot.RunMode.AUTO_MODE);
    }   //stopMode

    @Override
    public void runContinuous(double elapsedTime)
    {
        robot.irDrivePidCtrl.displayPidInfo(1);
        robot.irTurnPidCtrl.displayPidInfo(3);
        //
        // Run state machine.
        //
        if (sm.isReady())
        {
            State state = (State)sm.getState();
            robot.dashboard.displayPrintf(5, "State: %s", state.toString());
            switch (state)
            {
                case SEEK_IR:
                    //
                    // Go towards IR beacon until IR strength reaches 0.8.
                    //
                    robot.pidSeekIr.setTarget(0.8, 0.0, false, event);
                    sm.addEvent(event);
                    sm.waitForEvents(State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done, stop!
                    //
                    sm.stop();
                    break;
            }
        }
    }   //runContinuous

}   //class FtcAutoK9PidSeekIr
