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

import ftclib.FtcChoiceMenu;
import ftclib.FtcMenu;
import ftclib.FtcOpMode;
import ftclib.FtcValueMenu;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

@Autonomous(name="Auto: K9Bot Various Autonomous", group="3543AutoSamples")
@Disabled
public class FtcAutoK9 extends FtcOpMode implements FtcMenu.MenuButtons
{
    private enum AutoStrategy
    {
        DO_NOTHING,
        TIMED_DRIVE,
        DRIVE_AND_TURN,
        FOLLOW_LINE,
        SEEK_IR
    }   //enum AutoStrategy

    private enum Alliance
    {
        RED_ALLIANCE,
        BLUE_ALLIANCE
    }   //enum Alliance

    private enum TimedDriveState
    {
        DELAY,
        DRIVE,
        DONE
    }   //enum TimedDriveState

    private enum DriveAndTurnState
    {
        DELAY,
        DRIVE,
        TURN,
        DONE
    }   //enum DriveAndTurnState

    private enum LineFollowState
    {
        DELAY,
        FIND_LINE,
        TURN_TO_LINE,
        FOLLOW_LINE,
        DONE
    }   //enum LineFollowState

    private enum SeekIrState
    {
        DELAY,
        SEEK_IR,
        DONE
    }   //enum SeekIrState

    private K9Robot robot;
    //
    // State machine.
    //
    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine sm;
    //
    // Menu choices.
    //
    private double delay = 0.0;
    private AutoStrategy autoStrategy = AutoStrategy.DO_NOTHING;
    private double driveTime = 0.0;
    private double driveDistance = 0.0;
    private double turnDegrees = 0.0;
    private Alliance alliance = Alliance.RED_ALLIANCE;

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
        event = new TrcEvent("autoEvent");
        timer = new TrcTimer("autoTimer");
        sm = new TrcStateMachine("autoSM");
        //
        // Choice menus.
        //
        doMenus();
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void startMode()
    {
        robot.startMode(TrcRobot.RunMode.AUTO_MODE);
        //
        // Start the state machine according to the auto strategy.
        //
        switch (autoStrategy)
        {
            case TIMED_DRIVE:
                sm.start(TimedDriveState.DELAY);
                break;

            case DRIVE_AND_TURN:
                sm.start(DriveAndTurnState.DELAY);
                break;

            case FOLLOW_LINE:
                sm.start(LineFollowState.DELAY);
                break;

            case SEEK_IR:
                sm.start(SeekIrState.DELAY);
                break;
        }
    }   //startMode

    @Override
    public void stopMode()
    {
        robot.startMode(TrcRobot.RunMode.AUTO_MODE);
    }   //stopMode

    @Override
    public void runContinuous(double elapsedTime)
    {
        switch (autoStrategy)
        {
            case TIMED_DRIVE:
                doTimedDrive(delay, driveTime);
                break;

            case DRIVE_AND_TURN:
                doDriveAndTurn(delay, driveDistance, turnDegrees);
                break;

            case FOLLOW_LINE:
                doLineFollow(delay, alliance);
                break;

            case SEEK_IR:
                doIrSeek(delay);
                break;
        }
    }   //runContinuous

    //
    // Autonomous strategies.
    //

    private void doTimedDrive(double delay, double driveTime)
    {
        if (sm.isReady())
        {
            TimedDriveState state = (TimedDriveState)sm.getState();
            robot.dashboard.displayPrintf(1, "State: %s", state.toString());
            switch (state)
            {
                case DELAY:
                    //
                    // Do delay if necessary.
                    //
                    if (delay == 0.0)
                    {
                        sm.setState(TimedDriveState.DRIVE);
                    }
                    else
                    {
                        timer.set(delay, event);
                        sm.addEvent(event);
                        sm.waitForEvents(TimedDriveState.DRIVE);
                    }
                    break;

                case DRIVE:
                    //
                    // Drive the given distance.
                    //
                    robot.driveBase.tankDrive(0.5, 0.5);
                    timer.set(driveTime, event);
                    sm.addEvent(event);
                    sm.waitForEvents(TimedDriveState.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done.
                    //
                    robot.driveBase.stop();
                    sm.stop();
                    break;
            }
        }
    }   //doTimeDrive

    private void doDriveAndTurn(double delay, double distance, double degrees)
    {
        if (sm.isReady())
        {
            DriveAndTurnState state = (DriveAndTurnState)sm.getState();
            robot.dashboard.displayPrintf(1, "State: %s", state.toString());
            switch (state)
            {
                case DELAY:
                    //
                    // Do delay if necessary.
                    //
                    if (delay == 0.0)
                    {
                        sm.setState(DriveAndTurnState.DRIVE);
                    }
                    else
                    {
                        timer.set(delay, event);
                        sm.addEvent(event);
                        sm.waitForEvents(DriveAndTurnState.DRIVE);
                    }
                    break;

                case DRIVE:
                    //
                    // Drive the given distance in feet.
                    //
                    robot.pidDrive.setTarget(distance*12.0, 0.0, false, event);
                    sm.addEvent(event);
                    sm.waitForEvents(DriveAndTurnState.TURN);
                    break;

                case TURN:
                    //
                    // Turn the given degrees.
                    //
                    robot.pidDrive.setTarget(0.0, degrees, false, event);
                    sm.addEvent(event);
                    sm.waitForEvents(DriveAndTurnState.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done.
                    //
                    sm.stop();
                    break;
            }
        }
    }   //doDriveAndTurn

    private void doLineFollow(double delay, Alliance alliance)
    {
        if (sm.isReady())
        {
            LineFollowState state = (LineFollowState)sm.getState();
            robot.dashboard.displayPrintf(1, "State: %s", state.toString());
            switch (state)
            {
                case DELAY:
                    //
                    // Do delay if necessary.
                    //
                    if (delay == 0.0)
                    {
                        sm.setState(LineFollowState.FIND_LINE);
                    }
                    else
                    {
                        timer.set(delay, event);
                        sm.addEvent(event);
                        sm.waitForEvents(LineFollowState.FIND_LINE);
                    }
                    break;

                case FIND_LINE:
                    //
                    // Go forward slowly for 3 ft to find the line.
                    // If line is detected, PID drive will be interrupted.
                    //
                    robot.colorTrigger.setEnabled(true);
                    robot.drivePidCtrl.setOutputRange(-0.5, 0.5);
                    robot.pidDrive.setTarget(36.0, 0.0, false, event);
                    sm.addEvent(event);
                    sm.waitForEvents(LineFollowState.TURN_TO_LINE);
                    break;

                case TURN_TO_LINE:
                    //
                    // We have past the line slightly, so turn left or right 90 degree
                    // slowly to find the edge of the line. If the line is detected,
                    // PID turn will be interrupted.
                    //
                    robot.turnPidCtrl.setOutputRange(-0.5, 0.5);
                    robot.pidDrive.setTarget(
                            0.0,
                            alliance == FtcAutoK9.Alliance.RED_ALLIANCE? -90.0: 90.0,
                            false, event);
                    sm.addEvent(event);
                    sm.waitForEvents(LineFollowState.FOLLOW_LINE);
                    break;

                case FOLLOW_LINE:
                    //
                    // Follow the line for 5 ft.
                    //
                    robot.colorTrigger.setEnabled(false);
                    robot.drivePidCtrl.setOutputRange(-0.3, 0.3);
                    robot.colorPidCtrl.setOutputRange(-0.3, 0.3);
                    //
                    // Follow right edge if red alliance.
                    // Follow left edge if blue alliance.
                    //
                    robot.colorPidCtrl.setInverted(alliance == Alliance.RED_ALLIANCE);
                    robot.pidLineFollow.setTarget(60.0, K9Robot.COLOR_LINE_EDGE_DEADBAND, false, event);
                    sm.addEvent(event);
                    sm.waitForEvents(LineFollowState.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done, restore everything and stop!
                    //
                    robot.drivePidCtrl.setOutputRange(-1.0, 1.0);
                    robot.turnPidCtrl.setOutputRange(-1.0, 1.0);
                    robot.colorPidCtrl.setOutputRange(-1.0, 1.0);
                    sm.stop();
                    break;
            }
        }
    }   //doLineFollow

    private void doIrSeek(double delay)
    {
        if (sm.isReady())
        {
            SeekIrState state = (SeekIrState)sm.getState();
            robot.dashboard.displayPrintf(1, "State: %s", state.toString());
            switch (state)
            {
                case DELAY:
                    //
                    // Do delay if necessary.
                    //
                    if (delay == 0.0)
                    {
                        sm.setState(SeekIrState.SEEK_IR);
                    }
                    else
                    {
                        timer.set(delay, event);
                        sm.addEvent(event);
                        sm.waitForEvents(SeekIrState.SEEK_IR);
                    }
                    break;

                case SEEK_IR:
                    //
                    // Go towards IR beacon until IR strength reaches 0.8.
                    //
                    robot.pidSeekIr.setTarget(0.8, 0.0, false, event);
                    sm.addEvent(event);
                    sm.waitForEvents(SeekIrState.DONE);
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
    }   //doIrSeek

    private void doMenus()
    {
        //
        // Create the menus.
        //
        FtcValueMenu delayMenu = new FtcValueMenu("Delay time:", null, this,
                0.0, 10.0, 1.0, 0.0, "%.0f sec");
        FtcChoiceMenu<AutoStrategy> strategyMenu = new FtcChoiceMenu<>("Auto Strategies:", delayMenu, this);
        FtcValueMenu driveTimeMenu = new FtcValueMenu("Drive time:", strategyMenu, this,
                0.0, 10.0, 1.0, 4.0, "%.0f sec");
        FtcValueMenu distanceMenu = new FtcValueMenu("Drive distance:", strategyMenu, this,
                1.0, 8.0, 1.0, 1.0, "%.0f ft");
        FtcValueMenu degreesMenu = new FtcValueMenu("Turn degrees", strategyMenu, this,
                -360.0, 360.0, 90.0, 360.0, "%.0f deg");
        FtcChoiceMenu<Alliance> allianceMenu = new FtcChoiceMenu<>("Alliance:", strategyMenu, this);

        delayMenu.setChildMenu(strategyMenu);

        strategyMenu.addChoice("Do nothing", AutoStrategy.DO_NOTHING, true);
        strategyMenu.addChoice("Timed drive", AutoStrategy.TIMED_DRIVE, false, driveTimeMenu);
        strategyMenu.addChoice("Drive forward", AutoStrategy.DRIVE_AND_TURN, false, distanceMenu);
        strategyMenu.addChoice("Follow line", AutoStrategy.FOLLOW_LINE, false, allianceMenu);
        strategyMenu.addChoice("Seek IR", AutoStrategy.SEEK_IR, false);

        allianceMenu.addChoice("Red", Alliance.RED_ALLIANCE, true);
        allianceMenu.addChoice("Blue", Alliance.BLUE_ALLIANCE, false);

        //
        // Walk the menu tree starting with the delay menu as the root
        // menu and get user choices.
        //
        FtcMenu.walkMenuTree(delayMenu, this);
        //
        // Set choices variables.
        //
        delay = delayMenu.getCurrentValue();
        autoStrategy = (AutoStrategy)strategyMenu.getCurrentChoiceObject();
        driveTime = driveTimeMenu.getCurrentValue();
        driveDistance = distanceMenu.getCurrentValue();
        turnDegrees = degreesMenu.getCurrentValue();
        alliance = (Alliance)allianceMenu.getCurrentChoiceObject();

        robot.dashboard.displayPrintf(0, "Auto Strategy: %s", strategyMenu.getCurrentChoiceText());
    }   //doMenus

    //
    // Implements FtcMenu.MenuButtons interface.
    //

    @Override
    public boolean isMenuUpButton()
    {
        return gamepad1.dpad_up;
    }   //isMenuUpButton

    @Override
    public boolean isMenuDownButton()
    {
        return gamepad1.dpad_down;
    }   //isMenuDownButton

    @Override
    public boolean isMenuEnterButton()
    {
        return gamepad1.a;
    }   //isMenuEnterButton

    @Override
    public boolean isMenuBackButton()
    {
        return gamepad1.dpad_left;
    }   //isMenuBackButton

}   //class FtcAutoK9
