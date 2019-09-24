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
import trclib.TrcRobot;

@Autonomous(name="Auto: K9Bot Various Autonomous", group="FtcAutoSamples")
@Disabled
public class FtcAutoK9 extends FtcOpMode
{
    public enum Alliance
    {
        RED_ALLIANCE,
        BLUE_ALLIANCE
    }   //enum Alliance

    private enum AutoStrategy
    {
        TIMED_DRIVE,
        DRIVE_AND_TURN,
        FOLLOW_LINE,
        SEEK_IR,
        DO_NOTHING
    }   //enum AutoStrategy

    private K9Robot robot;
    private TrcRobot.RobotCommand autoCommand;
    //
    // Menu choices.
    //
    private double delay = 0.0;
    private AutoStrategy strategy = AutoStrategy.DO_NOTHING;
    private double driveTime = 0.0;
    private double drivePower = 0.0;
    private double driveDistance = 0.0;
    private double turnDegrees = 0.0;
    private Alliance alliance = Alliance.RED_ALLIANCE;

    //
    // Implements FtcOpMode abstract methods.
    //

    @Override
    public void initRobot()
    {
        //
        // Create the robot.
        //
        robot = new K9Robot();
        //
        // Choice menus.
        //
        doMenus();
        //
        // Creates the robot command for the chosen strategy.
        //
        switch (strategy)
        {
            case TIMED_DRIVE:
                autoCommand = new CmdTimedDrive(robot, delay, driveTime, 0.0, drivePower, 0.0);
                break;

            case DRIVE_AND_TURN:
                autoCommand = new CmdDriveAndTurn(robot, delay, driveDistance*12.0, turnDegrees);
                break;

            case FOLLOW_LINE:
                autoCommand = new CmdFollowLine(robot, delay, alliance);
                break;

            case SEEK_IR:
                autoCommand = new CmdSeekIR(robot, delay, alliance);
                break;

            case DO_NOTHING:
            default:
                autoCommand = null;
                break;
        }
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        robot.startMode();
    }   //startMode

    @Override
    public void stopMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        robot.stopMode();
    }   //stopMode

    @Override
    public void runContinuous(double elapsedTime)
    {
        if (autoCommand != null)
        {
            autoCommand.cmdPeriodic(elapsedTime);
        }
    }   //runContinuous

    private void doMenus()
    {
        //
        // Create the menus.
        //
        FtcValueMenu delayMenu = new FtcValueMenu(
                "Delay time:", null, 0.0, 10.0, 1.0, 0.0,
                "%.0f sec");
        FtcChoiceMenu<AutoStrategy> strategyMenu = new FtcChoiceMenu<>("Auto Strategies:", delayMenu);
        FtcValueMenu driveTimeMenu = new FtcValueMenu(
                "Drive time:", strategyMenu, 0.0, 10.0, 1.0, 4.0,
                "%.0f sec");
        FtcValueMenu drivePowerMenu = new FtcValueMenu(
                "Drive power:", driveTimeMenu, -1.0, 1.0, 0.1, 0.5,
                "%.1f");
        FtcValueMenu distanceMenu = new FtcValueMenu(
                "Drive distance:", strategyMenu, 1.0, 8.0, 1.0, 1.0,
                "%.0f ft");
        FtcValueMenu degreesMenu = new FtcValueMenu(
                "Turn degrees", strategyMenu, -360.0, 360.0, 90.0, 360.0,
                "%.0f deg");
        FtcChoiceMenu<Alliance> allianceMenu = new FtcChoiceMenu<>("Alliance:", strategyMenu);

        delayMenu.setChildMenu(strategyMenu);
        driveTimeMenu.setChildMenu(drivePowerMenu);

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
        FtcMenu.walkMenuTree(delayMenu);
        //
        // Set choices variables.
        //
        delay = delayMenu.getCurrentValue();
        strategy = strategyMenu.getCurrentChoiceObject();
        driveTime = driveTimeMenu.getCurrentValue();
        drivePower = drivePowerMenu.getCurrentValue();
        driveDistance = distanceMenu.getCurrentValue();
        turnDegrees = degreesMenu.getCurrentValue();
        alliance = allianceMenu.getCurrentChoiceObject();

        robot.dashboard.displayPrintf(0, "Auto Strategy: %s", strategyMenu.getCurrentChoiceText());
    }   //doMenus

}   //class FtcAutoK9
