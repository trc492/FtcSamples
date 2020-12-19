/*
 * Copyright (c) 2020 Titan Robotics Club (http://www.titanrobotics.com)
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

package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.CmdPidDrive;
import org.firstinspires.ftc.teamcode.common.CmdPurePursuitDrive;
import org.firstinspires.ftc.teamcode.common.CmdTimedDrive;

import java.util.Date;
import java.util.Locale;

import ftclib.FtcChoiceMenu;
import ftclib.FtcMenu;
import ftclib.FtcOpMode;
import ftclib.FtcValueMenu;
import trclib.TrcRobot;

/**
 * This class contains the Autonomous Mode program.
 */
@Autonomous(name="FtcAutonomous", group="FtcAuto")
public class FtcAuto extends FtcOpMode
{
    public enum MatchType
    {
        PRACTICE,
        QUALIFICATION,
        SEMI_FINAL,
        FINAL
    }   //enum MatchType

    /**
     * This class stores the match info.
     */
    public class MatchInfo
    {
        Date matchDate;
        MatchType matchType;
        int matchNumber;

        public String toString()
        {
            return String.format(
                    Locale.US, "date=\"%s\" type=\"%s\" number=%d", matchDate, matchType, matchNumber);
        }   //toString
    }   //class MatchInfo

    public enum Alliance
    {
        RED_ALLIANCE,
        BLUE_ALLIANCE
    }   //enum Alliance

    public enum AutoStrategy
    {
        PURE_PURSUIT_DRIVE,
        PID_DRIVE,
        TIMED_DRIVE,
        DO_NOTHING
    }   //AutoStrategy

    /**
     * This class stores the autonomous menu choices.
     */
    public class AutoChoices
    {
        Alliance alliance = Alliance.RED_ALLIANCE;
        double startDelay = 0.0;
        AutoStrategy strategy = AutoStrategy.DO_NOTHING;
        double xTarget = 0.0;
        double yTarget = 0.0;
        double turnTarget = 0.0;
        double driveTime = 0.0;
        double drivePower = 0.0;

        @Override
        public String toString()
        {
            return String.format(Locale.US,
                    "alliance=\"%s\" " +
                    "startDelay=%.0f " +
                    "strategy=\"%s\" " +
                    "xTarget=%.1f " +
                    "yTarget=%.1f " +
                    "turnTarget=%.0f " +
                    "driveTime=%.0f " +
                    "drivePower=%.1f",
                    alliance, startDelay, strategy,
                    xTarget, yTarget, turnTarget, driveTime, drivePower);
        }   //toString
    }   //class AutoChoices

    private static final boolean logEvents = true;
    private static final boolean debugXPid = true;
    private static final boolean debugYPid = true;
    private static final boolean debugTurnPid = true;
    private static final String moduleName = "FtcAuto";

    private Robot robot;
    private final MatchInfo matchInfo = new MatchInfo();
    private final AutoChoices autoChoices = new AutoChoices();
    private TrcRobot.RobotCommand autoCommand = null;

    //
    // Implements FtcOpMode abstract method.
    //

    /**
     * This method is called to initialize the robot. In FTC, this is called when the "Init" button on the Driver
     * Station phone is pressed.
     */
    @SuppressLint("SdCardPath")
    @Override
    public void initRobot()
    {
        //
        // Open trace log.
        //
        if (Robot.Preferences.useTraceLog)
        {
            String filePrefix = String.format(Locale.US, "%s%02d", matchInfo.matchType, matchInfo.matchNumber);
            robot.globalTracer.openTraceLog("/sdcard/FIRST/tracelog", filePrefix);
        }
        //
        // Initializing robot objects.
        //
        robot = new Robot(TrcRobot.getRunMode());
        //
        // Choice menus.
        //
        doMatchInfoMenus();
        doAutoChoicesMenus();
        //
        // Strategies.
        //
        switch (autoChoices.strategy)
        {
            case PURE_PURSUIT_DRIVE:
                if (!Robot.Preferences.visionOnly)
                {
                    autoCommand = new CmdPurePursuitDrive(
                            robot.driveBase, robot.posPidCoeff, robot.turnPidCoeff, robot.velPidCoeff);
                }
                break;

            case PID_DRIVE:
                if (!Robot.Preferences.visionOnly)
                {
                    autoCommand = new CmdPidDrive(
                            robot.driveBase, robot.pidDrive, autoChoices.startDelay,
                            autoChoices.xTarget*12.0, autoChoices.yTarget*12.0, autoChoices.turnTarget,
                            autoChoices.drivePower, false, null);
                }
                break;

            case TIMED_DRIVE:
                if (!Robot.Preferences.visionOnly)
                {
                    autoCommand = new CmdTimedDrive(
                            robot.driveBase, autoChoices.startDelay, autoChoices.driveTime,
                            0.0, autoChoices.drivePower, 0.0);
                }
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

    /**
     * This method is called when the competition mode is about to start. In FTC, this is called when the "Play"
     * button on the Driver Station phone is pressed. Typically, you put code that will prepare the robot for
     * start of competition here such as resetting the encoders/sensors and enabling some sensors to start
     * sampling.
     *
     * @param prevMode specifies the previous RunMode it is coming from (always null for FTC).
     * @param nextMode specifies the next RunMode it is going into.
     */
    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        if (Robot.Preferences.useTraceLog)
        {
            robot.globalTracer.setTraceLogEnabled(true);
        }
        robot.globalTracer.traceInfo(moduleName, "***** Starting autonomous *****");
        robot.globalTracer.logInfo(moduleName, "MatchInfo", "%s", matchInfo);
        robot.globalTracer.logInfo(moduleName, "AutoChoices", "%s", autoChoices);

        robot.startMode(nextMode);

        if (robot.battery != null)
        {
            robot.battery.setEnabled(true);
        }

        if (autoChoices.strategy == AutoStrategy.PURE_PURSUIT_DRIVE)
        {
            ((CmdPurePursuitDrive)autoCommand).start(RobotInfo.PURE_PURSUIT_TEST_PATH);
        }

        robot.dashboard.clearDisplay();
    }   //startMode

    /**
     * This method is called when competition mode is about to end. Typically, you put code that will do clean
     * up here such as disabling the sampling of some sensors.
     *
     * @param prevMode specifies the previous RunMode it is coming from.
     * @param nextMode specifies the next RunMode it is going into (always null for FTC).
     */
    @Override
    public void stopMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        if (autoCommand != null)
        {
            autoCommand.cancel();
        }

        robot.stopMode(prevMode);

        if (robot.battery != null)
        {
            robot.battery.setEnabled(false);
        }

        printPerformanceMetrics(robot.globalTracer);

        if (robot.globalTracer.tracerLogIsOpened())
        {
            robot.globalTracer.closeTraceLog();
        }
    }   //stopMode

    /**
     * This method is called periodically as fast as the control system allows. Typically, you put code that requires
     * servicing at a higher frequency here. To make the robot as responsive and as accurate as possible especially
     * in autonomous mode, you will typically put that code here.
     *
     * @param elapsedTime specifies the elapsed time since the mode started.
     */
    @Override
    public void runContinuous(double elapsedTime)
    {
        if (autoCommand != null)
        {
            //
            // Run the autonomous command.
            //
            autoCommand.cmdPeriodic(elapsedTime);

            if (robot.pidDrive.isActive() && (logEvents || debugXPid || debugYPid || debugTurnPid))
            {
                if (robot.battery != null)
                {
                    robot.globalTracer.traceInfo("Battery", "Voltage=%5.2fV (%5.2fV)",
                            robot.battery.getVoltage(), robot.battery.getLowestVoltage());
                }

                if (logEvents)
                {
                    robot.globalTracer.logEvent(moduleName, "RobotPose", "pose=\"%s\"",
                            robot.driveBase.getFieldPosition().toString());
                }

                if (debugXPid && robot.encoderXPidCtrl != null)
                {
                    robot.encoderXPidCtrl.printPidInfo(robot.globalTracer);
                }

                if (debugYPid && robot.encoderYPidCtrl != null)
                {
                    robot.encoderYPidCtrl.printPidInfo(robot.globalTracer);
                }

                if (debugTurnPid && robot.gyroPidCtrl != null)
                {
                    robot.gyroPidCtrl.printPidInfo(robot.globalTracer);
                }
            }
        }
    }   //runContinuous

    /**
     * This method creates the MatchInfo menus, displays them and stores the choices.
     */
    private void doMatchInfoMenus()
    {
        //
        // Construct menus.
        //
        FtcChoiceMenu<MatchType> matchTypeMenu = new FtcChoiceMenu<>("Match type:", null);
        FtcValueMenu matchNumberMenu = new FtcValueMenu(
                "Match number:", matchTypeMenu, 1.0, 50.0, 1.0, 1.0,
                "%.0f");
        //
        // Populate choice menus.
        //
        matchTypeMenu.addChoice("Practice", MatchType.PRACTICE, true, matchNumberMenu);
        matchTypeMenu.addChoice("Qualification", MatchType.QUALIFICATION, false, matchNumberMenu);
        matchTypeMenu.addChoice("Semi-final", MatchType.SEMI_FINAL, false, matchNumberMenu);
        matchTypeMenu.addChoice("Final", MatchType.FINAL, false, matchNumberMenu);
        //
        // Traverse menus.
        //
        FtcMenu.walkMenuTree(matchTypeMenu);
        //
        // Fetch choices.
        //
        matchInfo.matchDate = new Date();
        matchInfo.matchType = matchTypeMenu.getCurrentChoiceObject();
        matchInfo.matchNumber = (int)matchNumberMenu.getCurrentValue();
    }   //doMatchInfoMenus

    /**
     * This method creates the autonomous menus, displays them and stores the choices.
     */
    private void doAutoChoicesMenus()
    {
        //
        // Construct menus.
        //
        FtcChoiceMenu<Alliance> allianceMenu = new FtcChoiceMenu<>("Alliance:", null);
        FtcValueMenu startDelayMenu = new FtcValueMenu(
                "Start delay time:", allianceMenu, 0.0, 30.0, 1.0, 0.0,
                " %.0f sec");
        FtcChoiceMenu<AutoStrategy> strategyMenu = new FtcChoiceMenu<>("Auto Strategies:", startDelayMenu);
        FtcValueMenu xTargetMenu = new FtcValueMenu(
                "xTarget:", strategyMenu, -12.0, 12.0, 0.5, 4.0,
                " %.1f ft");
        FtcValueMenu yTargetMenu = new FtcValueMenu(
                "yTarget:", xTargetMenu, -12.0, 12.0, 0.5, 4.0,
                " %.1f ft");
        FtcValueMenu turnTargetMenu = new FtcValueMenu(
                "turnTarget:", yTargetMenu, -180.0, 180.0, 5.0, 90.0,
                " %.0f ft");
        FtcValueMenu driveTimeMenu = new FtcValueMenu(
                "Drive time:", strategyMenu, 0.0, 30.0, 1.0, 5.0,
                " %.0f sec");
        FtcValueMenu drivePowerMenu = new FtcValueMenu(
                "Drive power:", strategyMenu, -1.0, 1.0, 0.1, 0.5,
                " %.1f");

        startDelayMenu.setChildMenu(strategyMenu);
        xTargetMenu.setChildMenu(yTargetMenu);
        yTargetMenu.setChildMenu(turnTargetMenu);
        turnTargetMenu.setChildMenu(drivePowerMenu);
        driveTimeMenu.setChildMenu(drivePowerMenu);
        //
        // Populate choice menus.
        //
        allianceMenu.addChoice("Red", Alliance.RED_ALLIANCE, true, startDelayMenu);
        allianceMenu.addChoice("Blue", Alliance.BLUE_ALLIANCE, false, startDelayMenu);

        strategyMenu.addChoice("Pure Pursuit Drive", AutoStrategy.PURE_PURSUIT_DRIVE, false);
        strategyMenu.addChoice("PID Drive", AutoStrategy.PID_DRIVE, false, xTargetMenu);
        strategyMenu.addChoice("Timed Drive", AutoStrategy.TIMED_DRIVE, false, driveTimeMenu);
        strategyMenu.addChoice("Do nothing", AutoStrategy.DO_NOTHING, true);
        //
        // Traverse menus.
        //
        FtcMenu.walkMenuTree(allianceMenu);
        //
        // Fetch choices.
        //
        autoChoices.alliance = allianceMenu.getCurrentChoiceObject();
        autoChoices.startDelay = startDelayMenu.getCurrentValue();
        autoChoices.strategy = strategyMenu.getCurrentChoiceObject();
        autoChoices.xTarget = xTargetMenu.getCurrentValue();
        autoChoices.yTarget = yTargetMenu.getCurrentValue();
        autoChoices.turnTarget = turnTargetMenu.getCurrentValue();
        autoChoices.driveTime = driveTimeMenu.getCurrentValue();
        autoChoices.drivePower = drivePowerMenu.getCurrentValue();
        //
        // Show choices.
        //
        robot.dashboard.displayPrintf(2, "Auto Choices: %s", autoChoices);
    }   //doAutoChoicesMenus

}   //class FtcAuto
