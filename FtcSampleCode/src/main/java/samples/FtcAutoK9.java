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
import com.qualcomm.robotcore.hardware.IrSeekerSensor;

import ftclib.FtcChoiceMenu;
import ftclib.FtcDcMotor;
import ftclib.FtcMRGyro;
import ftclib.FtcMRI2cColorSensor;
import ftclib.FtcMenu;
import ftclib.FtcOpMode;
import ftclib.FtcValueMenu;
import hallib.HalDashboard;
import trclib.TrcAnalogTrigger;
import trclib.TrcDriveBase;
import trclib.TrcEvent;
import trclib.TrcPidController;
import trclib.TrcPidDrive;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

@Autonomous(name="Auto: K9Bot Various Autonomous", group="Ftc3543Sample")
//@Disabled
public class FtcAutoK9 extends FtcOpMode implements TrcPidController.PidInput,
                                                    TrcAnalogTrigger.TriggerHandler,
                                                    FtcMenu.MenuButtons
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

    //
    // PID drive constants.
    //
    private static final double DRIVE_KP                = 0.03;
    private static final double DRIVE_KI                = 0.0;
    private static final double DRIVE_KD                = 0.0;
    private static final double DRIVE_KF                = 0.0;
    private static final double DRIVE_TOLERANCE         = 1.0;
    private static final double DRIVE_SETTLING          = 0.2;
    private static final double DRIVE_INCHES_PER_COUNT  = (104.0/7416.5);

    //
    // PID turn constants.
    //
    private static final double TURN_KP                 = 0.05;
    private static final double TURN_KI                 = 0.0;
    private static final double TURN_KD                 = 0.0;
    private static final double TURN_KF                 = 0.0;
    private static final double TURN_TOLERANCE          = 1.0;
    private static final double TURN_SETTLING           = 0.2;

    //
    // PID line follow constants.
    //
    private static final double COLOR_KP                = 0.1;
    private static final double COLOR_KI                = 0.0;
    private static final double COLOR_KD                = 0.0;
    private static final double COLOR_KF                = 0.0;
    private static final double COLOR_TOLERANCE         = 2.0;
    private static final double COLOR_SETTLING          = 0.2;
    private static final double COLOR_BLACK             = 0.0;
    private static final double COLOR_BLUE              = 3.0;
    private static final double COLOR_RED               = 10.0;
    private static final double COLOR_WHITE             = 16.0;
    private static final double COLOR_DARK_LEVEL        = 0.0;
    private static final double COLOR_WHITE_LEVEL       = 10.0;
    private static final double COLOR_LINE_EDGE_LEVEL   = ((COLOR_DARK_LEVEL + COLOR_WHITE_LEVEL)/2.0);
    private static final double COLOR_LINE_EDGE_DEADBAND= (COLOR_LINE_EDGE_LEVEL*0.25);

    //
    // PID IR drive constants.
    //
    private static final double IRDRIVE_KP              = 0.8;
    private static final double IRDRIVE_KI              = 0.0;
    private static final double IRDRIVE_KD              = 0.0;
    private static final double IRDRIVE_KF              = 0.0;
    private static final double IRDRIVE_TOLERANCE       = 0.1;
    private static final double IRDRIVE_SETTLING        = 0.2;

    //
    // PID IR turn constants.
    //
    private static final double IRTURN_KP               = 0.1;
    private static final double IRTURN_KI               = 0.0;
    private static final double IRTURN_KD               = 0.0;
    private static final double IRTURN_KF               = 0.0;
    private static final double IRTURN_TOLERANCE        = 1.0;
    private static final double IRTURN_SETTLING         = 0.2;

    private HalDashboard dashboard;

    //
    // Sensors.
    //
    private FtcMRGyro gyro;
    private FtcMRI2cColorSensor colorSensor;
    private IrSeekerSensor irSeeker;
    private double prevIrAngle = 0.0;
    private double prevIrStrength = 0.0;
    //
    // DriveBase subsystem.
    //
    private FtcDcMotor motorLeft;
    private FtcDcMotor motorRight;
    private TrcDriveBase driveBase;
    //
    // PID drive.
    //
    private TrcPidController drivePidCtrl;
    private TrcPidController turnPidCtrl;
    private TrcPidDrive pidDrive;
    //
    // PID line follow.
    //
    private TrcPidController colorPidCtrl;
    private TrcPidDrive pidLineFollow;
    private TrcAnalogTrigger colorTrigger;
    //
    // PID seek IR.
    //
    private TrcPidController irDrivePidCtrl;
    private TrcPidController irTurnPidCtrl;
    private TrcPidDrive pidSeekIr;
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

    private void doMenus()
    {
        //
        // Create the menus.
        //
        FtcValueMenu delayMenu = new FtcValueMenu("Delay time:", null, this,
                                                  0.0, 10.0, 1.0, 0.0, "%.0f sec");
        FtcChoiceMenu strategyMenu = new FtcChoiceMenu("Auto Strategies:", delayMenu, this);
        FtcValueMenu driveTimeMenu = new FtcValueMenu("Drive time:", strategyMenu, this,
                                                      0.0, 10.0, 1.0, 4.0, "%.0f sec");
        FtcValueMenu distanceMenu = new FtcValueMenu("Drive distance:", strategyMenu, this,
                                                     1.0, 8.0, 1.0, 1.0, "%.0f ft");
        FtcValueMenu degreesMenu = new FtcValueMenu("Turn degrees", strategyMenu, this,
                                                    -360.0, 360.0, 90.0, 360.0, "%.0f deg");
        FtcChoiceMenu allianceMenu = new FtcChoiceMenu("Alliance:", strategyMenu, this);

        strategyMenu.addChoice("Do nothing", AutoStrategy.DO_NOTHING);
        strategyMenu.addChoice("Timed drive", AutoStrategy.TIMED_DRIVE, driveTimeMenu);
        strategyMenu.addChoice("Drive forward", AutoStrategy.DRIVE_AND_TURN, distanceMenu);
        strategyMenu.addChoice("Follow line", AutoStrategy.FOLLOW_LINE, allianceMenu);
        strategyMenu.addChoice("Seek IR", AutoStrategy.SEEK_IR);

        allianceMenu.addChoice("Red", Alliance.RED_ALLIANCE);
        allianceMenu.addChoice("Blue", Alliance.BLUE_ALLIANCE);

        //
        // Walk the menu tree starting with the delay menu as the root
        // menu and get user choices.
        //
        FtcMenu.walkMenuTree(delayMenu);
        //
        // Set choices variables.
        //
        delay = delayMenu.getCurrentValue();
        autoStrategy = (AutoStrategy)strategyMenu.getCurrentChoiceObject();
        driveTime = driveTimeMenu.getCurrentValue();
        driveDistance = distanceMenu.getCurrentValue();
        turnDegrees = degreesMenu.getCurrentValue();
        alliance = (Alliance)allianceMenu.getCurrentChoiceObject();

        dashboard.displayPrintf(0, "Auto Strategy: %s", strategyMenu.getCurrentChoiceText());
    }   //doMenus

    //
    // Implements FtcOpMode abstract method.
    //

    @Override
    public void initRobot()
    {
        hardwareMap.logDevices();
        dashboard = getDashboard();
        //
        // Sensors.
        //
        gyro = new FtcMRGyro("gyro_sensor");
        gyro.calibrate();
        colorSensor = new FtcMRI2cColorSensor("colorSensor");
        irSeeker = hardwareMap.irSeekerSensor.get("irSeeker");
        //
        // DriveBase subsystem.
        //
        motorLeft = new FtcDcMotor("motor_1");
        motorRight = new FtcDcMotor("motor_2");
        motorLeft.setInverted(true);
        driveBase = new TrcDriveBase(motorLeft, motorRight, gyro);
        driveBase.setYPositionScale(DRIVE_INCHES_PER_COUNT);
        //
        // PID drive.
        //
        drivePidCtrl = new TrcPidController(
                "drivePid",
                DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KF,
                DRIVE_TOLERANCE, DRIVE_SETTLING, this);
        turnPidCtrl = new TrcPidController(
                "turnPid",
                TURN_KP, TURN_KI, TURN_KD, TURN_KF,
                TURN_TOLERANCE, TURN_SETTLING, this);
        pidDrive = new TrcPidDrive("pidDrive", driveBase, null, drivePidCtrl, turnPidCtrl);
        //
        // PID line follow.
        //
        colorPidCtrl = new TrcPidController(
                "lightPid",
                COLOR_KP, COLOR_KI, COLOR_KD, COLOR_KF,
                COLOR_TOLERANCE, COLOR_SETTLING, this);
        colorPidCtrl.setAbsoluteSetPoint(true);
        pidLineFollow = new TrcPidDrive(
                "lineFollow", driveBase, null, drivePidCtrl, colorPidCtrl);
        colorTrigger = new TrcAnalogTrigger("colorTrigger", colorSensor, 0,
                                            new double[]{COLOR_BLACK, COLOR_WHITE}, this);
        //
        // PID IR seeking.
        //
        irDrivePidCtrl = new TrcPidController(
                "irDrivePid",
                IRDRIVE_KP, IRDRIVE_KI, IRDRIVE_KD, IRDRIVE_KF,
                IRDRIVE_TOLERANCE, IRDRIVE_SETTLING, this);
        irDrivePidCtrl.setAbsoluteSetPoint(true);
        irTurnPidCtrl = new TrcPidController(
                "irTurnPid",
                IRTURN_KP, IRTURN_KI, IRTURN_KD, IRTURN_KF,
                IRTURN_TOLERANCE, IRTURN_SETTLING, this);
        irDrivePidCtrl.setAbsoluteSetPoint(true);
        pidSeekIr = new TrcPidDrive(
                "seekIr", driveBase, null, irDrivePidCtrl, irTurnPidCtrl);
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
        dashboard.clearDisplay();
        gyro.setEnabled(true);
        colorSensor.setLEDEnabled(true);
        driveBase.resetPosition();
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
        gyro.setEnabled(false);
        colorSensor.setLEDEnabled(false);
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
    // Implements TrcPidController.PidInput
    //

    @Override
    public double getInput(TrcPidController pidCtrl)
    {
        double input = 0.0;

        if (pidCtrl == drivePidCtrl)
        {
            input = driveBase.getYPosition();
        }
        else if (pidCtrl == turnPidCtrl)
        {
            input = driveBase.getHeading();
        }
        else if (pidCtrl == colorPidCtrl)
        {
            input = (double)(Integer)colorSensor.getWhiteValue().value;
            //
            // Give it a deadband to minimize fish tailing.
            //
            if (Math.abs(input - COLOR_LINE_EDGE_LEVEL) < COLOR_LINE_EDGE_DEADBAND)
            {
                input = COLOR_LINE_EDGE_LEVEL;
            }
        }
        else if (pidCtrl == irDrivePidCtrl)
        {
            //
            // Get the IR strength.
            //
            if (irSeeker.signalDetected())
            {
                input = irSeeker.getStrength();
                prevIrStrength = input;
            }
            else
            {
                input = prevIrStrength;
            }
        }
        else if (pidCtrl == irTurnPidCtrl)
        {
            //
            // Get the IR direction.
            //
            if (irSeeker.signalDetected())
            {
                input = irSeeker.getAngle();
                prevIrAngle = input;
            }
            else
            {
                input = prevIrAngle;
            }
        }

        return input;
    }   //getInput

    //
    // Implements TrcAnalogTrigger.TriggerHandler
    //

    @Override
    public void AnalogTriggerEvent(
            TrcAnalogTrigger analogTrigger, int zoneIndex, double zoneValue)
    {
        if (analogTrigger == colorTrigger && pidDrive.isEnabled())
        {
            //
            // Line is detected, interrupt PID drive.
            //
            if (zoneIndex > 0)
            {
                pidDrive.cancel();
            }
        }
    }   //AnalogTriggerEvent

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

    //
    // Autonomous strategies.
    //

    private void doTimedDrive(double delay, double driveTime)
    {
        if (sm.isReady())
        {
            TimedDriveState state = (TimedDriveState)sm.getState();
            dashboard.displayPrintf(1, "State: %s", state.toString());
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
                    driveBase.tankDrive(0.5, 0.5);
                    timer.set(driveTime, event);
                    sm.addEvent(event);
                    sm.waitForEvents(TimedDriveState.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done.
                    //
                    driveBase.stop();
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
            dashboard.displayPrintf(1, "State: %s", state.toString());
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
                    // Drive the given distance.
                    //
                    pidDrive.setTarget(distance, 0.0, false, event);
                    sm.addEvent(event);
                    sm.waitForEvents(DriveAndTurnState.TURN);
                    break;

                case TURN:
                    //
                    // Turn the given degrees.
                    //
                    pidDrive.setTarget(0.0, degrees, false, event);
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
            dashboard.displayPrintf(1, "State: %s", state.toString());
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
                    colorTrigger.setEnabled(true);
                    drivePidCtrl.setOutputRange(-0.5, 0.5);
                    pidDrive.setTarget(36.0, 0.0, false, event);
                    sm.addEvent(event);
                    sm.waitForEvents(LineFollowState.TURN_TO_LINE);
                    break;

                case TURN_TO_LINE:
                    //
                    // We have past the line slightly, so turn left or right 90 degree
                    // slowly to find the edge of the line. If the line is detected,
                    // PID turn will be interrupted.
                    //
                    turnPidCtrl.setOutputRange(-0.5, 0.5);
                    pidDrive.setTarget(
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
                    colorTrigger.setEnabled(false);
                    drivePidCtrl.setOutputRange(-0.3, 0.3);
                    colorPidCtrl.setOutputRange(-0.3, 0.3);
                    //
                    // Follow right edge if red alliance.
                    // Follow left edge if blue alliance.
                    //
                    colorPidCtrl.setInverted(alliance == Alliance.RED_ALLIANCE);
                    pidLineFollow.setTarget(60.0, COLOR_LINE_EDGE_DEADBAND, false, event);
                    sm.addEvent(event);
                    sm.waitForEvents(LineFollowState.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done, restore everything and stop!
                    //
                    drivePidCtrl.setOutputRange(-1.0, 1.0);
                    turnPidCtrl.setOutputRange(-1.0, 1.0);
                    colorPidCtrl.setOutputRange(-1.0, 1.0);
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
            dashboard.displayPrintf(1, "State: %s", state.toString());
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
                    pidSeekIr.setTarget(0.8, 0.0, false, event);
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

}   //class FtcAutoK9
