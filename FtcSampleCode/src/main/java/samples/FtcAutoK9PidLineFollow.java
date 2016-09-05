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

import ftclib.FtcDcMotor;
import ftclib.FtcMRGyro;
import ftclib.FtcOpMode;
import ftclib.FtcOpticalDistanceSensor;
import hallib.HalDashboard;
import trclib.TrcAnalogTrigger;
import trclib.TrcDriveBase;
import trclib.TrcEvent;
import trclib.TrcPidController;
import trclib.TrcPidDrive;
import trclib.TrcStateMachine;

@Autonomous(name="Auto: K9Bot PID Line Following", group="Ftc3543Sample")
//@Disabled
public class FtcAutoK9PidLineFollow extends FtcOpMode implements TrcPidController.PidInput,
                                                                 TrcAnalogTrigger.TriggerHandler
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
    private static final double LIGHT_KP                = 0.02;
    private static final double LIGHT_KI                = 0.0;
    private static final double LIGHT_KD                = 0.0;
    private static final double LIGHT_KF                = 0.0;
    private static final double LIGHT_TOLERANCE         = 5.0;
    private static final double LIGHT_SETTLING          = 0.2;

    private static final double LIGHT_DARK_LEVEL        = 10.0;
    private static final double LIGHT_WHITE_LEVEL       = 60.0;
    private static final double LIGHT_THRESHOLD         = ((LIGHT_DARK_LEVEL + LIGHT_WHITE_LEVEL)/2.0);

    private HalDashboard dashboard;
    //
    // Sensors.
    //
    private FtcMRGyro gyro;
    private FtcOpticalDistanceSensor lightSensor;
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
    private TrcPidController lightPidCtrl;
    private TrcPidDrive lineFollowDrive;

    private TrcAnalogTrigger lightTrigger;
    //
    // Event and state machine.
    //
    private TrcEvent event;
    private TrcStateMachine sm;

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
        lightSensor = new FtcOpticalDistanceSensor("light_sensor");
        //
        // DriveBase subsystem.
        //
        motorLeft = new FtcDcMotor("motor_1");
        motorRight = new FtcDcMotor("motor_2");
        motorLeft.setInverted(true);
        driveBase = new TrcDriveBase(motorLeft, motorRight, gyro);
        driveBase.setYPositionScale(DRIVE_INCHES_PER_COUNT);
        driveBase.resetPosition();
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
        lightPidCtrl = new TrcPidController(
                "lightPid",
                LIGHT_KP, LIGHT_KI, LIGHT_KD, LIGHT_KF,
                LIGHT_TOLERANCE, LIGHT_SETTLING, this);
        lightPidCtrl.setAbsoluteSetPoint(true);
        lineFollowDrive = new TrcPidDrive(
                "lineFollow", driveBase, null, drivePidCtrl, lightPidCtrl);

        lightTrigger = new TrcAnalogTrigger("lightTrigger", lightSensor, 0,
                                            new double[]{LIGHT_DARK_LEVEL, LIGHT_WHITE_LEVEL},
                                            this);
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
    public void startMode()
    {
        dashboard.clearDisplay();
        gyro.setEnabled(true);
        lightSensor.setEnabled(true);
        driveBase.resetPosition();
        //
        // Start state machine at state FIND_LINE.
        //
        sm.start(State.FIND_LINE);
    }   //startMode

    @Override
    public void stopMode()
    {
        gyro.setEnabled(false);
        lightSensor.setEnabled(false);
    }   //stopMode

    @Override
    public void runContinuous(double elapsedTime)
    {
        drivePidCtrl.displayPidInfo(1);
        turnPidCtrl.displayPidInfo(3);
        lightPidCtrl.displayPidInfo(5);
        //
        // Run state machine.
        //
        if (sm.isReady())
        {
            State state = (State)sm.getState();
            dashboard.displayPrintf(7, "State: %s", state.toString());
            switch (state)
            {
                case FIND_LINE:
                    //
                    // Go forward slowly for 3 ft to find the line.
                    // If line is detected, PID drive will be interrupted.
                    //
                    lightTrigger.setEnabled(true);
                    drivePidCtrl.setOutputRange(-0.5, 0.5);
                    pidDrive.setTarget(36.0, 0.0, false, event);
                    sm.addEvent(event);
                    sm.waitForEvents(State.TURN_TO_LINE);
                    break;

                case TURN_TO_LINE:
                    //
                    // We have past the line slightly, so turn right 90 degree
                    // slowly to find the edge of the line. If the line is detected,
                    // PID turn will be interrupted.
                    //
                    turnPidCtrl.setOutputRange(-0.5, 0.5);
                    pidDrive.setTarget(0.0, 90.0, false, event);
                    sm.addEvent(event);
                    sm.waitForEvents(State.FOLLOW_LINE);
                    break;

                case FOLLOW_LINE:
                    //
                    // Follow the line for 5 ft.
                    //
                    lightTrigger.setEnabled(false);
                    drivePidCtrl.setOutputRange(-0.3, 0.3);
                    lightPidCtrl.setOutputRange(-0.3, 0.3);
                    lineFollowDrive.setTarget(60.0, LIGHT_THRESHOLD, false, event);
                    sm.addEvent(event);
                    sm.waitForEvents(State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done, restore everything and stop!
                    //
                    drivePidCtrl.setOutputRange(-1.0, 1.0);
                    turnPidCtrl.setOutputRange(-1.0, 1.0);
                    lightPidCtrl.setOutputRange(-1.0, 1.0);
                    sm.stop();
                    break;
            }
        }
    }   //runContinuous

    //
    // Implements TrcPidController.PidInput interface.
    //

    @Override
    public double getInput(TrcPidController pidCtrl)
    {
        double input = 0.0;

        if (pidCtrl == drivePidCtrl)
        {
            //
            // Get the robot's Y position from the DriveBase.
            //
            input = driveBase.getYPosition();
        }
        else if (pidCtrl == turnPidCtrl)
        {
            //
            // Get the robot's heading from the DriveBase.
            //
            input = driveBase.getHeading();
        }
        else if (pidCtrl == lightPidCtrl)
        {
            //
            // Get the light sensor reading.
            //
            input = (Double)lightSensor.getData(0).value;
        }

        return input;
    }   //getInput

    //
    // Implements TrcAnalogTrigger.TriggerHandler interface
    //

    @Override
    public void AnalogTriggerEvent(
            TrcAnalogTrigger analogTrigger, int zoneIndex, double zoneValue)
    {
        if (analogTrigger == lightTrigger && pidDrive.isEnabled())
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

}   //class FtcAutoK9PidLineFollow
