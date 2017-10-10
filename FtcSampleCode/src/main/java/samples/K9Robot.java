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

import android.widget.TextView;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import ftclib.FtcDcMotor;
import ftclib.FtcMRColorSensor;
import ftclib.FtcMRGyro;
import ftclib.FtcMRI2cColorSensor;
import ftclib.FtcOpMode;
import ftclib.FtcOpticalDistanceSensor;
import ftclib.FtcServo;
import hallib.HalDashboard;
import trclib.TrcAnalogTrigger;
import trclib.TrcDriveBase;
import trclib.TrcEnhancedServo;
import trclib.TrcPidController;
import trclib.TrcPidDrive;
import trclib.TrcRobot;

public class K9Robot implements TrcPidController.PidInput, TrcAnalogTrigger.TriggerHandler
{
    //
    // PID drive constants.
    //
    private static final double DRIVE_KP                = 0.03;
    private static final double DRIVE_KI                = 0.0;
    private static final double DRIVE_KD                = 0.0;
    private static final double DRIVE_TOLERANCE         = 1.0;
    private static final double DRIVE_INCHES_PER_COUNT  = (104.0/7416.5);
    //
    // PID turn constants.
    //
    private static final double TURN_KP                 = 0.05;
    private static final double TURN_KI                 = 0.0;
    private static final double TURN_KD                 = 0.0;
    private static final double TURN_TOLERANCE          = 1.0;
    //
    // PID line follow constants.
    //
    private static final double COLOR_KP                = 0.1;
    private static final double COLOR_KI                = 0.0;
    private static final double COLOR_KD                = 0.0;
    private static final double COLOR_TOLERANCE         = 2.0;
    private static final double COLOR_BLACK             = 0.0;
    private static final double COLOR_BLUE              = 3.0;
    private static final double COLOR_RED               = 10.0;
    private static final double COLOR_WHITE             = 16.0;
    private static final double COLOR_DARK_LEVEL        = 0.0;
    private static final double COLOR_WHITE_LEVEL       = 10.0;
    private static final double COLOR_LINE_EDGE_LEVEL   = ((COLOR_DARK_LEVEL + COLOR_WHITE_LEVEL)/2.0);
    public static final double COLOR_LINE_EDGE_DEADBAND = (COLOR_LINE_EDGE_LEVEL*0.25);
    //
    // PID line follow constants.
    //
    private static final double LIGHT_KP                = 0.02;
    private static final double LIGHT_KI                = 0.0;
    private static final double LIGHT_KD                = 0.0;
    private static final double LIGHT_TOLERANCE         = 5.0;

    private static final double LIGHT_DARK_LEVEL        = 10.0;
    private static final double LIGHT_WHITE_LEVEL       = 60.0;
    public static final double LIGHT_THRESHOLD          = ((LIGHT_DARK_LEVEL + LIGHT_WHITE_LEVEL)/2.0);
    //
    // PID IR drive constants.
    //
    private static final double IRDRIVE_KP              = 0.8;
    private static final double IRDRIVE_KI              = 0.0;
    private static final double IRDRIVE_KD              = 0.0;
    private static final double IRDRIVE_TOLERANCE       = 0.1;
    //
    // PID IR turn constants.
    //
    private static final double IRTURN_KP               = 0.1;
    private static final double IRTURN_KI               = 0.0;
    private static final double IRTURN_KD               = 0.0;
    private static final double IRTURN_TOLERANCE        = 1.0;

    public static final double ARM_MIN_RANGE            = 0.2;
    public static final double ARM_MAX_RANGE            = 0.9;
    public static final double CLAW_MIN_RANGE           = 0.2;
    public static final double CLAW_MAX_RANGE           = 0.7;
    public static final double SERVO_STEPRATE           = 2.0;

    //
    // Global objects.
    //
    public FtcOpMode opmode;
    public HardwareMap hardwareMap;
    public HalDashboard dashboard;
    public FtcRobotControllerActivity activity;
    //
    // Sensors.
    //
    public FtcMRGyro gyro;
    public FtcMRColorSensor colorSensor;
    public FtcOpticalDistanceSensor lightSensor;
    public IrSeekerSensor irSeeker;
    public double prevIrAngle = 0.0;
    public double prevIrStrength = 0.0;
    //
    // DriveBase subsystem.
    //
    public FtcDcMotor motorLeft;
    public FtcDcMotor motorRight;
    public TrcDriveBase driveBase;
    //
    // PID drive.
    //
    public TrcPidController drivePidCtrl;
    public TrcPidController turnPidCtrl;
    public TrcPidDrive pidDrive;
    //
    // PID line follow using color sensor.
    //
    public TrcPidController colorPidCtrl;
    public TrcPidDrive pidLineFollow;
    public TrcAnalogTrigger<FtcMRColorSensor.DataType> colorTrigger;
    //
    // PID line follow using Optical Distance sensor.
    //
    public TrcPidController lightPidCtrl;
    public TrcPidDrive lineFollowDrive;
    public TrcAnalogTrigger<FtcOpticalDistanceSensor.DataType> lightTrigger;
    //
    // PID seek IR.
    //
    public TrcPidController irDrivePidCtrl;
    public TrcPidController irTurnPidCtrl;
    public TrcPidDrive pidSeekIr;
    //
    // Other subsystems.
    //
    public FtcServo armServo;
    public TrcEnhancedServo arm;
    public FtcServo clawServo;
    public TrcEnhancedServo claw;

    public K9Robot(TrcRobot.RunMode runMode)
    {
        //
        // Initialize global objects.
        //
        opmode = FtcOpMode.getInstance();
        hardwareMap = opmode.hardwareMap;
        dashboard = HalDashboard.getInstance();
        activity = (FtcRobotControllerActivity) hardwareMap.appContext;
        hardwareMap.logDevices();
        dashboard.setTextView((TextView) activity.findViewById(FtcSampleCode.R.id.textOpMode));
        //
        // Initialize sensors.
        //
        gyro = new FtcMRGyro("gyro_sensor");
        gyro.calibrate();
        colorSensor = new FtcMRColorSensor("colorSensor");
        lightSensor = new FtcOpticalDistanceSensor("light_sensor");
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
                new TrcPidController.PidCoefficients(DRIVE_KP, DRIVE_KI, DRIVE_KD),
                DRIVE_TOLERANCE, this);
        turnPidCtrl = new TrcPidController(
                "turnPid",
                new TrcPidController.PidCoefficients(TURN_KP, TURN_KI, TURN_KD),
                TURN_TOLERANCE, this);
        pidDrive = new TrcPidDrive("pidDrive", driveBase, null, drivePidCtrl, turnPidCtrl);
        //
        // PID line follow using color sensor.
        //
        colorPidCtrl = new TrcPidController(
                "lightPid",
                new TrcPidController.PidCoefficients(COLOR_KP, COLOR_KI, COLOR_KD),
                COLOR_TOLERANCE, this);
        colorPidCtrl.setAbsoluteSetPoint(true);
        pidLineFollow = new TrcPidDrive(
                "lineFollow", driveBase, null, drivePidCtrl, colorPidCtrl);
        colorTrigger = new TrcAnalogTrigger<>(
                "colorTrigger", colorSensor, 0, FtcMRColorSensor.DataType.WHITE,
                new double[]{COLOR_BLACK, COLOR_WHITE}, this);
        //
        // PID line follow using Optical Distance sensor.
        //
        lightPidCtrl = new TrcPidController(
                "lightPid",
                new TrcPidController.PidCoefficients(LIGHT_KP, LIGHT_KI, LIGHT_KD),
                LIGHT_TOLERANCE, this);
        lightPidCtrl.setAbsoluteSetPoint(true);
        lineFollowDrive = new TrcPidDrive(
                "lineFollow", driveBase, null, drivePidCtrl, lightPidCtrl);

        lightTrigger = new TrcAnalogTrigger<>(
                "lightTrigger", lightSensor, 0, FtcOpticalDistanceSensor.DataType.RAW_LIGHT_DETECTED,
                new double[]{LIGHT_DARK_LEVEL, LIGHT_WHITE_LEVEL}, this);
        //
        // PID IR seeking.
        //
        irDrivePidCtrl = new TrcPidController(
                "irDrivePid",
                new TrcPidController.PidCoefficients(IRDRIVE_KP, IRDRIVE_KI, IRDRIVE_KD),
                IRDRIVE_TOLERANCE, this);
        irDrivePidCtrl.setAbsoluteSetPoint(true);
        irTurnPidCtrl = new TrcPidController(
                "irTurnPid",
                new TrcPidController.PidCoefficients(IRTURN_KP, IRTURN_KI, IRTURN_KD),
                IRTURN_TOLERANCE, this);
        irDrivePidCtrl.setAbsoluteSetPoint(true);
        pidSeekIr = new TrcPidDrive(
                "seekIr", driveBase, null, irDrivePidCtrl, irTurnPidCtrl);
        //
        // Arm subsystem.
        //
        armServo = new FtcServo("servo_1");
        armServo.setLogicalRange(ARM_MIN_RANGE, ARM_MAX_RANGE);
        arm = new TrcEnhancedServo("arm", armServo);
        arm.setPosition(ARM_MIN_RANGE);
        //
        // Claw subsystem.
        //
        clawServo = new FtcServo("servo_6");
        clawServo.setLogicalRange(CLAW_MIN_RANGE, CLAW_MAX_RANGE);
        claw = new TrcEnhancedServo("claw", clawServo);
        claw.setPosition(CLAW_MIN_RANGE);
    }   //K9Robot

    public void startMode(TrcRobot.RunMode runMode)
    {
        dashboard.clearDisplay();
        gyro.setEnabled(true);
        colorSensor.sensor.enableLed(true);
        driveBase.resetPosition();
    }   //startMode

    public void stopMode(TrcRobot.RunMode runMode)
    {
        gyro.setEnabled(false);
        colorSensor.sensor.enableLed(false);
    }   //stopMode

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
            input = colorSensor.sensor.alpha();
            //
            // Give it a deadband to minimize fish tailing.
            //
            if (Math.abs(input - COLOR_LINE_EDGE_LEVEL) < COLOR_LINE_EDGE_DEADBAND)
            {
                input = COLOR_LINE_EDGE_LEVEL;
            }
        }
        else if (pidCtrl == lightPidCtrl)
        {
            //
            // Get the light sensor reading.
            //
            input = lightSensor.sensor.getRawLightDetected();
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
        if (analogTrigger == colorTrigger && pidDrive.isActive())
        {
            //
            // Line is detected, interrupt PID drive.
            //
            if (zoneIndex > 0)
            {
                pidDrive.cancel();
            }
        }
        else if (analogTrigger == lightTrigger && pidDrive.isActive())
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

}   //class K9Robot
