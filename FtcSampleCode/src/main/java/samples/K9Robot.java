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

import com.qualcomm.robotcore.hardware.IrSeekerSensor;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import ftclib.FtcBNO055Imu;
import ftclib.FtcDcMotor;
import ftclib.FtcMRColorSensor;
import ftclib.FtcOpMode;
import ftclib.FtcOpticalDistanceSensor;
import ftclib.FtcRobotBattery;
import ftclib.FtcServo;
import hallib.HalDashboard;
import trclib.TrcAnalogTrigger;
import trclib.TrcDbgTrace;
import trclib.TrcEnhancedServo;
import trclib.TrcGyro;
import trclib.TrcPidController;
import trclib.TrcPidDrive;
import trclib.TrcSimpleDriveBase;

public class K9Robot
{
    private static final String moduleName = "K9Robot";
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

    public static final double ARM_RANGE_MIN            = 0.2;
    public static final double ARM_RANGE_MAX            = 0.9;
    public static final double CLAW_RANGE_MIN           = 0.2;
    public static final double CLAW_RANGE_MAX           = 0.7;
    public static final double SERVO_STEPRATE           = 2.0;

    //
    // Global objects.
    //
    public FtcOpMode opMode;
    public HalDashboard dashboard;
    public TrcDbgTrace globalTracer;
    public FtcRobotBattery battery = null;  //don't use battery monitor.
    //
    // Sensors.
    //
    public FtcBNO055Imu imu;
    public TrcGyro gyro;
    public double targetHeading = 0.0;
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
    public TrcSimpleDriveBase driveBase;
    //
    // PID drive.
    //
    public TrcPidController encoderXPidCtrl = null; //K9Robot doesn't support holonomic drive, so there is no X.
    public TrcPidController encoderYPidCtrl;
    public TrcPidController gyroPidCtrl;
    public TrcPidDrive pidDrive;
    public TrcPidController.PidCoefficients tunePidCoeff = new TrcPidController.PidCoefficients();
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

    public K9Robot()
    {
        //
        // Initialize global objects.
        //
        opMode = FtcOpMode.getInstance();
        opMode.hardwareMap.logDevices();
        dashboard = HalDashboard.getInstance();
        globalTracer = FtcOpMode.getGlobalTracer();
        dashboard.setTextView(
                ((FtcRobotControllerActivity)opMode.hardwareMap.appContext).findViewById(
                        FtcSampleCode.R.id.textOpMode));
        //
        // Initialize sensors.
        //
        imu = new FtcBNO055Imu("imu");
        gyro = imu.gyro;

        colorSensor = new FtcMRColorSensor("colorSensor");
        lightSensor = new FtcOpticalDistanceSensor("light_sensor");
        irSeeker = opMode.hardwareMap.irSeekerSensor.get("irSeeker");
        //
        // DriveBase subsystem.
        //
        motorLeft = new FtcDcMotor("motor_1");
        motorRight = new FtcDcMotor("motor_2");
        motorLeft.setOdometryEnabled(true);
        motorRight.setOdometryEnabled(true);
        motorLeft.setInverted(true);

        driveBase = new TrcSimpleDriveBase(motorLeft, motorRight, gyro);
        driveBase.setPositionScales(DRIVE_INCHES_PER_COUNT);
        //
        // PID drive.
        //
        encoderYPidCtrl = new TrcPidController(
                "encoderYPid",
                new TrcPidController.PidCoefficients(DRIVE_KP, DRIVE_KI, DRIVE_KD),
                DRIVE_TOLERANCE, driveBase::getYPosition);
        gyroPidCtrl = new TrcPidController(
                "gyroPid",
                new TrcPidController.PidCoefficients(TURN_KP, TURN_KI, TURN_KD),
                TURN_TOLERANCE, driveBase::getHeading);
        gyroPidCtrl.setAbsoluteSetPoint(true);
        pidDrive = new TrcPidDrive("pidDrive", driveBase, null, encoderYPidCtrl, gyroPidCtrl);
        //
        // PID line follow using color sensor.
        //
        colorPidCtrl = new TrcPidController(
                "colorPid",
                new TrcPidController.PidCoefficients(COLOR_KP, COLOR_KI, COLOR_KD),
                COLOR_TOLERANCE, this::getColorValue);
        colorPidCtrl.setAbsoluteSetPoint(true);
        pidLineFollow = new TrcPidDrive(
                "colorLineFollow", driveBase, null, encoderYPidCtrl, colorPidCtrl);
        // In order to line follow, we need to first find the line. We will first use pidDrive to keep the robot
        // moving forward for a set distance. Then colorTrigger will interrupt pidDrive once the line is detected.
        // Then we can use pidLineFollow to follow the line.
        colorTrigger = new TrcAnalogTrigger<>(
                "colorTrigger", colorSensor, 0, FtcMRColorSensor.DataType.WHITE,
                new double[]{COLOR_BLACK, COLOR_WHITE}, this::triggerEvent);
        //
        // PID line follow using Optical Distance sensor.
        //
        lightPidCtrl = new TrcPidController(
                "lightPid",
                new TrcPidController.PidCoefficients(LIGHT_KP, LIGHT_KI, LIGHT_KD),
                LIGHT_TOLERANCE, lightSensor.sensor::getRawLightDetected);
        lightPidCtrl.setAbsoluteSetPoint(true);
        lineFollowDrive = new TrcPidDrive(
                "lightLineFollow", driveBase, null, encoderYPidCtrl, lightPidCtrl);
        // In order to line follow, we need to first find the line. We will first use pidDrive to keep the robot
        // moving forward for a set distance. Then lightTrigger will interrupt pidDrive once the line is detected.
        // Then we can use lineFollowDrive to follow the line.
        lightTrigger = new TrcAnalogTrigger<>(
                "lightTrigger", lightSensor, 0, FtcOpticalDistanceSensor.DataType.RAW_LIGHT_DETECTED,
                new double[]{LIGHT_DARK_LEVEL, LIGHT_WHITE_LEVEL}, this::triggerEvent);
        //
        // PID IR seeking.
        //
        irDrivePidCtrl = new TrcPidController(
                "irDrivePid",
                new TrcPidController.PidCoefficients(IRDRIVE_KP, IRDRIVE_KI, IRDRIVE_KD),
                IRDRIVE_TOLERANCE, this::getIrStrength);
        irDrivePidCtrl.setAbsoluteSetPoint(true);
        irTurnPidCtrl = new TrcPidController(
                "irTurnPid",
                new TrcPidController.PidCoefficients(IRTURN_KP, IRTURN_KI, IRTURN_KD),
                IRTURN_TOLERANCE, this::getIrAngle);
        irDrivePidCtrl.setAbsoluteSetPoint(true);
        pidSeekIr = new TrcPidDrive(
                "seekIr", driveBase, null, irDrivePidCtrl, irTurnPidCtrl);
        //
        // Arm subsystem.
        //
        armServo = new FtcServo("servo_1");
        armServo.setLogicalRange(ARM_RANGE_MIN, ARM_RANGE_MAX);
        arm = new TrcEnhancedServo("arm", armServo);
        arm.setPosition(ARM_RANGE_MIN);
        //
        // Claw subsystem.
        //
        clawServo = new FtcServo("servo_6");
        clawServo.setLogicalRange(CLAW_RANGE_MIN, CLAW_RANGE_MAX);
        claw = new TrcEnhancedServo("claw", clawServo);
        claw.setPosition(CLAW_RANGE_MIN);
    }   //K9Robot

    public void startMode()
    {
        dashboard.clearDisplay();
        gyro.setEnabled(true);
        targetHeading = 0.0;
        driveBase.setOdometryEnabled(true);
        colorSensor.sensor.enableLed(true);
    }   //startMode

    public void stopMode()
    {
        gyro.setEnabled(false);
        colorSensor.sensor.enableLed(false);
        driveBase.setOdometryEnabled(false);
    }   //stopMode

    /**
     * This method is called when a threshold has been crossed.
     *
     * @param currZone specifies the zone it is going into.
     * @param prevZone specifies the zone it is coming out of.
     * @param zoneValue specifies the actual sensor value.
     */
    private void triggerEvent(int currZone, int prevZone, double zoneValue)
    {
        if (pidDrive.isActive() && currZone > 0)
        {
            pidDrive.cancel();
        }
    }   //triggerEvent

    /**
     * This method reads and returns the color sensor value.
     *
     * @return color sensor value.
     */
    private double getColorValue()
    {
        double input = colorSensor.sensor.alpha();
        //
        // Give it a deadband to minimize fish tailing.
        //
        if (Math.abs(input - COLOR_LINE_EDGE_LEVEL) < COLOR_LINE_EDGE_DEADBAND)
        {
            input = COLOR_LINE_EDGE_LEVEL;
        }

        return input;
    }   //getColorValue

    /**
     * This method reads and returns the IR strength value.
     *
     * @return IR strength value.
     */
    private double getIrStrength()
    {
        double input;
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

        return input;
    }   //getIrStrength

    /**
     * This method reads and returns the IR angle value.
     *
     * @return IR angle value.
     */
    private double getIrAngle()
    {
        double input;
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

        return input;
    }   //getIrAngle

    public void traceStateInfo(double elapsedTime, String stateName, double xDistance, double yDistance, double heading)
    {
        globalTracer.traceInfo(
                moduleName,
                "[%5.3f] >>>>> %s: xPos=%6.2f/%6.2f,yPos=%6.2f/%6.2f,heading=%6.1f/%6.1f",
                elapsedTime, stateName,
                driveBase.getXPosition(), xDistance, driveBase.getYPosition(), yDistance, driveBase.getHeading(),
                heading);
    }   //traceStateInfo

}   //class K9Robot
