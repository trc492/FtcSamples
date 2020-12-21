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

import android.speech.tts.TextToSpeech;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import ftclib.FtcAndroidTone;
import ftclib.FtcBNO055Imu;
import ftclib.FtcOpMode;
import ftclib.FtcRevBlinkin;
import ftclib.FtcRobotBattery;
import ftclib.FtcDcMotor;
import ftclib.FtcMotorActuator;
import ftclib.FtcVuforia;
import hallib.HalDashboard;
import trclib.TrcDbgTrace;
import trclib.TrcDigitalInput;
import trclib.TrcDriveBase;
import trclib.TrcDriveBaseOdometry;
import trclib.TrcGyro;
import trclib.TrcHomographyMapper;
import trclib.TrcMecanumDriveBase;
import trclib.TrcMotor;
import trclib.TrcPidController;
import trclib.TrcPidDrive;
import trclib.TrcRevBlinkin;
import trclib.TrcRobot;
import trclib.TrcServo;
import trclib.TrcUtil;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * This class creates the robot object that consists of sensors, indicators, drive base and all the subsystems.
 */
public class Robot
{
    //
    // Preferences and parameters.
    //
    public static class Preferences
    {
        static boolean visionOnly = false;
        static boolean initSubsystems = true;
        static boolean useExternalOdometry = false;
        static boolean hasElevator = true;
        static boolean hasBlinkin = true;
        static boolean useVuforia = true;
        static boolean useTensorFlow = false;
        static boolean showVuforiaView = false;
        static boolean showTensorFlowView = false;
        static boolean usePhoneFlashLight = true;
        static boolean useBlinkinFlashLight = true;
        static boolean useTraceLog = true;
        static boolean useSpeech = true;
        static boolean useBatteryMonitor = false;
        static boolean useLoopPerformanceMonitor = true;
        static boolean useVelocityControl = false;
    }   //class Preferences

    private static class PhoneParameters
    {
        static VuforiaLocalizer.CameraDirection cameraDir = RobotInfo.CAMERA_DIR;
        static VuforiaLocalizer.Parameters.CameraMonitorFeedback cameraMonitorFeedback =
                RobotInfo.CAMERA_MONITOR_FEEDBACK;
        static boolean phoneIsPortrait = RobotInfo.PHONE_IS_PORTRAIT;
        static double phoneFrontOffset = RobotInfo.PHONE_FRONT_OFFSET;
        static double phoneLeftOffset = RobotInfo.PHONE_LEFT_OFFSET;
        static double phoneHeightOffset = RobotInfo.PHONE_HEIGHT_OFFSET;
    }   //class PhoneParameters

    public enum DriveMode
    {
        TANK_MODE,
        HOLONOMIC_MODE,
    }   //enum DriveMode

    //
    // Global objects.
    //
    private static final String OPENCV_NATIVE_LIBRARY_NAME = "opencv_java3";
    public FtcOpMode opMode;
    public HalDashboard dashboard;
    public TrcDbgTrace globalTracer;
    public FtcAndroidTone androidTone;
    public TextToSpeech textToSpeech;
    //
    // Vision subsystems.
    //
    public FtcVuforia vuforia;
    public VuforiaVision vuforiaVision;
    public TensorFlowVision tensorFlowVision;
    //
    // Sensors and indicators.
    //
    public FtcRobotBattery battery;
    public FtcRevBlinkin blinkin;
    public FtcBNO055Imu imu;
    public TrcGyro gyro;
    //
    // DriveBase subsystem.
    //
    public FtcDcMotor leftFrontWheel = null;
    public FtcDcMotor rightFrontWheel = null;
    public FtcDcMotor leftBackWheel = null;
    public FtcDcMotor rightBackWheel = null;

    public TrcDriveBase driveBase = null;
    public DriveMode driveMode = DriveMode.HOLONOMIC_MODE;
    public TrcPidController encoderXPidCtrl = null;
    public TrcPidController encoderYPidCtrl = null;
    public TrcPidController gyroPidCtrl = null;
    public TrcPidDrive pidDrive = null;

    // Pure Pursuit PID controllers.
    public TrcPidController.PidCoefficients posPidCoeff = null;
    public TrcPidController.PidCoefficients turnPidCoeff = null;
    public TrcPidController.PidCoefficients velPidCoeff = null;

//    public TrcPose2D fieldOrigin = null;

    public TrcPidController.PidCoefficients tunePidCoeff = new TrcPidController.PidCoefficients();
    //
    // Subsystems.
    //
    public FtcMotorActuator elevator = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param runMode specifies robot running mode (Auto, TeleOp, Test), can be used to create and initialize mode
     *                specific sensors and subsystems if necessary.
     */
    public Robot(TrcRobot.RunMode runMode)
    {
        //
        // Initialize global objects.
        //
        opMode = FtcOpMode.getInstance();
        opMode.hardwareMap.logDevices();
        dashboard = HalDashboard.getInstance();
        dashboard.setTextView(
                ((FtcRobotControllerActivity)opMode.hardwareMap.appContext).findViewById(
                        com.qualcomm.ftcrobotcontroller.R.id.textOpMode));
        globalTracer = TrcDbgTrace.getGlobalTracer();
        androidTone = new FtcAndroidTone("AndroidTone");
        if (Preferences.useSpeech)
        {
            textToSpeech = FtcOpMode.getInstance().getTextToSpeech();
            speak("Init starting", "initStart");
        }
        //
        // Initialize vision subsystems.
        //
        if (Preferences.useVuforia || Preferences.useTensorFlow)
        {
            final String VUFORIA_LICENSE_KEY = "Your_Own_Vuforia_Key";
            int cameraViewId = !Preferences.showVuforiaView ? -1 :
                    opMode.hardwareMap.appContext.getResources().getIdentifier(
                            "cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());

            vuforia = new FtcVuforia(
                    VUFORIA_LICENSE_KEY, cameraViewId, PhoneParameters.cameraDir, PhoneParameters.cameraMonitorFeedback);

            if (runMode == TrcRobot.RunMode.AUTO_MODE || runMode == TrcRobot.RunMode.TEST_MODE)
            {
                if (Preferences.useVuforia)
                {
                    initVuforia();
                }

                if (Preferences.useTensorFlow)
                {
                    TrcHomographyMapper.Rectangle cameraRect = new TrcHomographyMapper.Rectangle(
                            RobotInfo.HOMOGRAPHY_CAMERA_TOPLEFT_X, RobotInfo.HOMOGRAPHY_CAMERA_TOPLEFT_Y,
                            RobotInfo.HOMOGRAPHY_CAMERA_TOPRIGHT_X, RobotInfo.HOMOGRAPHY_CAMERA_TOPRIGHT_Y,
                            RobotInfo.HOMOGRAPHY_CAMERA_BOTTOMLEFT_X, RobotInfo.HOMOGRAPHY_CAMERA_BOTTOMLEFT_Y,
                            RobotInfo.HOMOGRAPHY_CAMERA_BOTTOMRIGHT_X, RobotInfo.HOMOGRAPHY_CAMERA_BOTTOMRIGHT_Y);

                    TrcHomographyMapper.Rectangle worldRect = new TrcHomographyMapper.Rectangle(
                            RobotInfo.HOMOGRAPHY_WORLD_TOPLEFT_X, RobotInfo.HOMOGRAPHY_WORLD_TOPLEFT_Y,
                            RobotInfo.HOMOGRAPHY_WORLD_TOPRIGHT_X, RobotInfo.HOMOGRAPHY_WORLD_TOPRIGHT_Y,
                            RobotInfo.HOMOGRAPHY_WORLD_BOTTOMLEFT_X, RobotInfo.HOMOGRAPHY_WORLD_BOTTOMLEFT_Y,
                            RobotInfo.HOMOGRAPHY_WORLD_BOTTOMRIGHT_X, RobotInfo.HOMOGRAPHY_WORLD_BOTTOMRIGHT_Y);

                    initTensorFlow(Preferences.showTensorFlowView, cameraRect, worldRect);
                }
            }
        }
        //
        // If visionOnly is true, the robot controller is disconnected from the robot for testing vision.
        // In this case, we should not instantiate any robot hardware.
        //
        if (!Preferences.visionOnly)
        {
            //
            // Initialize sensors and indicators.
            //
            if (Preferences.useBatteryMonitor)
            {
                battery = new FtcRobotBattery();
            }

            if (Preferences.hasBlinkin)
            {
                blinkin = new FtcRevBlinkin("blinkin");
            }
            imu = new FtcBNO055Imu("imu");
            gyro = imu.gyro;
            //
            // Initialize DriveBase.
            //
            initDriveBase();
            //
            // Initialize other subsystems.
            //
            if (Preferences.initSubsystems)
            {
                if (Preferences.hasElevator)
                {
                    final FtcMotorActuator.Parameters elevatorParams = new FtcMotorActuator.Parameters()
                            .setPosRange(RobotInfo.ELEVATOR_MIN_HEIGHT, RobotInfo.ELEVATOR_MAX_HEIGHT)
                            .setScaleOffset(RobotInfo.ELEVATOR_SCALE, RobotInfo.ELEVATOR_OFFSET)
                            .setPidParams(
                                    RobotInfo.ELEVATOR_KP, RobotInfo.ELEVATOR_KI, RobotInfo.ELEVATOR_KD,
                                    RobotInfo.ELEVATOR_TOLERANCE)
                            .setMotorParams(
                                    RobotInfo.ELEVATOR_INVERTED, RobotInfo.ELEVATOR_HAS_LOWER_LIMIT_SWITCH,
                                    RobotInfo.ELEVATOR_HAS_UPPER_LIMIT_SWITCH, RobotInfo.ELEVATOR_CAL_POWER)
                            .setStallProtectionParams(
                                    RobotInfo.ELEVATOR_STALL_MIN_POWER, RobotInfo.ELEVATOR_STALL_TIMEOUT,
                                    RobotInfo.ELEVATOR_RESET_TIMEOUT);
                    elevator = new FtcMotorActuator("elevator", elevatorParams);
                    elevator.setBeep(androidTone);
                    elevator.zeroCalibrate();
                }
            }
        }
        //
        // Tell the driver initialization is complete.
        //
        speak("Init complete!", "initDone");
    }   //Robot

    /**
     * This method is call when the robot mode is about to start. It contains code to initialize robot hardware
     * necessary for running the robot mode.
     *
     * @param runMode specifies the robot mode it is about to start, can be used to initialize mode specific hardware.
     */
    public void startMode(TrcRobot.RunMode runMode)
    {
        final String funcName = "Robot.startMode";
        //
        // Since the IMU gyro is giving us cardinal heading, we need to enable its cardinal to cartesian converter.
        //
        if (gyro != null)
        {
            gyro.setEnabled(true);
        }
        //
        // Enable odometry only for autonomous or test modes.
        //
        if (driveBase != null && (runMode == TrcRobot.RunMode.AUTO_MODE || runMode == TrcRobot.RunMode.TEST_MODE))
        {
            leftFrontWheel.setOdometryEnabled(true);
            rightFrontWheel.setOdometryEnabled(true);
            leftBackWheel.setOdometryEnabled(true);
            rightBackWheel.setOdometryEnabled(true);
            driveBase.setOdometryEnabled(true);
        }
        //
        // Vision generally will impact performance, so we only enable it if it's needed such as in autonomous.
        //
        if (vuforiaVision != null && (runMode == TrcRobot.RunMode.AUTO_MODE || runMode == TrcRobot.RunMode.TEST_MODE))
        {
            globalTracer.traceInfo(funcName, "Enabling Vuforia.");
            vuforiaVision.setEnabled(true);
        }

        if (gyro != null)
        {
            gyro.setElapsedTimerEnabled(true);
        }
        TrcDigitalInput.setElapsedTimerEnabled(true);
        TrcMotor.setElapsedTimerEnabled(true);
        TrcServo.setElapsedTimerEnabled(true);
    }   //startMode

    /**
     * This method is call when the robot mode is about to end. It contains code to cleanup robot hardware before
     * exiting the robot mode.
     *
     * @param runMode specifies the robot mode it is about to start, can be used to cleanup mode specific hardware.
     */
    public void stopMode(TrcRobot.RunMode runMode)
    {
        final String funcName = "Robot.stopMode";

        gyro.printElapsedTime(globalTracer);
        gyro.setElapsedTimerEnabled(false);
        TrcDigitalInput.printElapsedTime(globalTracer);
        TrcDigitalInput.setElapsedTimerEnabled(false);
        TrcMotor.printElapsedTime(globalTracer);
        TrcMotor.setElapsedTimerEnabled(false);
        TrcServo.printElapsedTime(globalTracer);
        TrcServo.setElapsedTimerEnabled(false);

        if (vuforiaVision != null)
        {
            globalTracer.traceInfo(funcName, "Disabling Vuforia.");
            vuforiaVision.setEnabled(false);
        }

        if (tensorFlowVision != null)
        {
            globalTracer.traceInfo(funcName, "Shutting down TensorFlow.");
            tensorFlowVision.shutdown();
            tensorFlowVision = null;
        }
        if (driveBase != null)
        {
            driveBase.setOdometryEnabled(false);
            leftFrontWheel.setOdometryEnabled(false);
            rightFrontWheel.setOdometryEnabled(false);
            leftBackWheel.setOdometryEnabled(false);
            rightBackWheel.setOdometryEnabled(false);
        }

        if (gyro != null)
        {
            gyro.setEnabled(false);
        }

        if (textToSpeech != null)
        {
            textToSpeech.stop();
            textToSpeech.shutdown();
        }
    }   //stopMode

    /**
     * This method uses TextToSpeech to speak the given sentence.
     *
     * @param sentence specifies the sentence to be spoken.
     * @param utteranceId specifies the unique identifier for the request.
     */
    public void speak(String sentence, String utteranceId)
    {
        if (textToSpeech != null)
        {
            textToSpeech.speak(sentence, TextToSpeech.QUEUE_FLUSH, null, utteranceId);
        }
    }   //speak

    /**
     * This method turns the flashlight ON or OFF for the vision subsystem.
     *
     * @param phoneFlashOn specifies true to turn on the phone flashlight, false to turn off.
     * @param blinkinFlashOn specifies true to turn on the Blinkin LED, false to turn off.
     */
    public void setFlashLightOn(boolean phoneFlashOn, boolean blinkinFlashOn)
    {
        if (vuforia != null && Preferences.usePhoneFlashLight)
        {
            vuforia.setFlashlightEnabled(phoneFlashOn);
        }

        if (blinkin != null && Preferences.useBlinkinFlashLight)
        {
            blinkin.setPattern(
                    blinkinFlashOn? TrcRevBlinkin.LEDPattern.SolidWhite: TrcRevBlinkin.LEDPattern.SolidBlack);
        }
    }   //setFlashLightOn

    /**
     * This method initializes Vuforia for the vision subsystem.
     */
    private void initVuforia()
    {
        float phoneXRotate;
        float phoneYRotate;
        float phoneZRotate = 0.0f;
        /*
         * Create a transformation matrix describing where the phone is on the robot.
         *
         * The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the
         * Y axis. Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
         * pointing to the LEFT side of the Robot.  It's very important when you test this code that the top
         * of the camera is pointing to the left side of the  robot.  The rotation angles don't work if you flip
         * the phone.
         *
         * If using the back (High Res) camera:
         * We need to rotate the camera around it's long axis to bring the back camera forward.
         * This requires a negative 90 degree rotation on the Y axis
         *
         * If using the Front (Low Res) camera
         * We need to rotate the camera around it's long axis to bring the FRONT camera forward.
         * This requires a Positive 90 degree rotation on the Y axis
         *
         * Next, translate the camera lens to where it is on the robot.
         * In this example, it is centered (left to right), but 110 mm forward of the middle of the robot, and
         * 200 mm above ground level.
         */
        final int CAMERA_FORWARD_DISPLACEMENT =
                (int)((RobotInfo.ROBOT_LENGTH/2.0 - PhoneParameters.phoneFrontOffset)* TrcUtil.MM_PER_INCH);
        final int CAMERA_VERTICAL_DISPLACEMENT =
                (int)(PhoneParameters.phoneHeightOffset*TrcUtil.MM_PER_INCH);
        final int CAMERA_LEFT_DISPLACEMENT =
                (int)((RobotInfo.ROBOT_WIDTH/2.0 - PhoneParameters.phoneLeftOffset)*TrcUtil.MM_PER_INCH);
        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        phoneYRotate = PhoneParameters.cameraDir == BACK ? -90.0f : 90.0f;

        // Rotate the phone vertical about the X axis if it's in portrait mode
        phoneXRotate = PhoneParameters.phoneIsPortrait ? 90.0f : 0.0f;

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        phoneYRotate, phoneZRotate, phoneXRotate));

        vuforiaVision = new VuforiaVision(this, vuforia, robotFromCamera);
    }   //initVuforia

    /**
     * This method initialize TensorFlow for the vision subsystem.
     *
     * @param showTensorFlowView specifies true to show TensorFlow camera view.
     * @param cameraRect specifies the camera rectangle for Homography Mapper.
     * @param worldRect specifies the world rectangle for Homography Mapper.
     */
    private void initTensorFlow(
            boolean showTensorFlowView, TrcHomographyMapper.Rectangle cameraRect,
            TrcHomographyMapper.Rectangle worldRect)
    {
        final String funcName = "initTensorFlow";

        System.loadLibrary(OPENCV_NATIVE_LIBRARY_NAME);

        int tfodMonitorViewId = !showTensorFlowView ? -1 :
                opMode.hardwareMap.appContext.getResources().getIdentifier(
                        "tfodMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        tensorFlowVision = new TensorFlowVision(
                vuforia, tfodMonitorViewId, cameraRect, worldRect, globalTracer);
        tensorFlowVision.setEnabled(true, Preferences.usePhoneFlashLight);
        globalTracer.traceInfo(funcName, "Enabling TensorFlow.");
    } //initTensorFlow

    /**
     * This method creates and initializes the drive base related components.
     */
    private void initDriveBase()
    {
        leftFrontWheel = new FtcDcMotor("lfWheel");
        rightFrontWheel = new FtcDcMotor("rfWheel");
        leftBackWheel = new FtcDcMotor("lbWheel");
        rightBackWheel = new FtcDcMotor("rbWheel");

        leftFrontWheel.motor.setMode(RobotInfo.DRIVE_MOTOR_MODE);
        rightFrontWheel.motor.setMode(RobotInfo.DRIVE_MOTOR_MODE);
        leftBackWheel.motor.setMode(RobotInfo.DRIVE_MOTOR_MODE);
        rightBackWheel.motor.setMode(RobotInfo.DRIVE_MOTOR_MODE);

        if (Preferences.useVelocityControl)
        {
            leftFrontWheel.enableVelocityMode(RobotInfo.MOTOR_MAX_VELOCITY);
            rightFrontWheel.enableVelocityMode(RobotInfo.MOTOR_MAX_VELOCITY);
            leftBackWheel.enableVelocityMode(RobotInfo.MOTOR_MAX_VELOCITY);
            rightBackWheel.enableVelocityMode(RobotInfo.MOTOR_MAX_VELOCITY);
        }

        leftFrontWheel.setInverted(RobotInfo.LEFT_WHEEL_INVERTED);
        leftBackWheel.setInverted(RobotInfo.LEFT_WHEEL_INVERTED);
        rightFrontWheel.setInverted(RobotInfo.RIGHT_WHEEL_INVERTED);
        rightBackWheel.setInverted(RobotInfo.RIGHT_WHEEL_INVERTED);

        leftFrontWheel.setBrakeModeEnabled(RobotInfo.DRIVE_WHEEL_BRAKE_MODE);
        leftBackWheel.setBrakeModeEnabled(RobotInfo.DRIVE_WHEEL_BRAKE_MODE);
        rightFrontWheel.setBrakeModeEnabled(RobotInfo.DRIVE_WHEEL_BRAKE_MODE);
        rightBackWheel.setBrakeModeEnabled(RobotInfo.DRIVE_WHEEL_BRAKE_MODE);

        driveBase = new TrcMecanumDriveBase(leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel, gyro);
        if (Preferences.useExternalOdometry)
        {
            //
            // Create the external odometry device that uses the left front encoder port as the X odometry and
            // the left and right back encoder ports as the Y1 and Y2 odometry. Gyro will serve as the angle
            // odometry.
            //
            TrcDriveBaseOdometry driveBaseOdometry = new TrcDriveBaseOdometry(
                    leftFrontWheel, leftBackWheel, rightBackWheel, gyro);
            //
            // Set the drive base to use the external odometry device overriding the built-in one.
            //
            driveBase.setDriveBaseOdometry(driveBaseOdometry);
        }
        driveBase.setOdometryScales(RobotInfo.ENCODER_X_INCHES_PER_COUNT, RobotInfo.ENCODER_Y_INCHES_PER_COUNT);
        driveMode = DriveMode.HOLONOMIC_MODE;
        //
        // Initialize PID drive.
        //
        encoderXPidCtrl = new TrcPidController(
                "encoderXPidCtrl",
                new TrcPidController.PidCoefficients(
                        RobotInfo.ENCODER_X_KP, RobotInfo.ENCODER_X_KI, RobotInfo.ENCODER_X_KD),
                RobotInfo.ENCODER_X_TOLERANCE, driveBase::getXPosition);
        encoderYPidCtrl = new TrcPidController(
                "encoderYPidCtrl",
                new TrcPidController.PidCoefficients(
                        RobotInfo.ENCODER_Y_KP, RobotInfo.ENCODER_Y_KI, RobotInfo.ENCODER_Y_KD),
                RobotInfo.ENCODER_Y_TOLERANCE, driveBase::getYPosition);
        gyroPidCtrl = new TrcPidController(
                "gyroPidCtrl",
                new TrcPidController.PidCoefficients(
                        RobotInfo.GYRO_KP, RobotInfo.GYRO_KI, RobotInfo.GYRO_KD),
                RobotInfo.GYRO_TOLERANCE, driveBase::getHeading);
        gyroPidCtrl.setAbsoluteSetPoint(true);
        gyroPidCtrl.setOutputLimit(RobotInfo.TURN_POWER_LIMIT);

        pidDrive = new TrcPidDrive(
                "pidDrive", driveBase, encoderXPidCtrl, encoderYPidCtrl, gyroPidCtrl);
        pidDrive.setAbsoluteTargetModeEnabled(true);
        pidDrive.setStallTimeout(RobotInfo.PIDDRIVE_STALL_TIMEOUT);
        pidDrive.setBeep(androidTone);
        pidDrive.setMsgTracer(globalTracer);

        posPidCoeff = new TrcPidController.PidCoefficients(
                RobotInfo.PURE_PURSUIT_POS_KP, RobotInfo.PURE_PURSUIT_POS_KI, RobotInfo.PURE_PURSUIT_POS_KD);
        turnPidCoeff = new TrcPidController.PidCoefficients(
                RobotInfo.PURE_PURSUIT_TURN_KP, RobotInfo.PURE_PURSUIT_TURN_KI, RobotInfo.PURE_PURSUIT_TURN_KD);
         velPidCoeff = new TrcPidController.PidCoefficients(
                 RobotInfo.PURE_PURSUIT_VEL_KP, RobotInfo.PURE_PURSUIT_VEL_KI, RobotInfo.PURE_PURSUIT_VEL_KD,
                 RobotInfo.PURE_PURSUIT_VEL_KF);
    }   //initDriveBase

}   //class Robot
