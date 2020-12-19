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

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import trclib.TrcPose2D;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class RobotInfo
{
    //
    // DriveBase subsystem.
    //
    static final DcMotor.RunMode DRIVE_MOTOR_MODE               = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    static final boolean LEFT_WHEEL_INVERTED                    = false;
    static final boolean RIGHT_WHEEL_INVERTED                   = true;
    static final boolean DRIVE_WHEEL_BRAKE_MODE                 = true;
    static final double TURN_POWER_LIMIT                        = 0.5;
    static final double SLOW_DRIVE_POWER_SCALE                  = 0.5;
    //
    // Velocity controlled constants.
    //
    static final double MOTOR_MAX_VELOCITY                      = 2800;     //encoder counts/second

    static final double ENCODER_X_KP                            = 0.095;
    static final double ENCODER_X_KI                            = 0.0;
    static final double ENCODER_X_KD                            = 0.001;
    static final double ENCODER_X_TOLERANCE                     = 1.0;
    static final double ENCODER_X_INCHES_PER_COUNT              = 0.0163125145666872;

    static final double ENCODER_Y_KP                            = 0.06;
    static final double ENCODER_Y_KI                            = 0.0;
    static final double ENCODER_Y_KD                            = 0.002;
    static final double ENCODER_Y_TOLERANCE                     = 1.0;
    static final double ENCODER_Y_INCHES_PER_COUNT              = 0.0174484434975099;

    static final double GYRO_KP                                 = 0.009;
    static final double GYRO_KI                                 = 0.0;
    static final double GYRO_KD                                 = 0.0005;
    static final double GYRO_TOLERANCE                          = 2.0;

    static final double PIDDRIVE_STALL_TIMEOUT                  = 0.2;  //in seconds.
    //
    // Pure Pursuit parameters.
    //
    static final double PURE_PURSUIT_POS_KP                     = (.1 + .05) / 2.0;
    static final double PURE_PURSUIT_POS_KI                     = 0.0;
    static final double PURE_PURSUIT_POS_KD                     = 0.0;

    static final double PURE_PURSUIT_TURN_KP                    = 0.0125;
    static final double PURE_PURSUIT_TURN_KI                    = 0.0;
    static final double PURE_PURSUIT_TURN_KD                    = 0.00125;

    static final double PURE_PURSUIT_VEL_KP                     = 0.0;
    static final double PURE_PURSUIT_VEL_KI                     = 0.0;
    static final double PURE_PURSUIT_VEL_KD                     = 0.9;
    // Neverest 40 motor, max shaft speed = 160 RPM
    // motor-to-wheel tooth ratio = 24:16 = 3:2
    // wheel max angular speed = (3 / 2) * 160 RPM
    // max tangential speed of wheel (in/s) = wheel max angular speed * 2 * pi * radius / 60.0
    // = (3 / 2) * (160 RPM) * 2 * 3.1415926 * (2 in.) / 60.0
    // = 50.2654816 in./sec.
    // KF should be set to the reciprocal of max tangential velocity (time to travel unit distance), units: sec./in.
    static final double PURE_PURSUIT_VEL_KF                     = 1.0 / 50.2654816;
    static final TrcPose2D[] PURE_PURSUIT_TEST_PATH             = new TrcPose2D[] {
            new TrcPose2D(0.0, 0.0, 0.0),
            new TrcPose2D(0.0, 36.0, 90.0),
            new TrcPose2D(24.0, 36.0, 0.0),
            new TrcPose2D(24.0, 48.0, 0.0)
    };

    //
    // Vision subsystem.
    //
    static final VuforiaLocalizer.CameraDirection CAMERA_DIR    = BACK;
    static final VuforiaLocalizer.Parameters.CameraMonitorFeedback CAMERA_MONITOR_FEEDBACK =
            VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
    static final String TRACKABLE_IMAGES_FILE                   = "Skystone";
    static final boolean PHONE_IS_PORTRAIT                      = false;
    static final double ROBOT_LENGTH                            = 18.0; //Robot length in inches
    static final double ROBOT_WIDTH                             = 18.0; //Robot width in inches
    static final double PHONE_FRONT_OFFSET                      = 5.5;  //Phone offset from front of robot in inches
    static final double PHONE_HEIGHT_OFFSET                     = 5.75; //Phone offset from floor in inches
    static final double PHONE_LEFT_OFFSET                       = 11.0; //Phone offset from left of robot in inches

    static final double HOMOGRAPHY_CAMERA_TOPLEFT_X             = 0.0;
    static final double HOMOGRAPHY_CAMERA_TOPLEFT_Y             = 360.0;
    static final double HOMOGRAPHY_CAMERA_TOPRIGHT_X            = 1280.0;
    static final double HOMOGRAPHY_CAMERA_TOPRIGHT_Y            = 360.0;
    static final double HOMOGRAPHY_CAMERA_BOTTOMLEFT_X          = 0.0;
    static final double HOMOGRAPHY_CAMERA_BOTTOMLEFT_Y          = 720.0;
    static final double HOMOGRAPHY_CAMERA_BOTTOMRIGHT_X         = 1280.0;
    static final double HOMOGRAPHY_CAMERA_BOTTOMRIGHT_Y         = 720.0;
    // These should be in real-world robot coordinates. Needs calibration after camera is actually mounted in position.
    // Measurement unit: inches
    static final double HOMOGRAPHY_WORLD_TOPLEFT_X              = -61.0;
    static final double HOMOGRAPHY_WORLD_TOPLEFT_Y              = 83.0;
    static final double HOMOGRAPHY_WORLD_TOPRIGHT_X             = 33.0;
    static final double HOMOGRAPHY_WORLD_TOPRIGHT_Y             = 83.0;
    static final double HOMOGRAPHY_WORLD_BOTTOMLEFT_X           = -39.5;
    static final double HOMOGRAPHY_WORLD_BOTTOMLEFT_Y           = 19.0;
    static final double HOMOGRAPHY_WORLD_BOTTOMRIGHT_X          = 12.0;
    static final double HOMOGRAPHY_WORLD_BOTTOMRIGHT_Y          = 19.0;
    //
    // Elevator subsystem.
    //
    static final double ELEVATOR_KP                             = 0.5;
    static final double ELEVATOR_KI                             = 0.0;
    static final double ELEVATOR_KD                             = 0.0;
    static final double ELEVATOR_TOLERANCE                      = 0.5;
    static final boolean ELEVATOR_INVERTED                      = false;
    static final boolean ELEVATOR_HAS_LOWER_LIMIT_SWITCH        = true;
    static final boolean ELEVATOR_HAS_UPPER_LIMIT_SWITCH        = false;
    static final double ELEVATOR_CAL_POWER                      = 0.5;
    static final double ELEVATOR_STALL_MIN_POWER                = 0.5;
    static final double ELEVATOR_STALL_TIMEOUT                  = 1.0;
    static final double ELEVATOR_RESET_TIMEOUT                  = 0.5;
    static final double ELEVATOR_MIN_HEIGHT                     = 0.0;
    static final double ELEVATOR_MAX_HEIGHT                     = 19.0;
    static final double ELEVATOR_SCALE                          = 17.0/4718.0;
    static final double ELEVATOR_OFFSET                         = 0.0;

}   //class RobotInfo
