/*
 * Titan Robotics Framework Library
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.net)
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

package hallib;

import trclib.TrcDbgTrace;
import trclib.TrcUtil;

/**
 * This class implements a robot drive base that supports 2-motor, 4-motor or 6-motor
 * drive trains. It supports tank drive, arcade drive, mecanum drive and swerve drive.
 * This is a port from the WPILib RobotDrive class and extended with addition features.
 */
public class HalRobotDrive
{
    public static class MotorType
    {
        public final int value;

        private static final int kFrontLeft_val = 0;
        private static final int kFrontRight_val = 1;
        private static final int kRearLeft_val = 2;
        private static final int kRearRight_val = 3;
        private static final int kMidLeft_val = 4;
        private static final int kMidRight_val = 5;

        public static final MotorType kFrontLeft = new MotorType(kFrontLeft_val);
        public static final MotorType kFrontRight = new MotorType(kFrontRight_val);
        public static final MotorType kRearLeft = new MotorType(kRearLeft_val);
        public static final MotorType kRearRight = new MotorType(kRearRight_val);
        public static final MotorType kMidLeft = new MotorType(kMidLeft_val);
        public static final MotorType kMidRight = new MotorType(kMidRight_val);

        private MotorType(int value)
        {
            this.value = value;
        }   //MotorType
    }   //class MotorType

    private static final String moduleName = "HalRobotDrive";
    private static final boolean debugEnabled = false;
    private TrcDbgTrace dbgTrace = null;

    public static double kDefaultSensitivity = 0.5;
    public static double kDefaultMaxOutput = 1.0;

    private static double MOTOR_MAX_VALUE = 1.0;
    private static double MOTOR_MIN_VALUE = -1.0;
    private static int MAX_NUM_MOTORS = 6;

    private double sensitivity;
    private double maxOutput;
    private int numMotors;
    private HalMotorController frontLeftMotor;
    private HalMotorController frontRightMotor;
    private HalMotorController rearLeftMotor;
    private HalMotorController rearRightMotor;
    private HalMotorController midLeftMotor;
    private HalMotorController midRightMotor;

    /**
     * Constructor: Create an instance of the object with 6 motors.
     *
     * @param frontLeftMotor specifies the left front motor controller object.
     * @param midLeftMotor specifies he left mid motor controller object.
     * @param rearLeftMotor specifies the left rear motor controller object.
     * @param frontRightMotor specifies the right front motor controller object.
     * @param midRightMotor specifies he right mid motor controller object.
     * @param rearRightMotor specifies the right rear motor controller object.
     */
    public HalRobotDrive(
            HalMotorController frontLeftMotor,
            HalMotorController midLeftMotor,
            HalMotorController rearLeftMotor,
            HalMotorController frontRightMotor,
            HalMotorController midRightMotor,
            HalMotorController rearRightMotor)
    {
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(
                    moduleName,
                    false,
                    TrcDbgTrace.TraceLevel.API,
                    TrcDbgTrace.MsgLevel.INFO);
        }

        sensitivity = kDefaultSensitivity;
        maxOutput = kDefaultMaxOutput;
        numMotors = 0;

        this.frontLeftMotor = frontLeftMotor;
        if (frontLeftMotor != null) numMotors++;
        this.midLeftMotor = midLeftMotor;
        if (midLeftMotor != null) numMotors++;
        this.rearLeftMotor = rearLeftMotor;
        if (rearLeftMotor != null) numMotors++;
        this.frontRightMotor = frontRightMotor;
        if (frontRightMotor != null) numMotors++;
        this.midRightMotor = midRightMotor;
        if (midRightMotor != null) numMotors++;
        this.rearRightMotor = rearRightMotor;
        if (rearRightMotor != null) numMotors++;

        stopMotor();
    }   //HalRobotDrive

    /**
     * This method drives the motors with the given magnitude and curve values.
     *
     * @param magnitude specifies the magnitude value.
     * @param curve specifies the curve value.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     */
    public void drive(double magnitude, double curve, boolean inverted)
    {
        final String funcName = "drive";
        double leftOutput;
        double rightOutput;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                                "mag=%f,curve=%f,inverted=%s",
                                magnitude, curve, Boolean.toString(inverted));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (curve < 0.0)
        {
            double value = Math.log(-curve);
            double ratio = (value - sensitivity)/(value + sensitivity);
            if (ratio == 0.0)
            {
                ratio = 0.0000000001;
            }
            leftOutput = magnitude/ratio;
            rightOutput = magnitude;
        }
        else if (curve > 0.0)
        {
            double value = Math.log(curve);
            double ratio = (value - sensitivity)/(value + sensitivity);
            if (ratio == 0.0)
            {
                ratio = 0.0000000001;
            }
            leftOutput = magnitude;
            rightOutput = magnitude/ratio;
        }
        else
        {
            leftOutput = magnitude;
            rightOutput = magnitude;
        }
        tankDrive(leftOutput, rightOutput, inverted);
    }   //drive

    /**
     * This method drives the motors with the given magnitude and curve values.
     *
     * @param magnitude specifies the magnitude value.
     * @param curve specifies the curve value.
     */
    public void drive(double magnitude, double curve)
    {
        drive(magnitude, curve, false);
    }   //drive

    /**
     * This method stops all the motors.
     */
    public void stopMotor()
    {
        final String funcName = "stopMotor";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (frontLeftMotor != null) frontLeftMotor.setPower(0.0);
        if (frontRightMotor != null) frontRightMotor.setPower(0.0);
        if (rearLeftMotor != null) rearLeftMotor.setPower(0.0);
        if (rearRightMotor != null) rearRightMotor.setPower(0.0);
        if (midLeftMotor != null) midLeftMotor.setPower(0.0);
        if (midRightMotor != null) midRightMotor.setPower(0.0);
    }   //stopMotor

    /**
     * This method sets the sensitivity for the drive() method.
     *
     * @param sensitivity specifies the sensitivity value.
     */
    public void setSensitivity(double sensitivity)
    {
        final String funcName = "setSensitivity";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                                "sensitivity=%f", sensitivity);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.sensitivity = sensitivity;
    }   //setSensitivity

    /**
     * This method sets the maximum output value of the motor.
     *
     * @param maxOutput specifies the maximum output value.
     */
    public void setMaxOutput(double maxOutput)
    {
        final String funcName = "setMaxOutput";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "maxOutput=%f", maxOutput);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.maxOutput = maxOutput;
    }   //setMaxOutput

    /**
     * This method returns the number of motors in the drive train.
     *
     * @return number of motors.
     */
    public int getNumMotors()
    {
        final String funcName = "getNumMotors";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%d", numMotors);
        }

        return numMotors;
    }   //getNumMotors

    /**
     * This method inverts direction of a given motor in the drive train.
     *
     * @param motorType specifies the motor in the drive train.
     * @param isInverted specifies true if inverting motor direction.
     */
    public void setInvertedMotor(MotorType motorType, boolean isInverted)
    {
        final String funcName = "setInvertedMotor";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                                "type=%s,inverted=%s",
                                motorType.toString(), Boolean.toString(isInverted));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        switch (motorType.value)
        {
            case MotorType.kFrontLeft_val:
                if (frontLeftMotor != null)
                {
                    frontLeftMotor.setInverted(isInverted);
                }
                break;

            case MotorType.kFrontRight_val:
                if (frontRightMotor != null)
                {
                    frontRightMotor.setInverted(isInverted);
                }
                break;

            case MotorType.kRearLeft_val:
                if (rearLeftMotor != null)
                {
                    rearLeftMotor.setInverted(isInverted);
                }
                break;

            case MotorType.kRearRight_val:
                if (rearRightMotor != null)
                {
                    rearRightMotor.setInverted(isInverted);
                }
                break;

            case MotorType.kMidLeft_val:
                if (midLeftMotor != null)
                {
                    midLeftMotor.setInverted(isInverted);
                }
                break;

            case MotorType.kMidRight_val:
                if (midRightMotor != null)
                {
                    midRightMotor.setInverted(isInverted);
                }
                break;
        }
    }   //setInvertedMotor

    /**
     * This method implements tank drive where leftPower controls the left motors
     * and right power controls the right motors.
     *
     * @param leftPower specifies left power value.
     * @param rightPower specifies right power value.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     */
    public void tankDrive(double leftPower, double rightPower, boolean inverted)
    {
        final String funcName = "tankDrive";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                                "leftPower=%f,rightPower=%f,inverted=%s",
                                leftPower, rightPower, Boolean.toString(inverted));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        leftPower = TrcUtil.limit(leftPower);
        rightPower = TrcUtil.limit(rightPower);

        if (inverted)
        {
            double swap = leftPower;
            leftPower = -rightPower;
            rightPower = -swap;
        }

        if (frontLeftMotor != null)
        {
            frontLeftMotor.setPower(leftPower);
        }

        if (frontRightMotor != null)
        {
            frontRightMotor.setPower(rightPower);
        }

        if (rearLeftMotor != null)
        {
            rearLeftMotor.setPower(leftPower);
        }

        if (rearRightMotor != null)
        {
            rearRightMotor.setPower(rightPower);
        }

        if (midLeftMotor != null)
        {
            midLeftMotor.setPower(leftPower);
        }

        if (midRightMotor != null)
        {
            midRightMotor.setPower(rightPower);
        }
    }   //tankDrive

    /**
     * This method implements tank drive where leftPower controls the left motors
     * and right power controls the right motors.
     *
     * @param leftPower specifies left power value.
     * @param rightPower specifies right power value.
     */
    public void tankDrive(double leftPower, double rightPower)
    {
        tankDrive(leftPower, rightPower, false);
    }   //tankDrive

    /**
     * This method implements arcade drive where drivePower controls how fast
     * the robot goes in the y-axis and turnPower controls how fast it will
     * turn.
     *
     * @param drivePower specifies the drive power value.
     * @param turnPower specifies the turn power value.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     */
    public void arcadeDrive(double drivePower, double turnPower, boolean inverted)
    {
        final String funcName = "arcadeDrive";
        double leftPower;
        double rightPower;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                                "drivePower=%f,turnPower=%f,inverted=%s",
                                drivePower, turnPower, Boolean.toString(inverted));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        drivePower = TrcUtil.limit(drivePower);
        turnPower = TrcUtil.limit(turnPower);

        if (drivePower + turnPower > MOTOR_MAX_VALUE)
        {
            //
            // Forward right:
            //  left = drive + turn - (drive + turn - MOTOR_MAX_VALUE)
            //  right = drive - turn - (drive + turn - MOTOR_MAX_VALUE)
            //
            leftPower = MOTOR_MAX_VALUE;
            rightPower = -2*turnPower + MOTOR_MAX_VALUE;
        }
        else if (drivePower - turnPower > MOTOR_MAX_VALUE)
        {
            //
            // Forward left:
            //  left = drive + turn - (drive - turn - MOTOR_MAX_VALUE)
            //  right = drive - turn - (drive - turn - MOTOR_MAX_VALUE)
            //
            leftPower = 2*turnPower + MOTOR_MAX_VALUE;
            rightPower = MOTOR_MAX_VALUE;
        }
        else if (drivePower + turnPower < MOTOR_MIN_VALUE)
        {
            //
            // Backward left:
            //  left = drive + turn - (drive + turn - MOTOR_MIN_VALUE)
            //  right = drive - turn - (drive + turn - MOTOR_MIN_VALUE)
            //
            leftPower = MOTOR_MIN_VALUE;
            rightPower = -2*turnPower + MOTOR_MIN_VALUE;
        }
        else if (drivePower - turnPower < MOTOR_MIN_VALUE)
        {
            //
            // Backward right:
            //  left = drive + turn - (drive - turn - MOTOR_MIN_VALUE)
            //  right = drive - turn - (drive - turn - MOTOR_MIN_VALUE)
            //
            leftPower = 2*turnPower + MOTOR_MIN_VALUE;
            rightPower = MOTOR_MIN_VALUE;
        }
        else
        {
            leftPower = drivePower + turnPower;
            rightPower = drivePower - turnPower;
        }
        tankDrive(leftPower, rightPower, inverted);
    }   //arcadeDrive

    /**
     * This method implements arcade drive where drivePower controls how fast
     * the robot goes in the y-axis and turnPower controls how fast it will
     * turn.
     *
     * @param drivePower specifies the drive power value.
     * @param turnPower specifies the turn power value.
     */
    public void arcadeDrive(double drivePower, double turnPower)
    {
        arcadeDrive(drivePower, turnPower, false);
    }   //arcadeDrive

    /**
     * This method implements mecanum drive where x controls how fast the robot will
     * go in the x direction, and y controls how fast the robot will go in the y direction.
     * Rotation controls how fast the robot rotates and gyroAngle specifies the heading
     * the robot should maintain.
     * @param x specifies the x power.
     * @param y specifies the y power.
     * @param rotation specifies the rotating power.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     * @param gyroAngle specifies the gyro angle to maintain.
     */
    public void mecanumDrive_Cartesian(double x, double y, double rotation,
                                       boolean inverted, double gyroAngle)
    {
        final String funcName = "mecanumDrive_Cartesian";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                                "x=%f,y=%f,rot=%f,inverted=%s,angle=%f",
                                x, y, rotation, Boolean.toString(inverted), gyroAngle);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (numMotors != 4)
        {
            throw new IllegalArgumentException("Mecanum drive requires 4 motors");
        }

        x = TrcUtil.limit(x);
        y = TrcUtil.limit(y);
        rotation = TrcUtil.limit(rotation);

        if (inverted)
        {
            x = -x;
            y = -y;
        }

        double cosA = Math.cos(Math.toRadians(gyroAngle));
        double sinA = Math.sin(Math.toRadians(gyroAngle));
        x = x*cosA - y*sinA;
        y = x*sinA + y*cosA;

        double wheelSpeeds[] = new double[4];
        wheelSpeeds[MotorType.kFrontLeft_val] = x + y + rotation;
        wheelSpeeds[MotorType.kFrontRight_val] = -x + y - rotation;
        wheelSpeeds[MotorType.kRearLeft_val] = -x + y + rotation;
        wheelSpeeds[MotorType.kRearRight_val] = x + y - rotation;
        normalize(wheelSpeeds);

        if (frontLeftMotor != null)
        {
            frontLeftMotor.setPower(wheelSpeeds[MotorType.kFrontLeft_val]);
        }

        if (frontRightMotor != null)
        {
            frontRightMotor.setPower(wheelSpeeds[MotorType.kFrontRight_val]);
        }

        if (rearLeftMotor != null)
        {
            rearLeftMotor.setPower(wheelSpeeds[MotorType.kRearLeft_val]);
        }

        if (rearRightMotor != null)
        {
            rearRightMotor.setPower(wheelSpeeds[MotorType.kRearRight_val]);
        }
    }   //mecanumDrive_Cartesian

    /**
     * This method implements mecanum drive where x controls how fast the robot will
     * go in the x direction, and y controls how fast the robot will go in the y direction.
     * Rotation controls how fast the robot rotates and gyroAngle specifies the heading
     * the robot should maintain.
     * @param x specifies the x power.
     * @param y specifies the y power.
     * @param rotation specifies the rotating power.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     */
    public void mecanumDrive_Cartesian(double x, double y, double rotation, boolean inverted)
    {
        mecanumDrive_Cartesian(x, y, rotation, inverted, 0.0);
    }   //mecanumDrive_Cartesian

    /**
     * This method implements mecanum drive where x controls how fast the robot will
     * go in the x direction, and y controls how fast the robot will go in the y direction.
     * Rotation controls how fast the robot rotates.
     *
     * @param x specifies the x power.
     * @param y specifies the y power.
     * @param rotation specifies the rotating power.
     */
    public void mecanumDrive_Cartesian(double x, double y, double rotation)
    {
        mecanumDrive_Cartesian(x, y, rotation, false, 0.0);
    }   //mecanumDrive_Cartesian

    /**
     * This method implements mecanum drive where magnitude controls how fast the robot
     * will go in the given direction and how fast it will robote.
     *
     * @param magnitude specifies the magnitude combining x and y axes.
     * @param direction specifies the direction in degrees.
     * @param rotation specifies the rotation power.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     */
    public void mecanumDrive_Polar(double magnitude, double direction, double rotation,
                                   boolean inverted)
    {
        final String funcName = "mecanumDrive_Polar";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                                "mag=%f,dir=%f,rot=%f,inverted=%s",
                                magnitude, direction, rotation, Boolean.toString(inverted));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (numMotors != 4)
        {
            throw new IllegalArgumentException("Mecanum drive requires 4 motors");
        }

        magnitude = TrcUtil.limit(magnitude)*Math.sqrt(2.0);
        if (inverted)
        {
            direction += 180.0;
            direction %= 360.0;
        }

        double dirInRad = Math.toRadians(direction + 45.0);
        double cosD = Math.cos(dirInRad);
        double sinD = Math.sin(dirInRad);

        double wheelSpeeds[] = new double[4];
        wheelSpeeds[MotorType.kFrontLeft_val] = (sinD*magnitude + rotation);
        wheelSpeeds[MotorType.kFrontRight_val] = (cosD*magnitude - rotation);
        wheelSpeeds[MotorType.kRearLeft_val] = (cosD*magnitude + rotation);
        wheelSpeeds[MotorType.kRearRight_val] = (sinD*magnitude - rotation);
        normalize(wheelSpeeds);

        if (frontLeftMotor != null)
        {
            frontLeftMotor.setPower(wheelSpeeds[MotorType.kFrontLeft_val]);
        }

        if (frontRightMotor != null)
        {
            frontRightMotor.setPower(wheelSpeeds[MotorType.kFrontRight_val]);
        }

        if (rearLeftMotor != null)
        {
            rearLeftMotor.setPower(wheelSpeeds[MotorType.kRearLeft_val]);
        }

        if (rearRightMotor != null)
        {
            rearRightMotor.setPower(wheelSpeeds[MotorType.kRearRight_val]);
        }
    }   //mecanumDrive_Polar

    /**
     * This method implements mecanum drive where magnitude controls how fast the robot
     * will go in the given direction and how fast it will robote.
     *
     * @param magnitude specifies the magnitude combining x and y axes.
     * @param direction specifies the direction in degrees.
     * @param rotation specifies the rotation power.
     */
    public void mecanumDrive_Polar(double magnitude, double direction, double rotation)
    {
        mecanumDrive_Polar(magnitude, direction, rotation, false);
    }   //mecanumDrive_Polar

    /**
     * This method normalizes the power to the four wheels for mecanum drive.
     *
     * @param wheelSpeeds specifies the wheel speed of all four wheels.
     */
    private void normalize(double[] wheelSpeeds)
    {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++)
        {
            double magnitude = Math.abs(wheelSpeeds[i]);
            if (magnitude > maxMagnitude)
            {
                maxMagnitude = magnitude;
            }
        }

        if (maxMagnitude > 1.0)
        {
            for (int i = 0; i < wheelSpeeds.length; i++)
            {
                wheelSpeeds[i] /= maxMagnitude;
            }
        }
    }   //normalize

}   //HalRobotDrive
