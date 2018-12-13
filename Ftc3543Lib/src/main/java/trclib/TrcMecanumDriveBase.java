/*
 * Copyright (c) 2018 Titan Robotics Club (http://www.titanrobotics.com)
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

package trclib;

/**
 * This class implements a platform independent mecanum drive base. A mecanum drive base consists of 4 motor driven
 * wheels. It extends the TrcSimpleDriveBase class so it inherits all the SimpleDriveBase methods and features.
 */
public class TrcMecanumDriveBase extends TrcSimpleDriveBase
{
    /**
     * Constructor: Create an instance of the 4-wheel mecanum drive base.
     *
     * @param leftFrontMotor specifies the left front motor of the drive base.
     * @param leftRearMotor specifies the left rear motor of the drive base.
     * @param rightFrontMotor specifies the right front motor of the drive base.
     * @param rightRearMotor specifies the right rear motor of the drive base.
     * @param gyro specifies the gyro. If none, it can be set to null.
     */
    public TrcMecanumDriveBase(
        TrcMotorController leftFrontMotor, TrcMotorController leftRearMotor,
        TrcMotorController rightFrontMotor, TrcMotorController rightRearMotor,
        TrcGyro gyro)
    {
        super(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor, gyro);
    }   //TrcMecanumDriveBase

    /**
     * Constructor: Create an instance of the 4-wheel mecanum drive base.
     *
     * @param leftFrontMotor specifies the left front motor of the drive base.
     * @param leftRearMotor specifies the left rear motor of the drive base.
     * @param rightFrontMotor specifies the right front motor of the drive base.
     * @param rightRearMotor specifies the right rear motor of the drive base.
     */
    public TrcMecanumDriveBase(
        TrcMotorController leftFrontMotor, TrcMotorController leftRearMotor,
        TrcMotorController rightFrontMotor, TrcMotorController rightRearMotor)
    {
        super(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor, null);
    }   //TrcMecanumDriveBase

    /**
     * This method checks if it supports holonomic drive.
     *
     * @return true if this drive base supports holonomic drive, false otherwise.
     */
    @Override
    public boolean supportsHolonomicDrive()
    {
        return true;
    }   //supportsHolonomicDrive

    /**
     * This method implements holonomic drive where x controls how fast the robot will go in the x direction, and y
     * controls how fast the robot will go in the y direction. Rotation controls how fast the robot rotates and
     * gyroAngle specifies the heading the robot should maintain.
     *
     * @param x specifies the x power.
     * @param y specifies the y power.
     * @param rotation specifies the rotating power.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     * @param gyroAngle specifies the gyro angle to maintain.
     */
    @Override
    protected void holonomicDrive(double x, double y, double rotation, boolean inverted, double gyroAngle)
    {
        final String funcName = "holonomicDrive";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "x=%f,y=%f,rot=%f,inverted=%s,angle=%f",
                                x, y, rotation, Boolean.toString(inverted), gyroAngle);
        }

        x = TrcUtil.clipRange(x);
        y = TrcUtil.clipRange(y);
        rotation = TrcUtil.clipRange(rotation);

        if (inverted)
        {
            x = -x;
            y = -y;
        }

        double cosA = Math.cos(Math.toRadians(gyroAngle));
        double sinA = Math.sin(Math.toRadians(gyroAngle));
        double x1 = x*cosA - y*sinA;
        double y1 = x*sinA + y*cosA;

        if (isGyroAssistEnabled())
        {
            rotation += getGyroAssistPower(rotation);
        }

        double wheelPowers[] = new double[4];
        wheelPowers[MotorType.LEFT_FRONT.value] = x1 + y1 + rotation;
        wheelPowers[MotorType.RIGHT_FRONT.value] = -x1 + y1 - rotation;
        wheelPowers[MotorType.LEFT_REAR.value] = -x1 + y1 + rotation;
        wheelPowers[MotorType.RIGHT_REAR.value] = x1 + y1 - rotation;
        TrcUtil.normalizeInPlace(wheelPowers);

        double wheelPower;

        wheelPower = wheelPowers[MotorType.LEFT_FRONT.value];
        if (motorPowerMapper != null)
        {
            wheelPower = motorPowerMapper.translateMotorPower(wheelPower, leftFrontMotor.getVelocity());
        }
        leftFrontMotor.set(wheelPower);

        wheelPower = wheelPowers[MotorType.RIGHT_FRONT.value];
        if (motorPowerMapper != null)
        {
            wheelPower = motorPowerMapper.translateMotorPower(wheelPower, rightFrontMotor.getVelocity());
        }
        rightFrontMotor.set(wheelPower);

        wheelPower = wheelPowers[MotorType.LEFT_REAR.value];
        if (motorPowerMapper != null)
        {
            wheelPower = motorPowerMapper.translateMotorPower(wheelPower, leftRearMotor.getVelocity());
        }
        leftRearMotor.set(wheelPower);

        wheelPower = wheelPowers[MotorType.RIGHT_REAR.value];
        if (motorPowerMapper != null)
        {
            wheelPower = motorPowerMapper.translateMotorPower(wheelPower, rightRearMotor.getVelocity());
        }
        rightRearMotor.set(wheelPower);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //holonomicDrive

    /**
     * This method is called periodically to monitor the position sensors to update the odometry data. It assumes the
     * caller has the odometry lock.
     *
     * @param odometry specifies the odometry object to be updated.
     */
    @Override
    protected void updateOdometry(Odometry odometry)
    {
        //
        // Call super class to update Y and rotation odometry.
        //
        super.updateOdometry(odometry);
        //
        // According to RobotDrive.mecanumDrive_Cartesian in WPILib:
        //
        // LF =  x + y + rot    RF = -x + y - rot
        // LR = -x + y + rot    RR =  x + y - rot
        //
        // (LF + RR) - (RF + LR) = (2x + 2y) - (-2x + 2y)
        // => (LF + RR) - (RF + LR) = 4x
        // => x = ((LF + RR) - (RF + LR))/4
        //
        // LF + RF + LR + RR = 4y
        // => y = (LF + RF + LR + RR)/4
        //
        // (LF + LR) - (RF + RR) = (2y + 2rot) - (2y - 2rot)
        // => (LF + LR) - (RF + RR) = 4rot
        // => rot = ((LF + LR) - (RF + RR))/4
        //
        odometry.xRawPos = TrcUtil.average(
                odometry.currPositions[MotorType.LEFT_FRONT.value],
                odometry.currPositions[MotorType.RIGHT_REAR.value],
                -odometry.currPositions[MotorType.RIGHT_FRONT.value],
                -odometry.currPositions[MotorType.LEFT_REAR.value]);
        odometry.xRawVel = TrcUtil.average(
                odometry.currVelocities[MotorType.LEFT_FRONT.value],
                odometry.currVelocities[MotorType.RIGHT_REAR.value],
                -odometry.currVelocities[MotorType.RIGHT_FRONT.value],
                -odometry.currVelocities[MotorType.LEFT_REAR.value]);
    }   //updateOdometry

}   //class TrcMecanumDriveBase
