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
 * This class implements a platform independent swerve drive base. A swerve drive base consists of 4 swerve modules
 * each of which consists of a driving motor and a PID controlled steering motor. It extends the TrcSimpleDriveBase
 * class so it inherits all the SimpleDriveBase methods and features
 *
 * The implementation of swerve algorithm is based on Ether's white paper:
 *  http://www.chiefdelphi.com/media/papers/download/3028
 */
public class TrcSwerveDriveBase extends TrcSimpleDriveBase
{
    private final TrcSwerveModule lfModule, rfModule, lrModule, rrModule;
    private final double wheelBaseWidth, wheelBaseLength, wheelBaseDiagonal;

    /**
     * Constructor: Create an instance of the 4-wheel swerve drive base.
     *
     * @param leftFrontMotor specifies the left front motor of the drive base.
     * @param leftRearMotor specifies the left rear motor of the drive base.
     * @param rightFrontMotor specifies the right front motor of the drive base.
     * @param rightRearMotor specifies the right rear motor of the drive base.
     * @param gyro specifies the gyro. If none, it can be set to null.
     * @param wheelBaseWidth specifies the width of the wheel base in inches.
     * @param wheelBaseLength specifies the length of the wheel base in inches.
     */
    public TrcSwerveDriveBase(
        TrcSwerveModule leftFrontMotor, TrcSwerveModule leftRearMotor,
        TrcSwerveModule rightFrontMotor, TrcSwerveModule rightRearMotor,
        TrcGyro gyro, double wheelBaseWidth, double wheelBaseLength)
    {
        super(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor, gyro);

        this.lfModule = leftFrontMotor;
        this.rfModule = rightFrontMotor;
        this.lrModule = leftRearMotor;
        this.rrModule = rightRearMotor;
        this.wheelBaseWidth = wheelBaseWidth;
        this.wheelBaseLength = wheelBaseLength;
        this.wheelBaseDiagonal = TrcUtil.magnitude(wheelBaseWidth, wheelBaseLength);
    }   //TrcSwerveDriveBase

    /**
     * Constructor: Create an instance of the 4-wheel swerve drive base.
     *
     * @param leftFrontMotor specifies the left front motor of the drive base.
     * @param leftRearMotor specifies the left rear motor of the drive base.
     * @param rightFrontMotor specifies the right front motor of the drive base.
     * @param rightRearMotor specifies the right rear motor of the drive base.
     * @param wheelBaseWidth specifies the width of the wheel base in inches.
     * @param wheelBaseLength specifies the length of the wheel base in inches.
     */
    public TrcSwerveDriveBase(
        TrcSwerveModule leftFrontMotor, TrcSwerveModule leftRearMotor,
        TrcSwerveModule rightFrontMotor, TrcSwerveModule rightRearMotor,
        double wheelBaseWidth, double wheelBaseLength)
    {
        this(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor, null, wheelBaseWidth, wheelBaseLength);
    }   //TrcSwerveDriveBase

    /**
     * This method does zero calibration on the steer angle encoders.
     */
    public void zeroCalibrateSteering()
    {
        lfModule.zeroCalibrateSteering();
        rfModule.zeroCalibrateSteering();
        lrModule.zeroCalibrateSteering();
        rrModule.zeroCalibrateSteering();
    }   //zeroCalibrateSteering

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
     * This method sets the position scales. The raw position from the encoder is in encoder counts. By setting the
     * scale factor, one could make getPosition to return unit in inches, for example. This also automatically
     * calculates the rotateScale, which is used for approximating the heading without the gyro.
     *
     * @param xScale specifies the X position scale.
     * @param yScale specifies the Y position scale.
     */
    @Override
    public void setPositionScales(double xScale, double yScale)
    {
        // encDist / perimeter = rotPos / 360.0
        // encDist * 360.0 / perimeter = rotPos
        // Therefore, rotScale = 360.0 / perimeter
        double perimeter = wheelBaseDiagonal * Math.PI;
        double rotScale = 360.0 / perimeter;

        super.setPositionScales(xScale, yScale, rotScale);
    }   //setPositionScales

    /**
     * This method sets the steering angle of all four wheels.
     *
     * @param angle specifies the steering angle to be set.
     * @param optimize specifies true to optimize steering angle to be no greater than 90 degrees, false otherwise.
     */
    public void setSteerAngle(double angle, boolean optimize)
    {
        final String funcName = "setSteerAngle";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "angle=%f,optimize=%s", angle, optimize);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        lfModule.setSteerAngle(angle, optimize);
        rfModule.setSteerAngle(angle, optimize);
        lrModule.setSteerAngle(angle, optimize);
        rrModule.setSteerAngle(angle, optimize);
    }   //setSteerAngle

    /**
     * This method sets the steering angle of all four wheels.
     *
     * @param angle specifies the steering angle to be set.
     */
    public void setSteerAngle(double angle)
    {
        setSteerAngle(angle, true);
    }   //setSteerAngle

    /**
     * This method stops the drive base. If steerNeutral is true, it also sets all steering angles to zero.
     *
     * @param resetSteer specifies true to set steering angle to zero.
     */
    public void stop(boolean resetSteer)
    {
        final String funcName = "stop";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "steerNeutral=%s", resetSteer);
        }

        super.stop();

        if (resetSteer)
        {
            setSteerAngle(0.0, false);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //stop

    /**
     * This method stops the drive base and reset the steering angle to zero.
     */
    @Override
    public void stop()
    {
        stop(false);
    }   //stop

    /**
     * This method implements tank drive where leftPower controls the left motors and right power controls the right
     * motors. It will set the steering angle to zero, but note that it will take time for the steering angles to
     * reach zero. Since we do not wait for the steering angle to reach neutral, it is possible the drive base will
     * move diagonally initially. If this is undesirable, the caller should make sure steering angles are already at
     * zero before calling this method.
     *
     * @param leftPower specifies left power value.
     * @param rightPower specifies right power value.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     */
    @Override
    public void tankDrive(double leftPower, double rightPower, boolean inverted)
    {
        setSteerAngle(0.0, false);
        super.tankDrive(leftPower, rightPower, inverted);
    }   //tankDrive

    /**
     * This method implements holonomic drive where x controls how fast the robot will go in the x direction, and y
     * controls how fast the robot will go in the y direction. Rotation controls how fast the robot rotates and
     * gyroAngle specifies the heading the robot should maintain.
     *
     * @param x specifies the x power.
     * @param y specifies the y power.
     * @param rotation specifies the rotating power.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     * @param gyroAngle specifies the gyro angle to maintain for field relative drive. DO NOT use this with inverted.
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

        if(inverted)
        {
            x = -x;
            y = -y;
        }

        if(gyroAngle != 0)
        {
            if(inverted)
            {
                globalTracer.traceWarn(
                    funcName, "You should not be using inverted and field reference frame at the same time!");
            }

            double gyroRadians = Math.toRadians(gyroAngle);
            double temp = y * Math.cos(gyroRadians) + x * Math.sin(gyroRadians);
            x = -y * Math.sin(gyroRadians) + x * Math.cos(gyroRadians);
            y = temp;
        }

        double a = x - (rotation * wheelBaseLength/wheelBaseDiagonal);
        double b = x + (rotation * wheelBaseLength/wheelBaseDiagonal);
        double c = y - (rotation * wheelBaseWidth/wheelBaseDiagonal);
        double d = y + (rotation * wheelBaseWidth/wheelBaseDiagonal);

        // The white paper goes in order rf, lf, lr, rr. We like to do lf, rf, lr, rr.
        // Note: atan2(y, x) in java will take care of x being zero.
        //       If will return pi/2 for positive y and -pi/2 for negative y.
        double lfAngle = Math.toDegrees(Math.atan2(b, d));
        double rfAngle = Math.toDegrees(Math.atan2(b, c));
        double lrAngle = Math.toDegrees(Math.atan2(a, d));
        double rrAngle = Math.toDegrees(Math.atan2(a, c));

        // The white paper goes in order rf, lf, lr, rr. We like to do lf, rf, lr, rr.
        double lfPower = TrcUtil.magnitude(b, d);
        double rfPower = TrcUtil.magnitude(b, c);
        double lrPower = TrcUtil.magnitude(a, d);
        double rrPower = TrcUtil.magnitude(a, c);

        double[] normalizedPowers = TrcUtil.normalize(lfPower, rfPower, lrPower, rrPower);
        lfPower = this.clipMotorOutput(normalizedPowers[0]);
        rfPower = this.clipMotorOutput(normalizedPowers[1]);
        lrPower = this.clipMotorOutput(normalizedPowers[2]);
        rrPower = this.clipMotorOutput(normalizedPowers[3]);

        if (motorPowerMapper != null)
        {
            lfPower = motorPowerMapper.translateMotorPower(lfPower, lfModule.getVelocity());
            rfPower = motorPowerMapper.translateMotorPower(rfPower, rfModule.getVelocity());
            lrPower = motorPowerMapper.translateMotorPower(lrPower, lrModule.getVelocity());
            rrPower = motorPowerMapper.translateMotorPower(rrPower, rrModule.getVelocity());
        }

        lfModule.setSteerAngle(lfAngle);
        rfModule.setSteerAngle(rfAngle);
        lrModule.setSteerAngle(lrAngle);
        rrModule.setSteerAngle(rrAngle);

        lfModule.set(lfPower);
        rfModule.set(rfPower);
        lrModule.set(lrPower);
        rrModule.set(rrPower);

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
        final String funcName = "updateOdometry";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK);
        }

        double lfAngle = lfModule.getSteerAngle();
        double rfAngle = rfModule.getSteerAngle();
        double lrAngle = lrModule.getSteerAngle();
        double rrAngle = rrModule.getSteerAngle();

        double avgAngleDeg = TrcUtil.average(lfAngle, rfAngle, lrAngle, rrAngle);
        double avgAngleRad = Math.toRadians(avgAngleDeg);
        double angleCos = Math.cos(avgAngleRad);
        double angleSin = Math.sin(avgAngleRad);

        double timeDelta = odometry.currTimestamp - odometry.prevTimestamp;
        double avgEncDelta = TrcUtil.average(
                odometry.currPositions[MotorType.LEFT_FRONT.value] - odometry.prevPositions[MotorType.LEFT_FRONT.value],
                odometry.currPositions[MotorType.RIGHT_FRONT.value] - odometry.prevPositions[MotorType.RIGHT_FRONT.value],
                odometry.currPositions[MotorType.LEFT_REAR.value] - odometry.prevPositions[MotorType.LEFT_REAR.value],
                odometry.currPositions[MotorType.RIGHT_REAR.value] - odometry.prevPositions[MotorType.RIGHT_REAR.value]);
        double avgEncVel = timeDelta != 0.0 ? avgEncDelta / timeDelta : 0.0;

        odometry.xRawPos = getRawXPosition() + avgEncDelta*angleCos;
        odometry.xRawVel = avgEncVel*angleCos;
        odometry.yRawPos = getRawYPosition() + avgEncDelta*angleSin;
        odometry.yRawVel = avgEncVel*angleSin;
        //
        // Rotation position is only valid when the robot is doing turn-in-place.
        // In Swerve Drive, the wheels are steered in a diamond formation (i.e. tangential to the turning circle).
        // So the rotation position is the degree turned by the robot in the turning circle.
        //
        odometry.rotRawPos = getRawRotationPosition() + avgEncDelta;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //updateOdometry

}   //class TrcSwerveDriveBase
