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

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

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
    private final TrcSwerveModule lfModule, rfModule, lbModule, rbModule;
    private final double wheelBaseWidth, wheelBaseLength, wheelBaseDiagonal;
    private double[] lastWheelPos = new double[4];

    /**
     * Constructor: Create an instance of the 4-wheel swerve drive base.
     *
     * @param leftFrontMotor specifies the left front motor of the drive base.
     * @param leftBackMotor specifies the left back motor of the drive base.
     * @param rightFrontMotor specifies the right front motor of the drive base.
     * @param rightBackMotor specifies the right back motor of the drive base.
     * @param gyro specifies the gyro. If none, it can be set to null.
     * @param wheelBaseWidth specifies the width of the wheel base in inches.
     * @param wheelBaseLength specifies the length of the wheel base in inches.
     */
    public TrcSwerveDriveBase(
        TrcSwerveModule leftFrontMotor, TrcSwerveModule leftBackMotor,
        TrcSwerveModule rightFrontMotor, TrcSwerveModule rightBackMotor,
        TrcGyro gyro, double wheelBaseWidth, double wheelBaseLength)
    {
        super(leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor, gyro);

        this.lfModule = leftFrontMotor;
        this.rfModule = rightFrontMotor;
        this.lbModule = leftBackMotor;
        this.rbModule = rightBackMotor;
        this.wheelBaseWidth = wheelBaseWidth;
        this.wheelBaseLength = wheelBaseLength;
        this.wheelBaseDiagonal = TrcUtil.magnitude(wheelBaseWidth, wheelBaseLength);
    }   //TrcSwerveDriveBase

    /**
     * Constructor: Create an instance of the 4-wheel swerve drive base.
     *
     * @param leftFrontMotor specifies the left front motor of the drive base.
     * @param leftBackMotor specifies the left back motor of the drive base.
     * @param rightFrontMotor specifies the right front motor of the drive base.
     * @param rightBackMotor specifies the right back motor of the drive base.
     * @param wheelBaseWidth specifies the width of the wheel base in inches.
     * @param wheelBaseLength specifies the length of the wheel base in inches.
     */
    public TrcSwerveDriveBase(
        TrcSwerveModule leftFrontMotor, TrcSwerveModule leftBackMotor,
        TrcSwerveModule rightFrontMotor, TrcSwerveModule rightBackMotor,
        double wheelBaseWidth, double wheelBaseLength)
    {
        this(leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor, null, wheelBaseWidth, wheelBaseLength);
    }   //TrcSwerveDriveBase

    /**
     * This method does zero calibration on the steer angle encoders.
     */
    public void zeroCalibrateSteering()
    {
        lfModule.zeroCalibrateSteering();
        rfModule.zeroCalibrateSteering();
        lbModule.zeroCalibrateSteering();
        rbModule.zeroCalibrateSteering();
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
     * This method sets the odometry scales. The raw position from the encoder is in encoder counts. By setting the
     * scale factor, one could make getPosition to return unit in inches, for example.
     *
     * @param xScale     specifies the X position scale.
     * @param yScale     specifies the Y position scale.
     * @param angleScale specifies the angle scale.
     */
    @Override
    public void setOdometryScales(double xScale, double yScale, double angleScale)
    {
        if (xScale != yScale)
        {
            throw new IllegalArgumentException("Swerve does not have different x and y scales!");
        }

        super.setOdometryScales(xScale, yScale, angleScale);
    }   //setOdometryScales

    /**
     * This method sets the odometry scales. The raw position from the encoder is in encoder counts. By setting the
     * scale factor, one could make getPosition to return unit in inches, for example.
     *
     * @param scale specifies the position scale for each motor.
     */
    @Override
    public void setOdometryScales(double scale)
    {
        super.setOdometryScales(scale, scale, 1.0);
    }   //setOdometryScales

    /**
     * This method sets the steering angle of all four wheels.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *              ownership aware.
     * @param angle specifies the steering angle to be set.
     * @param optimize specifies true to optimize steering angle to be no greater than 90 degrees, false otherwise.
     */
    public void setSteerAngle(String owner, double angle, boolean optimize)
    {
        final String funcName = "setSteerAngle";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                "owner=%s,angle=%f,optimize=%s", owner, angle, optimize);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (validateOwnership(owner))
        {
            lfModule.setSteerAngle(angle, optimize);
            rfModule.setSteerAngle(angle, optimize);
            lbModule.setSteerAngle(angle, optimize);
            rbModule.setSteerAngle(angle, optimize);
        }
    }   //setSteerAngle

    /**
     * This method sets the steering angle of all four wheels.
     *
     * @param angle specifies the steering angle to be set.
     * @param optimize specifies true to optimize steering angle to be no greater than 90 degrees, false otherwise.
     */
    public void setSteerAngle(double angle, boolean optimize)
    {
        setSteerAngle(null, angle, optimize);
    }   //setSteerAngle

    /**
     * This method sets the steering angle of all four wheels.
     *
     * @param angle specifies the steering angle to be set.
     */
    public void setSteerAngle(double angle)
    {
        setSteerAngle(null, angle, true);
    }   //setSteerAngle

    /**
     * This method stops the drive base. If steerNeutral is true, it also sets all steering angles to zero.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *              ownership aware.
     * @param resetSteer specifies true to set steering angle to zero.
     */
    public void stop(String owner, boolean resetSteer)
    {
        final String funcName = "stop";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "owner=%s,steerNeutral=%s", owner, resetSteer);
        }

        if (validateOwnership(owner))
        {
            super.stop(owner);

            if (resetSteer)
            {
                setSteerAngle(0.0, false);
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //stop

    /**
     * This method stops the drive base. If steerNeutral is true, it also sets all steering angles to zero.
     *
     * @param resetSteer specifies true to set steering angle to zero.
     */
    public void stop(boolean resetSteer)
    {
        stop(null, resetSteer);
    }   //stop

    /**
     * This method stops the drive base and reset the steering angle to zero.
     */
    @Override
    public void stop()
    {
        stop(null, false);
    }   //stop

    /**
     * This method implements tank drive where leftPower controls the left motors and right power controls the right
     * motors. It will set the steering angle to zero, but note that it will take time for the steering angles to
     * reach zero. Since we do not wait for the steering angle to reach neutral, it is possible the drive base will
     * move diagonally initially. If this is undesirable, the caller should make sure steering angles are already at
     * zero before calling this method.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *              ownership aware.
     * @param leftPower specifies left power value.
     * @param rightPower specifies right power value.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     */
    @Override
    public void tankDrive(String owner, double leftPower, double rightPower, boolean inverted)
    {
        setSteerAngle(owner, 0.0, false);
        super.tankDrive(owner, leftPower, rightPower, inverted);
    }   //tankDrive

    /**
     * This method implements holonomic drive where x controls how fast the robot will go in the x direction, and y
     * controls how fast the robot will go in the y direction. Rotation controls how fast the robot rotates and
     * gyroAngle specifies the heading the robot should maintain.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *              ownership aware.
     * @param x specifies the x power.
     * @param y specifies the y power.
     * @param rotation specifies the rotating power.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     * @param gyroAngle specifies the gyro angle to maintain for field relative drive. DO NOT use this with inverted.
     */
    @Override
    protected void holonomicDrive(String owner, double x, double y, double rotation, boolean inverted, double gyroAngle)
    {
        final String funcName = "holonomicDrive";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "owner=%s, x=%f,y=%f,rot=%f,inverted=%s,angle=%f",
                                owner, x, y, rotation, Boolean.toString(inverted), gyroAngle);
        }

        if (validateOwnership(owner))
        {
            if (x == 0.0 && y == 0.0 && rotation == 0.0)
            {
                lfModule.set(0.0);
                rfModule.set(0.0);
                lbModule.set(0.0);
                rbModule.set(0.0);

                lfModule.setSteerAngle(lfModule.getSteerAngle());
                rfModule.setSteerAngle(rfModule.getSteerAngle());
                lbModule.setSteerAngle(lbModule.getSteerAngle());
                rbModule.setSteerAngle(rbModule.getSteerAngle());
            }
            else
            {
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

                // The white paper goes in order rf, lf, lb, rb. We like to do lf, rf, lb, rb.
                // Note: atan2(y, x) in java will take care of x being zero.
                //       If will return pi/2 for positive y and -pi/2 for negative y.
                double lfAngle = Math.toDegrees(Math.atan2(b, d));
                double rfAngle = Math.toDegrees(Math.atan2(b, c));
                double lbAngle = Math.toDegrees(Math.atan2(a, d));
                double rbAngle = Math.toDegrees(Math.atan2(a, c));

                // The white paper goes in order rf, lf, lb, rb. We like to do lf, rf, lb, rb.
                double lfPower = TrcUtil.magnitude(b, d);
                double rfPower = TrcUtil.magnitude(b, c);
                double lbPower = TrcUtil.magnitude(a, d);
                double rbPower = TrcUtil.magnitude(a, c);

                double[] normalizedPowers = TrcUtil.normalize(lfPower, rfPower, lbPower, rbPower);
                lfPower = this.clipMotorOutput(normalizedPowers[0]);
                rfPower = this.clipMotorOutput(normalizedPowers[1]);
                lbPower = this.clipMotorOutput(normalizedPowers[2]);
                rbPower = this.clipMotorOutput(normalizedPowers[3]);

                if (motorPowerMapper != null)
                {
                    lfPower = motorPowerMapper.translateMotorPower(lfPower, lfModule.getVelocity());
                    rfPower = motorPowerMapper.translateMotorPower(rfPower, rfModule.getVelocity());
                    lbPower = motorPowerMapper.translateMotorPower(lbPower, lbModule.getVelocity());
                    rbPower = motorPowerMapper.translateMotorPower(rbPower, rbModule.getVelocity());
                }

                lfModule.setSteerAngle(lfAngle);
                rfModule.setSteerAngle(rfAngle);
                lbModule.setSteerAngle(lbAngle);
                rbModule.setSteerAngle(rbAngle);

                lfModule.set(lfPower);
                rfModule.set(rfPower);
                lbModule.set(lbPower);
                rbModule.set(rbPower);
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //holonomicDrive

    /**
     * This method is called periodically to calculate the delta between the previous and current motor odometries.
     *
     * @param prevOdometries specifies the previous motor odometries.
     * @param currOdometries specifies the current motor odometries.
     * @return an Odometry object describing the odometry changes since the last update.
     */
    @Override
    protected Odometry getOdometryDelta(
            TrcOdometrySensor.Odometry prevOdometries[], TrcOdometrySensor.Odometry currOdometries[])
    {
        final String funcName = "getOdometryDelta";
        Odometry delta = new Odometry();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
        //
        // Average the posDelta vectors and velocity vectors of all four wheels:
        //  (sum posDelta vectors of all wheels)/num_of_wheels
        //  (sum velocity vectors of all wheels)/num_of_wheels
        //
        int numMotors = currOdometries.length;
        RealVector[] wheelPosVectors = new RealVector[numMotors];
        RealVector[] wheelVelVectors = new RealVector[numMotors];
        RealVector posSum = new ArrayRealVector(2);
        RealVector velSum = new ArrayRealVector(2);
        for (int i = 0; i < numMotors; i++)
        {
            double angle = ((TrcSwerveModule) currOdometries[i].sensor).getSteerAngle();
            double posDelta = currOdometries[i].currPos - prevOdometries[i].currPos;
            // xScale and yScale on SwerveDrive should be identical.
            wheelPosVectors[i] = TrcUtil.polarToCartesian(posDelta, angle).mapMultiply(xScale);
            wheelVelVectors[i] =
                    TrcUtil.polarToCartesian(currOdometries[i].velocity, angle).mapMultiply(xScale);
            posSum = posSum.add(wheelPosVectors[i]);
            velSum = velSum.add(wheelVelVectors[i]);
        }
        double multiplier = 1 / numMotors;
        posSum.mapMultiplyToSelf(multiplier);
        velSum.mapMultiplyToSelf(multiplier);
        //
        // Calculate the odometry delta.
        //
        delta.position.x = posSum.getEntry(0);
        delta.position.y = posSum.getEntry(1);

        delta.velocity.x = velSum.getEntry(0);
        delta.velocity.y = velSum.getEntry(1);

        double x = wheelBaseWidth / 2;
        double y = wheelBaseLength / 2;
        // This is black magic math, and it actually needs to be tested.
        // CodeReview: Please put a reference to your research material so we know where it came from.
        double dRot = x * (wheelPosVectors[0].getEntry(1) + wheelPosVectors[2].getEntry(1) -
                           wheelPosVectors[1].getEntry(1) - wheelPosVectors[3].getEntry(1)) +
                      y * (wheelPosVectors[0].getEntry(0) + wheelPosVectors[1].getEntry(0) -
                           wheelPosVectors[2].getEntry(0) - wheelPosVectors[3].getEntry(0));

        dRot /= 4 * Math.pow(wheelBaseDiagonal, 2);
        dRot = Math.toDegrees(dRot);
        delta.position.angle = dRot;

        double rotVel = x * (wheelVelVectors[0].getEntry(1) + wheelVelVectors[2].getEntry(1) -
                             wheelVelVectors[1].getEntry(1) - wheelVelVectors[3].getEntry(1)) +
                        y * (wheelVelVectors[0].getEntry(0) + wheelVelVectors[1].getEntry(0) -
                             wheelVelVectors[2].getEntry(0) - wheelVelVectors[3].getEntry(0));
        rotVel /= 4 * Math.pow(wheelBaseDiagonal, 2);
        rotVel = Math.toDegrees(rotVel);
        delta.velocity.angle = rotVel;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }

        return delta;
    }   //getOdometryDelta

}   //class TrcSwerveDriveBase
