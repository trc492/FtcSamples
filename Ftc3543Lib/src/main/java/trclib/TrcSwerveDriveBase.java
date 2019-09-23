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
    private final TrcSwerveModule lfModule, rfModule, lrModule, rrModule;
    private final double wheelBaseWidth, wheelBaseLength, wheelBaseDiagonal;
    private double[] lastWheelPos = new double[4];

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
     * scale factor, one could make getPosition to return unit in inches, for example.
     *
     * @param xScale   specifies the X position scale.
     * @param yScale   specifies the Y position scale.
     * @param rotScale specifies the rotation scale.
     */
    @Override
    public void setPositionScales(double xScale, double yScale, double rotScale)
    {
        if (xScale != yScale)
        {
            throw new IllegalArgumentException("Swerve does not have different x and y scales!");
        }

        super.setPositionScales(xScale, yScale, rotScale);
    }   //setPositionScales

    /**
     * This method sets the position scales. The raw position from the encoder is in encoder counts. By setting the
     * scale factor, one could make getPosition to return unit in inches, for example. This also automatically
     * calculates the rotateScale, which is used for approximating the heading without the gyro.
     *
     * @param scale specifies the position scale for each motor.
     */
    @Override
    public void setPositionScales(double scale)
    {
        super.setPositionScales(scale, scale, 1.0);
    }   //setPositionScales

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
            lrModule.setSteerAngle(angle, optimize);
            rrModule.setSteerAngle(angle, optimize);
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
                lrModule.set(0.0);
                rrModule.set(0.0);

                lfModule.setSteerAngle(lfModule.getSteerAngle());
                rfModule.setSteerAngle(rfModule.getSteerAngle());
                lrModule.setSteerAngle(lrModule.getSteerAngle());
                rrModule.setSteerAngle(rrModule.getSteerAngle());
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
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //holonomicDrive

    /**
     * This method is called periodically to monitor the position sensors for calculating the delta from the previous
     * position.
     *
     * @param motorsState specifies the state information of the drivebase motors for calculating pose delta.
     * @return a TrcPose2D object describing the change in position since the last call.
     */
    @Override
    protected TrcPose2D getPoseDelta(MotorsState motorsState)
    {
        final String funcName = "getPoseDelta";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK);
        }

        TrcSwerveModule[] modules = new TrcSwerveModule[] { lfModule, rfModule, lrModule, rrModule };
        RealVector[] wheelVectors = new RealVector[4];
        RealVector[] wheelVelocities = new RealVector[4];

        for (int i = 0; i < modules.length; i++)
        {
            double angle = modules[i].getSteerAngle();
            double pos = modules[i].getPosition();
            double posDiff = pos - lastWheelPos[i];

            lastWheelPos[i] = pos;
            wheelVectors[i] = TrcUtil.polarToCartesian(posDiff, angle).mapMultiply(xScale); // x and y scales are same
            wheelVelocities[i] = TrcUtil.polarToCartesian(modules[i].getVelocity(), angle).mapMultiply(xScale);
        }

        RealVector posSum = new ArrayRealVector(2);
        RealVector velSum = new ArrayRealVector(2);

        for (int i = 0; i < 4; i++)
        {
            posSum = posSum.add(wheelVectors[i]);
            velSum = velSum.add(wheelVelocities[i]);
        }

        posSum.mapMultiplyToSelf(0.25);
        velSum.mapMultiplyToSelf(0.25);

        TrcPose2D poseDelta = new TrcPose2D();

        poseDelta.x = posSum.getEntry(0);
        poseDelta.y = posSum.getEntry(1);

        poseDelta.xVel = velSum.getEntry(0);
        poseDelta.yVel = velSum.getEntry(1);

        double x = wheelBaseWidth / 2;
        double y = wheelBaseLength / 2;
        // This is black magic math, and it actually needs to be tested.
        double dRot = x * (wheelVectors[0].getEntry(1) + wheelVectors[2].getEntry(1) -
                           wheelVectors[1].getEntry(1) - wheelVectors[3].getEntry(1)) +
                      y * (wheelVectors[0].getEntry(0) + wheelVectors[1].getEntry(0) -
                           wheelVectors[2].getEntry(0) - wheelVectors[3].getEntry(0));

        dRot /= 4 * Math.pow(wheelBaseDiagonal, 2);
        dRot = Math.toDegrees(dRot);
        poseDelta.heading = dRot;

        double rotVel = x * (wheelVelocities[0].getEntry(1) + wheelVelocities[2].getEntry(1) -
                             wheelVelocities[1].getEntry(1) - wheelVelocities[3].getEntry(1)) +
                        y * (wheelVelocities[0].getEntry(0) + wheelVelocities[1].getEntry(0) -
                             wheelVelocities[2].getEntry(0) - wheelVelocities[3].getEntry(0));
        rotVel /= 4 * Math.pow(wheelBaseDiagonal, 2);
        rotVel = Math.toDegrees(rotVel);
        poseDelta.turnRate = rotVel;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }

        return poseDelta;
    }   //getPoseDelta

}   //class TrcSwerveDriveBase
