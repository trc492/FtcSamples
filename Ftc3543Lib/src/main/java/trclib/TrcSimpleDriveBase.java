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
 * This class implements a platform independent simple drive base. The SimpleDriveBase class implements a drive train
 * that may consist of 2 to 6 motors. It supports tank drive, curve drive and arcade drive with motor stalled detection
 * and inverted drive mode. It also supports gyro assisted drive to keep robot driving straight.
 */
public class TrcSimpleDriveBase extends TrcDriveBase
{
    public enum MotorType
    {
        LEFT_FRONT(0),
        RIGHT_FRONT(1),
        LEFT_REAR(2),
        RIGHT_REAR(3),
        LEFT_MID(4),
        RIGHT_MID(5);

        public final int value;

        MotorType(int value)
        {
            this.value = value;
        }
    }   //enum MotorType

    protected final TrcMotorController leftFrontMotor;
    protected final TrcMotorController rightFrontMotor;
    protected final TrcMotorController leftRearMotor;
    protected final TrcMotorController rightRearMotor;
    protected final TrcMotorController leftMidMotor;
    protected final TrcMotorController rightMidMotor;

    /**
     * Constructor: Create an instance of a 6-wheel drive base.
     *
     * @param leftFrontMotor specifies the left front motor of the drive base.
     * @param leftMidMotor specifies the left mid motor of a 6-wheel drive base.
     * @param leftRearMotor specifies the left rear motor of the drive base.
     * @param rightFrontMotor specifies the right front motor of the drive base.
     * @param rightMidMotor specifies the right mid motor of a 6-wheel drive base.
     * @param rightRearMotor specifies the right rear motor of the drive base.
     * @param gyro specifies the gyro. If none, it can be set to null.
     */
    public TrcSimpleDriveBase(
        TrcMotorController leftFrontMotor, TrcMotorController leftMidMotor, TrcMotorController leftRearMotor,
        TrcMotorController rightFrontMotor, TrcMotorController rightMidMotor, TrcMotorController rightRearMotor,
        TrcGyro gyro)
    {
        super(new TrcMotorController[]
                {leftFrontMotor, rightFrontMotor, leftRearMotor, rightRearMotor, leftMidMotor, rightMidMotor},
              gyro);

        if (leftFrontMotor == null || rightFrontMotor == null ||
            leftRearMotor == null || rightRearMotor == null ||
            leftMidMotor == null || rightMidMotor == null)
        {
            throw new IllegalArgumentException("All 6 motors must not be null.");
        }

        this.leftFrontMotor = leftFrontMotor;
        this.rightFrontMotor = rightFrontMotor;
        this.leftRearMotor = leftRearMotor;
        this.rightRearMotor = rightRearMotor;
        this.leftMidMotor = leftMidMotor;
        this.rightMidMotor = rightMidMotor;
    }   //TrcSimpleDriveBase

    /**
     * Constructor: Create an instance of a 6-wheel drive base.
     *
     * @param leftFrontMotor specifies the left front motor of the drive base.
     * @param leftMidMotor specifies the left mid motor of a 6-wheel drive base.
     * @param leftRearMotor specifies the left rear motor of the drive base.
     * @param rightFrontMotor specifies the right front motor of the drive base.
     * @param rightMidMotor specifies the right mid motor of a 6-wheel drive base.
     * @param rightRearMotor specifies the right rear motor of the drive base.
     */
    public TrcSimpleDriveBase(
        TrcMotorController leftFrontMotor, TrcMotorController leftMidMotor, TrcMotorController leftRearMotor,
        TrcMotorController rightFrontMotor, TrcMotorController rightMidMotor, TrcMotorController rightRearMotor)
    {
        this(leftFrontMotor, leftMidMotor, leftRearMotor, rightFrontMotor, rightMidMotor, rightRearMotor, null);
    }   //TrcSimpleDriveBase

    /**
     * Constructor: Create an instance of a 4-wheel drive base.
     *
     * @param leftFrontMotor specifies the left front motor of the drive base.
     * @param leftRearMotor specifies the left rear motor of the drive base.
     * @param rightFrontMotor specifies the right front motor of the drive base.
     * @param rightRearMotor specifies the right rear motor of the drive base.
     * @param gyro specifies the gyro. If none, it can be set to null.
     */
    public TrcSimpleDriveBase(
        TrcMotorController leftFrontMotor, TrcMotorController leftRearMotor,
        TrcMotorController rightFrontMotor, TrcMotorController rightRearMotor,
        TrcGyro gyro)
    {
        super(new TrcMotorController[] {leftFrontMotor, rightFrontMotor, leftRearMotor, rightRearMotor}, gyro);

        if (leftFrontMotor == null || rightFrontMotor == null || leftRearMotor == null || rightRearMotor == null)
        {
            throw new IllegalArgumentException("All 4 motors must not be null.");
        }

        this.leftFrontMotor = leftFrontMotor;
        this.rightFrontMotor = rightFrontMotor;
        this.leftRearMotor = leftRearMotor;
        this.rightRearMotor = rightRearMotor;
        this.leftMidMotor = null;
        this.rightMidMotor = null;
    }   //TrcSimpleDriveBase

    /**
     * Constructor: Create an instance of a 4-wheel drive base.
     *
     * @param leftFrontMotor specifies the left front motor of the drive base.
     * @param leftRearMotor specifies the left rear motor of the drive base.
     * @param rightFrontMotor specifies the right front motor of the drive base.
     * @param rightRearMotor specifies the right rear motor of the drive base.
     */
    public TrcSimpleDriveBase(
        TrcMotorController leftFrontMotor, TrcMotorController leftRearMotor,
        TrcMotorController rightFrontMotor, TrcMotorController rightRearMotor)
    {
        this(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor, null);
    }   //TrcSimpleDriveBase

    /**
     * Constructor: Create an instance of a 2-wheel drive base.
     *
     * @param leftMotor specifies the left rear motor of the drive base.
     * @param rightMotor specifies the right rear motor of the drive base.
     * @param gyro specifies the gyro. If none, it can be set to null.
     */
    public TrcSimpleDriveBase(
        TrcMotorController leftMotor, TrcMotorController rightMotor, TrcGyro gyro)
    {
        super(new TrcMotorController[] {leftMotor, rightMotor}, gyro);

        if (leftMotor == null || rightMotor == null)
        {
            throw new IllegalArgumentException("All 2 motors must not be null.");
        }

        this.leftFrontMotor = leftMotor;
        this.rightFrontMotor = rightMotor;
        this.leftRearMotor = null;
        this.rightRearMotor = null;
        this.leftMidMotor = null;
        this.rightMidMotor = null;
    }   //TrcSimpleDriveBase

    /**
     * Constructor: Create an instance of a 2-wheel drive base.
     *
     * @param leftMotor specifies the left rear motor of the drive base.
     * @param rightMotor specifies the right rear motor of the drive base.
     */
    public TrcSimpleDriveBase(TrcMotorController leftMotor, TrcMotorController rightMotor)
    {
        this(leftMotor, rightMotor, null);
    }   //TrcSimpleDriveBase

    // CodeReview: Please explain what is this method for? Nobody is calling it. Why divide yScale by wheel base width?
    /**
     * This method sets the wheel base width of the robot drive base.
     *
     * @param width specifies the wheel base width.
     */
    public void setWheelBaseWidth(double width)
    {
        setPositionScales(xScale, yScale, yScale / width);
    }   //setWheelBaseWidth

    /**
     * This method inverts direction of a given motor in the drive train.
     *
     * @param motorType specifies the motor in the drive train.
     * @param inverted specifies true if inverting motor direction.
     */
    public void setInvertedMotor(MotorType motorType, boolean inverted)
    {
        setInvertedMotor(motorType.value, inverted);
    }   //setInvertedMotor

    /**
     * This method checks if the specified motor has stalled.
     *
     * @param motorType specifies the motor in the drive train.
     * @param stallTime specifies the stall time in seconds to be considered stalled.
     * @return true if the motor is stalled, false otherwise.
     */
    public boolean isMotorStalled(MotorType motorType, double stallTime)
    {
        return isMotorStalled(motorType.value, stallTime);
    }   //isMotorStalled

    /**
     * This method implements tank drive where leftPower controls the left motors and right power controls the right
     * motors.
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
        final String funcName = "tankDrive";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                "owner=%s,leftPower=%f,rightPower=%f,inverted=%s", owner, leftPower, rightPower, inverted);
        }

        if (validateOwnership(owner))
        {
            leftPower = TrcUtil.clipRange(leftPower);
            rightPower = TrcUtil.clipRange(rightPower);

            if (inverted)
            {
                double swap = leftPower;
                leftPower = -rightPower;
                rightPower = -swap;
            }

            if (isGyroAssistEnabled())
            {
                double assistPower = getGyroAssistPower((leftPower - rightPower)/2.0);
                leftPower += assistPower;
                rightPower -= assistPower;
                double maxMag = Math.max(Math.abs(leftPower), Math.abs(rightPower));
                if (maxMag > 1.0)
                {
                    leftPower /= maxMag;
                    rightPower /= maxMag;
                }
            }

            leftPower = clipMotorOutput(leftPower);
            rightPower = clipMotorOutput(rightPower);

            double wheelPower;

            if (leftFrontMotor != null)
            {
                wheelPower = leftPower;
                if (motorPowerMapper != null)
                {
                    wheelPower = motorPowerMapper.translateMotorPower(wheelPower, leftFrontMotor.getVelocity());
                }
                leftFrontMotor.set(wheelPower);
            }

            if (rightFrontMotor != null)
            {
                wheelPower = rightPower;
                if (motorPowerMapper != null)
                {
                    wheelPower = motorPowerMapper.translateMotorPower(wheelPower, rightFrontMotor.getVelocity());
                }
                rightFrontMotor.set(wheelPower);
            }

            if (leftRearMotor != null)
            {
                wheelPower = leftPower;
                if (motorPowerMapper != null)
                {
                    wheelPower = motorPowerMapper.translateMotorPower(wheelPower, leftRearMotor.getVelocity());
                }
                leftRearMotor.set(wheelPower);
            }

            if (rightRearMotor != null)
            {
                wheelPower = rightPower;
                if (motorPowerMapper != null)
                {
                    wheelPower = motorPowerMapper.translateMotorPower(wheelPower, rightRearMotor.getVelocity());
                }
                rightRearMotor.set(wheelPower);
            }

            if (leftMidMotor != null)
            {
                wheelPower = leftPower;
                if (motorPowerMapper != null)
                {
                    wheelPower = motorPowerMapper.translateMotorPower(wheelPower, leftMidMotor.getVelocity());
                }
                leftMidMotor.set(wheelPower);
            }

            if (rightMidMotor != null)
            {
                wheelPower = rightPower;
                if (motorPowerMapper != null)
                {
                    wheelPower = motorPowerMapper.translateMotorPower(wheelPower, rightMidMotor.getVelocity());
                }
                rightMidMotor.set(wheelPower);
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //tankDrive

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

        TrcPose2D poseDelta = new TrcPose2D();

        // Calculate heading and turn rate using positional info in case we don't have a gyro.
        // Get the average of all left and right motors separately, since this drivebase may have between 2-6 motors
        double lPos = 0.0, rPos = 0.0;
        double lVel = 0.0, rVel = 0.0;

        for (int i = 0; i < motorsState.motorPosDiffs.length; i++)
        {
            double posDiff = motorsState.motorPosDiffs[i];
            double vel = motorsState.currVelocities[i];

            if (i % 2 == 0)
            {
                lPos += posDiff;
                lVel += vel;
            }
            else
            {
                rPos += posDiff;
                rVel += vel;
            }
        }

        double motorsPerSide = getNumMotors() / 2.0;
        lPos /= motorsPerSide;
        rPos /= motorsPerSide;
        lVel /= motorsPerSide;
        rVel /= motorsPerSide;

        poseDelta.x = 0;
        poseDelta.y = (lPos + rPos)/2 * yScale;

        poseDelta.xVel = 0;
        poseDelta.yVel = (lVel + rVel)/2 * yScale;

        poseDelta.heading = Math.toDegrees((lPos - rPos) * rotScale);
        poseDelta.turnRate = Math.toDegrees((lVel - rVel) * rotScale);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }

        return poseDelta;
    }   //getPoseDelta

}   //class TrcSimpleDriveBase
