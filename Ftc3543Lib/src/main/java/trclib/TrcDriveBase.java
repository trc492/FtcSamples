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

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

/**
 * This class implements a platform independent drive base. It is intended to be extended by subclasses that
 * implements different drive base configurations (e.g. SimpleDriveBase, MecanumDriveBase and SwerveDriveBase).
 * The subclasses must provide the tankDrive and holonomicDrive methods. If the subclass cannot support a certain
 * driving strategy (e.g. holonomicDrive), it should throw an UnsupportedOperationException. They must also provide
 * the getPoseDelta method where it will calculate the drivebase position info according to sensors such as encoders
 * and gyro.
 */
public abstract class TrcDriveBase implements TrcExclusiveSubsystem
{
    private static final String moduleName = "TrcDriveBase";
    protected static final TrcDbgTrace globalTracer = TrcDbgTrace.getGlobalTracer();
    protected static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    //
    // If true, the change in pose is a twist, and is applied to the current pose using a non-zero curvature
    // (non-zero rotation velocity).
    // If false, use zero curvature (assume path is a bunch of straight lines). This is less accurate.
    //
    private static final boolean USE_CURVED_PATH = true;
    protected TrcDbgTrace dbgTrace = null;

    protected class MotorsState
    {
        public double prevTimestamp;
        public double currTimestamp;
        public double[] currPositions;
        public double[] currVelocities;
        public double[] prevPositions;
        public double[] stallStartTimes;
        public double[] motorPosDiffs;
    }   //class MotorsState

    /**
     * This method is called periodically to calculate the position delta from the previouse pose.
     *
     * @param motorsState specifies the MotorsState object containing the relevant data to calculate pose.
     * @return a TrcPose2D object describing the change in position since the last update.
     */
    protected abstract TrcPose2D getPoseDelta(MotorsState motorsState);

    /**
     * This method implements tank drive where leftPower controls the left motors and right power controls the right
     * motors.
     *
     * @param owner      specifies the ID string of the caller for checking ownership, can be null if caller is not
     *                   ownership aware.
     * @param leftPower  specifies left power value.
     * @param rightPower specifies right power value.
     * @param inverted   specifies true to invert control (i.e. robot front becomes robot back).
     */
    public abstract void tankDrive(String owner, double leftPower, double rightPower, boolean inverted);

    /**
     * This interface is provided by the caller to translate the motor power to actual motor power according to
     * the motor curve. This is useful to linearize the motor performance. This is very useful for many reasons.
     * It could allow the drive base to drive straight by translating wheel power to actual torque. It could also
     * allow us to implement our own ramp rate to limit acceleration and deceleration.
     */
    public interface MotorPowerMapper
    {
        /**
         * This method is called to translate the desired motor power to the actual motor power taking into
         * consideration of the motor torque curve with the current motor velocity.
         *
         * @param power    specifies the desired motor power.
         * @param velocity specifies the current motor velocity in the unit of encoder counts per second.
         * @return resulting motor power.
         */
        double translateMotorPower(double power, double velocity);
    }   //interface MotorPowerMapper

    private static double DEF_SENSITIVITY = 0.5;
    private static double DEF_MAX_OUTPUT = 1.0;

    private final TrcMotorController[] motors;
    private final TrcGyro gyro;
    protected final TrcPose2D odometry;
    private final MotorsState motorsState;
    protected double xScale, yScale, rotScale;
    private TrcTaskMgr.TaskObject odometryTaskObj;
    private TrcTaskMgr.TaskObject stopTaskObj;
    protected MotorPowerMapper motorPowerMapper = null;
    private double sensitivity = DEF_SENSITIVITY;
    private double maxOutput = DEF_MAX_OUTPUT;
    private double gyroMaxRotationRate = 0.0;
    private double gyroAssistKp = 1.0;
    private boolean gyroAssistEnabled = false;
    private TrcPose2D referencePose = null;
    // Change of basis matrices to convert between coordinate systems
    private final RealMatrix enuToNwuChangeOfBasis = MatrixUtils
        .createRealMatrix(new double[][] { { 0, 1 }, { -1, 0 } });
    private final RealMatrix nwuToEnuChangeOfBasis = enuToNwuChangeOfBasis.transpose();

    /**
     * Constructor: Create an instance of the object.
     *
     * @param motors specifies the array of motors in the drive base.
     * @param gyro   specifies the gyro. If none, it can be set to null.
     */
    public TrcDriveBase(TrcMotorController[] motors, TrcGyro gyro)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer ?
                globalTracer :
                new TrcDbgTrace(moduleName, tracingEnabled, traceLevel, msgLevel);
        }

        this.motors = motors;
        this.gyro = gyro;

        odometry = new TrcPose2D();
        motorsState = new MotorsState();
        motorsState.motorPosDiffs = new double[motors.length];
        motorsState.currPositions = new double[motors.length];
        motorsState.currVelocities = new double[motors.length];
        motorsState.prevPositions = new double[motors.length];
        motorsState.stallStartTimes = new double[motors.length];
        resetOdometry(true, true);
        xScale = yScale = rotScale = 1.0;
        resetStallTimer();

        TrcTaskMgr taskMgr = TrcTaskMgr.getInstance();
        odometryTaskObj = taskMgr.createTask(moduleName + ".odometryTask", this::odometryTask);
        stopTaskObj = taskMgr.createTask(moduleName + ".stopTask", this::stopTask);
        stopTaskObj.registerTask(TrcTaskMgr.TaskType.STOP_TASK);
    }   //TrcDriveBase

    /**
     * Constructor: Create an instance of the object.
     *
     * @param motors specifies the array of motors in the drive base.
     */
    public TrcDriveBase(TrcMotorController... motors)
    {
        this(motors, null);
    }   //TrcDriveBase

    /**
     * This method is called to enable/disable the odometry task that keeps track of the robot position and orientation.
     *
     * @param enabled specifies true to enable, false to disable.
     */
    public void setOdometryEnabled(boolean enabled)
    {
        final String funcName = "setOdometryEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enabled=%s", enabled);
        }

        if (enabled)
        {
            resetOdometry(false, false);
            odometryTaskObj.registerTask(TrcTaskMgr.TaskType.STANDALONE_TASK, 20);
        }
        else
        {
            odometryTaskObj.unregisterTask(TrcTaskMgr.TaskType.STANDALONE_TASK);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setOdometryEnabled

    /**
     * This method returns the absolute robot pose. The absolute position is in reference to the robot starting
     * position.
     *
     * @return a copy of the pose object relative to the robot starting position.
     */
    public TrcPose2D getAbsolutePose()
    {
        synchronized (odometry)
        {
            return TrcPose2D.copyOf(odometry);
        }
    }   //getAbsolutePose

    /**
     * This method returns the robot pose relative to <code>pose</code>.
     *
     * @param pose The new reference pose to transform to.
     * @return The pose object transformed into the new reference pose.
     */
    public TrcPose2D getPoseRelativeTo(TrcPose2D pose)
    {
        synchronized (odometry)
        {
            return odometry.relativeTo(pose);
        }
    }   //getPoseRelativeTo

    /**
     * This method returns the robot pose relative to the reference pose, or the robot absolute pose if there is no
     * reference pose.
     *
     * @return robot pose relative to the reference pose, or robot absolute pose if no reference pose.
     */
    public TrcPose2D getRelativePose()
    {
        return referencePose == null ? getAbsolutePose() : getPoseRelativeTo(referencePose);
    }   //getRelativePose

    /**
     * This method returns the reference pose, or the global origin if none available.
     *
     * @return the reference pose, or if none have been set, return the global origin. (all zeros)
     */
    public TrcPose2D getReferencePose()
    {
        return referencePose == null ? new TrcPose2D() : TrcPose2D.copyOf(referencePose);
    }   //getReferencePose

    /**
     * This method sets the given pose as the reference pose. All relative poses will be relative to this reference
     * pose.
     *
     * @param referencePose The pose to set as the reference for all relative poses.
     */
    public void setReferencePose(TrcPose2D referencePose)
    {
        this.referencePose = referencePose;
    }   //setReferencePose

    /**
     * This method sets the current pose as the reference pose. All relative poses will be relative to this reference
     * pose.
     */
    public void setReferencePose()
    {
        setReferencePose(getAbsolutePose());
    }   //setReferencePose

    /**
     * This method clears the reference pose. All relative poses will instead be absolute poses.
     */
    public void clearReferencePose()
    {
        referencePose = null;
    }   //clearReferencePose

    /**
     * This method sets the position scales. The raw position from the encoder is in encoder counts. By setting the
     * scale factor, one could make getPosition to return unit in inches, for example.
     *
     * @param xScale   specifies the X position scale.
     * @param yScale   specifies the Y position scale.
     * @param rotScale specifies the rotation scale.
     */
    public void setPositionScales(double xScale, double yScale, double rotScale)
    {
        final String funcName = "setPositionScales";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "xScale=%f,yScale=%f,rotScale=%f", xScale, yScale,
                rotScale);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.xScale = xScale;
        this.yScale = yScale;
        this.rotScale = rotScale;
    }   //setPositionScales

    /**
     * This method sets the position scales. The raw position from the encoder is in encoder counts. By setting the
     * scale factor, one could make getPosition to return unit in inches, for example.
     *
     * @param xScale specifies the X position scale.
     * @param yScale specifies the Y position scale.
     */
    public void setPositionScales(double xScale, double yScale)
    {
        setPositionScales(xScale, yScale, 1.0);
    }   //setPositionScales

    /**
     * This method sets the position scales. The raw position from the encoder is in encoder counts. By setting the
     * scale factor, one could make getPosition to return unit in inches, for example.
     *
     * @param yScale specifies the Y position scale.
     */
    public void setPositionScales(double yScale)
    {
        setPositionScales(1.0, yScale, 1.0);
    }   //setPositionScales

    /**
     * This method returns the X position in scaled unit.
     *
     * @return X position.
     */
    public double getXPosition()
    {
        final String funcName = "getXPosition";
        final double pos;

        pos = getRelativePose().x;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", pos);
        }

        return pos;
    }   //getXPosition

    /**
     * This method returns the Y position in scaled unit.
     *
     * @return Y position.
     */
    public double getYPosition()
    {
        final String funcName = "getYPosition";
        double pos;

        pos = getRelativePose().y;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", pos);
        }

        return pos;
    }   //getYPosition

    /**
     * This method returns the heading of the drive base in degrees. If there is a gyro, the gyro heading is returned,
     * otherwise it returns the rotation position by using the encoders.
     *
     * @return drive base heading
     */
    public double getHeading()
    {
        final String funcName = "getHeading";
        final double heading;

        synchronized (odometry)
        {
            heading = odometry.heading;
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", heading);
        }

        return heading;
    }   //getHeading

    /**
     * This method returns the drive base velocity in the X direction.
     *
     * @return X velocity.
     */
    public double getXVelocity()
    {
        final String funcName = "getXVelocity";
        final double vel;

        vel = getRelativePose().xVel;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", vel);
        }

        return vel;
    }   //getXVelocity

    /**
     * This method returns the drive base velocity in the Y direction.
     *
     * @return Y velocity.
     */
    public double getYVelocity()
    {
        final String funcName = "getYVelocity";
        final double vel;

        vel = getRelativePose().yVel;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", vel);
        }

        return vel;
    }   //getYVelocity

    /**
     * This method returns the gyro turn rate.
     *
     * @return gyro turn rate.
     */
    public double getGyroTurnRate()
    {
        final String funcName = "getGyroTurnRate";
        final double turnRate;

        synchronized (odometry)
        {
            turnRate = odometry.turnRate;
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", turnRate);
        }

        return turnRate;
    }   //getGyroTurnRate

    /**
     * This method resets the drive base odometry. This includes the motor encoders, drive base position, velocity and
     * gyro heading.
     *
     * @param hardware  specifies true for resetting hardware position, false for resetting software position.
     * @param resetGyro specifies true to also reset the gyro heading, false otherwise.
     */
    public void resetOdometry(boolean hardware, boolean resetGyro)
    {
        final String funcName = "resetOdometry";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        clearReferencePose();

        synchronized (odometry)
        {
            motorsState.prevTimestamp = motorsState.currTimestamp = TrcUtil.getCurrentTime();

            for (int i = 0; i < motors.length; i++)
            {
                motors[i].resetPosition(hardware);
                motorsState.currPositions[i] = 0.0;
                motorsState.currVelocities[i] = 0.0;
                motorsState.prevPositions[i] = 0.0;
                motorsState.stallStartTimes[i] = motorsState.currTimestamp;
            }

            odometry.x = odometry.y = 0.0;
            odometry.xVel = odometry.yVel = 0.0;

            if (gyro != null && resetGyro)
            {
                gyro.resetZIntegrator();
                odometry.heading = odometry.turnRate = 0.0;
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //resetOdometry

    /**
     * This method resets the drive base position odometry. This includes the motor encoders, the gyro heading and
     * all the cached values.
     *
     * @param hardware specifies true for resetting hardware position, false for resetting software position.
     */
    public void resetOdometry(boolean hardware)
    {
        resetOdometry(hardware, true);
    }   //resetOdometry

    /**
     * This method resets the drive base position odometry. This includes the motor encoders, the gyro heading and
     * all the cached values.
     */
    public void resetOdometry()
    {
        resetOdometry(false, true);
    }   //resetOdometry

    /**
     * This method sets a motor power mapper. If null, it unsets the previously set mapper.
     *
     * @param motorPowerMapper specifies the motor power mapper. If null, clears the mapper.
     */
    public void setMotorPowerMapper(MotorPowerMapper motorPowerMapper)
    {
        this.motorPowerMapper = motorPowerMapper;
    }   //setMotorPowerMapper

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
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "sensitivity=%f", sensitivity);
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

        this.maxOutput = Math.abs(maxOutput);
    }   //setMaxOutput

    /**
     * This method clips the motor output to the range of -maxOutput to maxOutput.
     *
     * @param output specifies the motor output.
     * @return clipped motor output.
     */
    protected double clipMotorOutput(double output)
    {
        final String funcName = "clipMotorOutput";
        double motorOutput = TrcUtil.clipRange(output, -maxOutput, maxOutput);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "output=%f", output);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", motorOutput);
        }

        return motorOutput;
    }   //clipMotorOutput

    /**
     * This method enables gyro assist drive.
     *
     * @param gyroMaxRotationRate specifies the maximum rotation rate of the robot base reported by the gyro.
     * @param gyroAssistKp        specifies the gyro assist proportional constant.
     */
    public void enableGyroAssist(double gyroMaxRotationRate, double gyroAssistKp)
    {
        final String funcName = "enableGyroAssist";

        if (debugEnabled)
        {
            dbgTrace
                .traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "gyroMaxRate=%f,gyroAssistKp=%f", gyroMaxRotationRate,
                    gyroAssistKp);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.gyroMaxRotationRate = gyroMaxRotationRate;
        this.gyroAssistKp = gyroAssistKp;
        this.gyroAssistEnabled = true;
    }   //enableGyroAssist

    /**
     * This method enables/disables gyro assist drive.
     */
    public void disableGyroAssist()
    {
        final String funcName = "enableGyroAssist";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.gyroMaxRotationRate = 0.0;
        this.gyroAssistKp = 1.0;
        this.gyroAssistEnabled = false;
    }   //disableGyroAssist

    /**
     * This method checks if Gyro Assist is enabled.
     *
     * @return true if Gyro Assist is enabled, false otherwise.
     */
    public boolean isGyroAssistEnabled()
    {
        return gyroAssistEnabled;
    }   //isGyroAssistEnabled

    /**
     * This method calculates and returns the gyro assist power.
     *
     * @param rotation specifies the rotation power.
     * @return gyro assist power.
     */
    public double getGyroAssistPower(double rotation)
    {
        double error = rotation - gyro.getZRotationRate().value / gyroMaxRotationRate;
        return gyroAssistEnabled ? TrcUtil.clipRange(gyroAssistKp * error) : 0.0;
    }   //getGyroAssistPower

    /**
     * This method checks if it supports holonomic drive. Subclasses that support holonomic drive should override
     * this method.
     *
     * @return true if this drive base supports holonomic drive, false otherwise.
     */
    public boolean supportsHolonomicDrive()
    {
        return false;
    }   //supportsHolonomicDrive

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
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%d", motors.length);
        }

        return motors.length;
    }   //getNumMotors

    /**
     * This method inverts direction of a given motor in the drive train.
     *
     * @param index    specifies the index in the motors array.
     * @param inverted specifies true if inverting motor direction.
     */
    protected void setInvertedMotor(int index, boolean inverted)
    {
        final String funcName = "setInvertedMotor";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "index=%d,inverted=%s", index, inverted);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        motors[index].setInverted(inverted);
    }   //setInvertedMotor

    /**
     * This method checks if the specified motor has stalled.
     *
     * @param index     specifies the motor index.
     * @param stallTime specifies the stall time in seconds to be considered stalled.
     * @return true if the motor is stalled, false otherwise.
     */
    protected boolean isMotorStalled(int index, double stallTime)
    {
        final String funcName = "isMotorStalled";
        double currTime = TrcUtil.getCurrentTime();
        final boolean stalled;

        synchronized (odometry)
        {
            stalled = currTime - motorsState.stallStartTimes[index] > stallTime;
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "index=%d,stallTime=%.3f", index, stallTime);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", stalled);
        }

        return stalled;
    }   //isMotorStalled

    /**
     * This method resets the all stall timers.
     */
    public void resetStallTimer()
    {
        final String funcName = "resetStallTimer";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        synchronized (odometry)
        {
            double currTime = TrcUtil.getCurrentTime();

            for (int i = 0; i < motorsState.stallStartTimes.length; i++)
            {
                motorsState.stallStartTimes[i] = currTime;
            }
        }
    }   //resetStallTimer

    /**
     * This method checks if all motors on the drive base have been stalled for at least the specified stallTime.
     *
     * @param stallTime specifies the stall time in seconds.
     * @return true if the drive base is stalled, false otherwise.
     */
    public boolean isStalled(double stallTime)
    {
        final String funcName = "isStalled";
        boolean stalled = true;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "stallTime=%.3f", stallTime);
        }

        synchronized (odometry)
        {
            for (int i = 0; i < motorsState.stallStartTimes.length; i++)
            {
                if (!isMotorStalled(i, stallTime))
                {
                    stalled = false;
                    break;
                }
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", stalled);
        }

        return stalled;
    }   //isStalled

    /**
     * This method enables/disables brake mode of the drive base.
     *
     * @param enabled specifies true to enable brake mode, false to disable it.
     */
    public void setBrakeMode(boolean enabled)
    {
        final String funcName = "setBrakeMode";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enabled=%s", Boolean.toString(enabled));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        for (TrcMotorController motor : motors)
        {
            motor.setBrakeModeEnabled(enabled);
        }
    }   //setBrakeMode

    /**
     * This methods stops the drive base.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *              ownership aware.
     */
    public void stop(String owner)
    {
        final String funcName = "stop";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "owner=%s", owner);
        }

        if (validateOwnership(owner))
        {
            for (TrcMotorController motor : motors)
            {
                motor.set(0.0);
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //stop

    /**
     * This methods stops the drive base.
     */
    public void stop()
    {
        stop(null);
    }   //stop

    /**
     * This method implements tank drive where leftPower controls the left motors and right power controls the right
     * motors.
     *
     * @param owner      specifies the ID string of the caller for checking ownership, can be null if caller is not
     *                   ownership aware.
     * @param leftPower  specifies left power value.
     * @param rightPower specifies right power value.
     */
    public void tankDrive(String owner, double leftPower, double rightPower)
    {
        tankDrive(owner, leftPower, rightPower, false);
    }   //tankDrive

    /**
     * This method implements tank drive where leftPower controls the left motors and right power controls the right
     * motors.
     *
     * @param leftPower  specifies left power value.
     * @param rightPower specifies right power value.
     */
    public void tankDrive(double leftPower, double rightPower)
    {
        tankDrive(null, leftPower, rightPower, false);
    }   //tankDrive

    /**
     * This method implements tank drive where leftPower controls the left motors and right power controls the right
     * motors.
     *
     * @param leftPower  specifies left power value.
     * @param rightPower specifies right power value.
     * @param inverted   specifies true to invert control (i.e. robot front becomes robot back).
     */
    public void tankDrive(double leftPower, double rightPower, boolean inverted)
    {
        tankDrive(null, leftPower, rightPower, inverted);
    }   //tankDrive

    /**
     * This method drives the motors at "magnitude" and "curve". Both magnitude and curve are -1.0 to +1.0 values,
     * where 0.0 represents stopped and not turning. curve less than 0 will turn left and curve greater than 0 will
     * turn right. The algorithm for steering provides a constant turn radius for any normal speed range, both
     * forward and backward. Increasing sensitivity causes sharper turns for fixed values of curve.
     *
     * @param owner     specifies the ID string of the caller for checking ownership, can be null if caller is not
     *                  ownership aware.
     * @param magnitude specifies the speed setting for the outside wheel in a turn, forward or backwards, +1 to -1.
     * @param curve     specifies the rate of turn, constant for different forward speeds. Set curve less than 0 for left
     *                  turn or curve greater than 0 for right turn. Set curve = e^(-r/w) to get a turn radius r for
     *                  wheel base w of your robot. Conversely, turn radius r = -ln(curve)*w for a given value of curve
     *                  and wheel base w.
     * @param inverted  specifies true to invert control (i.e. robot front becomes robot back).
     */
    public void curveDrive(String owner, double magnitude, double curve, boolean inverted)
    {
        final String funcName = "curveDrive";
        double leftOutput;
        double rightOutput;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "owner=%s,mag=%f,curve=%f,inverted=%s", owner,
                magnitude, curve, inverted);
        }

        if (validateOwnership(owner))
        {
            if (curve < 0.0)
            {
                double value = Math.log(-curve);
                double ratio = (value - sensitivity) / (value + sensitivity);
                if (ratio == 0.0)
                {
                    ratio = 0.0000000001;
                }
                leftOutput = magnitude / ratio;
                rightOutput = magnitude;
            }
            else if (curve > 0.0)
            {
                double value = Math.log(curve);
                double ratio = (value - sensitivity) / (value + sensitivity);
                if (ratio == 0.0)
                {
                    ratio = 0.0000000001;
                }
                leftOutput = magnitude;
                rightOutput = magnitude / ratio;
            }
            else
            {
                leftOutput = magnitude;
                rightOutput = magnitude;
            }

            tankDrive(owner, leftOutput, rightOutput, inverted);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //curveDrive

    /**
     * This method drives the motors at "magnitude" and "curve". Both magnitude and curve are -1.0 to +1.0 values,
     * where 0.0 represents stopped and not turning. curve less than 0 will turn left and curve greater than 0 will
     * turn right. The algorithm for steering provides a constant turn radius for any normal speed range, both
     * forward and backward. Increasing sensitivity causes sharper turns for fixed values of curve.
     *
     * @param magnitude specifies the speed setting for the outside wheel in a turn, forward or backwards, +1 to -1.
     * @param curve     specifies the rate of turn, constant for different forward speeds. Set curve less than 0 for left
     *                  turn or curve greater than 0 for right turn. Set curve = e^(-r/w) to get a turn radius r for
     *                  wheel base w of your robot. Conversely, turn radius r = -ln(curve)*w for a given value of curve
     *                  and wheel base w.
     * @param inverted  specifies true to invert control (i.e. robot front becomes robot back).
     */
    public void curveDrive(double magnitude, double curve, boolean inverted)
    {
        curveDrive(null, magnitude, curve, inverted);
    }   //curveDrive

    /**
     * This method drives the motors with the given magnitude and curve values.
     *
     * @param magnitude specifies the magnitude value.
     * @param curve     specifies the curve value.
     */
    public void curveDrive(double magnitude, double curve)
    {
        curveDrive(null, magnitude, curve, false);
    }   //curveDrive

    /**
     * This method implements arcade drive where drivePower controls how fast the robot goes in the y-axis and
     * turnPower controls how fast it will turn.
     *
     * @param owner      specifies the ID string of the caller for checking ownership, can be null if caller is not
     *                   ownership aware.
     * @param drivePower specifies the drive power value.
     * @param turnPower  specifies the turn power value.
     * @param inverted   specifies true to invert control (i.e. robot front becomes robot back).
     */
    public void arcadeDrive(String owner, double drivePower, double turnPower, boolean inverted)
    {
        final String funcName = "arcadeDrive";
        double leftPower;
        double rightPower;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "owner=%s,drivePower=%f,turnPower=%f,inverted=%s",
                owner, drivePower, turnPower, inverted);
        }

        if (validateOwnership(owner))
        {
            drivePower = TrcUtil.clipRange(drivePower);
            turnPower = TrcUtil.clipRange(turnPower);

            leftPower = drivePower + turnPower;
            rightPower = drivePower - turnPower;
            double maxMag = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (maxMag > 1.0)
            {
                leftPower /= maxMag;
                rightPower /= maxMag;
            }

            tankDrive(owner, leftPower, rightPower, inverted);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //arcadeDrive

    /**
     * This method implements arcade drive where drivePower controls how fast the robot goes in the y-axis and
     * turnPower controls how fast it will turn.
     *
     * @param drivePower specifies the drive power value.
     * @param turnPower  specifies the turn power value.
     * @param inverted   specifies true to invert control (i.e. robot front becomes robot back).
     */
    public void arcadeDrive(double drivePower, double turnPower, boolean inverted)
    {
        arcadeDrive(null, drivePower, turnPower, inverted);
    }   //arcadeDrive

    /**
     * This method implements arcade drive where drivePower controls how fast the robot goes in the y-axis and
     * turnPower controls how fast it will turn.
     *
     * @param drivePower specifies the drive power value.
     * @param turnPower  specifies the turn power value.
     */
    public void arcadeDrive(double drivePower, double turnPower)
    {
        arcadeDrive(null, drivePower, turnPower, false);
    }   //arcadeDrive

    /**
     * This method implements holonomic drive where x controls how fast the robot will go in the x direction, and y
     * controls how fast the robot will go in the y direction. Rotation controls how fast the robot rotates and
     * gyroAngle specifies the heading the robot should maintain. Subclasses that supports holonomic drive should
     * override this method.
     *
     * @param owner     specifies the ID string of the caller for checking ownership, can be null if caller is not
     *                  ownership aware.
     * @param x         specifies the x power.
     * @param y         specifies the y power.
     * @param rotation  specifies the rotating power.
     * @param inverted  specifies true to invert control (i.e. robot front becomes robot back).
     * @param gyroAngle specifies the current gyro heading. Use this to drive by the field reference frame.
     */
    protected void holonomicDrive(String owner, double x, double y, double rotation, boolean inverted, double gyroAngle)
    {
        throw new UnsupportedOperationException("Holonomic drive is not supported by this drive base!");
    }   //holonomicDrive

    /**
     * This method implements holonomic drive where x controls how fast the robot will go in the x direction, and y
     * controls how fast the robot will go in the y direction. Rotation controls how fast the robot rotates and
     * gyroAngle specifies the heading the robot should maintain.
     *
     * @param owner    specifies the ID string of the caller for checking ownership, can be null if caller is not ownership aware.
     * @param x        specifies the x power.
     * @param y        specifies the y power.
     * @param rotation specifies the rotating power.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     */
    public void holonomicDrive(String owner, double x, double y, double rotation, boolean inverted)
    {
        holonomicDrive(owner, x, y, rotation, inverted, 0.0);
    }   //holonomicDrive

    /**
     * This method implements holonomic drive where x controls how fast the robot will go in the x direction, and y
     * controls how fast the robot will go in the y direction. Rotation controls how fast the robot rotates and
     * gyroAngle specifies the heading the robot should maintain.
     *
     * @param x        specifies the x power.
     * @param y        specifies the y power.
     * @param rotation specifies the rotating power.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     */
    public void holonomicDrive(double x, double y, double rotation, boolean inverted)
    {
        holonomicDrive(null, x, y, rotation, inverted, 0.0);
    }   //holonomicDrive

    /**
     * This method implements holonomic drive where x controls how fast the robot will go in the x direction, and y
     * controls how fast the robot will go in the y direction. Rotation controls how fast the robot rotates and
     * gyroAngle specifies the heading the robot should maintain.
     *
     * @param owner     specifies the ID string of the caller for checking ownership, can be null if caller is not ownership aware.
     * @param x         specifies the x power.
     * @param y         specifies the y power.
     * @param rotation  specifies the rotating power.
     * @param gyroAngle specifies the current gyro heading. Use this to drive by the field reference frame.
     */
    public void holonomicDrive(String owner, double x, double y, double rotation, double gyroAngle)
    {
        holonomicDrive(owner, x, y, rotation, false, gyroAngle);
    }   //holonomicDrive

    /**
     * This method implements holonomic drive where x controls how fast the robot will go in the x direction, and y
     * controls how fast the robot will go in the y direction. Rotation controls how fast the robot rotates and
     * gyroAngle specifies the heading the robot should maintain.
     *
     * @param x         specifies the x power.
     * @param y         specifies the y power.
     * @param rotation  specifies the rotating power.
     * @param gyroAngle specifies the current gyro heading. Use this to drive by the field reference frame.
     */
    public void holonomicDrive(double x, double y, double rotation, double gyroAngle)
    {
        holonomicDrive(null, x, y, rotation, false, gyroAngle);
    }   //holonomicDrive

    /**
     * This method implements holonomic drive where x controls how fast the robot will go in the x direction, and y
     * controls how fast the robot will go in the y direction. Rotation controls how fast the robot rotates and
     * gyroAngle specifies the heading the robot should maintain.
     *
     * @param owner    specifies the ID string of the caller for checking ownership, can be null if caller is not ownership aware.
     * @param x        specifies the x power.
     * @param y        specifies the y power.
     * @param rotation specifies the rotating power.
     */
    public void holonomicDrive(String owner, double x, double y, double rotation)
    {
        holonomicDrive(owner, x, y, rotation, false, 0.0);
    }   //holonomicDrive

    /**
     * This method implements holonomic drive where x controls how fast the robot will go in the x direction, and y
     * controls how fast the robot will go in the y direction. Rotation controls how fast the robot rotates and
     * gyroAngle specifies the heading the robot should maintain.
     *
     * @param x        specifies the x power.
     * @param y        specifies the y power.
     * @param rotation specifies the rotating power.
     */
    public void holonomicDrive(double x, double y, double rotation)
    {
        holonomicDrive(null, x, y, rotation, false, 0.0);
    }   //holonomicDrive

    /**
     * This method implements holonomic drive where magnitude controls how fast the robot will go in the given
     * direction and how fast it will rotate.
     *
     * @param owner     specifies the ID string of the caller for checking ownership, can be null if caller is not
     *                  ownership aware.
     * @param magnitude specifies the magnitude combining x and y axes.
     * @param direction specifies the direction in degrees. 0 is forward. Positive is clockwise.
     * @param rotation  specifies the rotation power.
     * @param inverted  specifies true to invert control (i.e. robot front becomes robot back).
     */
    public void holonomicDrive_Polar(String owner, double magnitude, double direction, double rotation,
        boolean inverted)
    {
        double dirInRads = Math.toRadians(direction);
        holonomicDrive(owner, magnitude * Math.sin(dirInRads), magnitude * Math.cos(dirInRads), rotation, inverted,
            0.0);
    }   //holonomicDrive_Polar

    /**
     * This method implements holonomic drive where magnitude controls how fast the robot will go in the given
     * direction and how fast it will rotate.
     *
     * @param magnitude specifies the magnitude combining x and y axes.
     * @param direction specifies the direction in degrees.
     * @param rotation  specifies the rotation power.
     * @param inverted  specifies true to invert control (i.e. robot front becomes robot back).
     */
    public void holonomicDrive_Polar(double magnitude, double direction, double rotation, boolean inverted)
    {
        holonomicDrive_Polar(null, magnitude, direction, rotation, inverted);
    }   //holonomicDrive_Polar

    /**
     * This method implements holonomic drive where magnitude controls how fast the robot will go in the given
     * direction and how fast it will rotate.
     *
     * @param owner     specifies the ID string of the caller for checking ownership, can be null if caller is not
     *                  ownership aware.
     * @param magnitude specifies the magnitude combining x and y axes.
     * @param direction specifies the direction in degrees.
     * @param rotation  specifies the rotation power.
     * @param gyroAngle specifies the current gyro heading. Use this to drive by the field reference frame.
     */
    public void holonomicDrive_Polar(String owner, double magnitude, double direction, double rotation,
        double gyroAngle)
    {
        double dirInRads = Math.toRadians(direction);
        holonomicDrive(owner, magnitude * Math.sin(dirInRads), magnitude * Math.cos(dirInRads), rotation, false,
            gyroAngle);
    }   //holonomicDrive_Polar

    /**
     * This method implements holonomic drive where magnitude controls how fast the robot will go in the given
     * direction and how fast it will rotate.
     *
     * @param magnitude specifies the magnitude combining x and y axes.
     * @param direction specifies the direction in degrees. 0 is forward. Positive is clockwise.
     * @param rotation  specifies the rotation power.
     * @param gyroAngle specifies the current gyro heading. Use this to drive by the field reference frame.
     */
    public void holonomicDrive_Polar(double magnitude, double direction, double rotation, double gyroAngle)
    {
        holonomicDrive_Polar(null, magnitude, direction, rotation, gyroAngle);
    }   //holonomicDrive_Polar

    /**
     * This method implements holonomic drive where magnitude controls how fast the robot will go in the given
     * direction and how fast it will rotate.
     *
     * @param owner     specifies the ID string of the caller for checking ownership, can be null if caller is not
     *                  ownership aware.
     * @param magnitude specifies the magnitude combining x and y axes.
     * @param direction specifies the direction in degrees.
     * @param rotation  specifies the rotation power.
     */
    public void holonomicDrive_Polar(String owner, double magnitude, double direction, double rotation)
    {
        double dirInRads = Math.toRadians(direction);
        holonomicDrive(owner, magnitude * Math.sin(dirInRads), magnitude * Math.cos(dirInRads), rotation, false, 0.0);
    }   //holonomicDrive_Polar

    /**
     * This method implements holonomic drive where magnitude controls how fast the robot will go in the given
     * direction and how fast it will rotate.
     *
     * @param magnitude specifies the magnitude combining x and y axes.
     * @param direction specifies the direction in degrees. 0 is forward. Positive is clockwise.
     * @param rotation  specifies the rotation power.
     */
    public void holonomicDrive_Polar(double magnitude, double direction, double rotation)
    {
        holonomicDrive_Polar(null, magnitude, direction, rotation);
    }   //holonomicDrive_Polar

    /**
     * This method is called periodically to update the drive base odometry (xPos, yPos, rotPos, heading).
     *
     * @param taskType specifies the type of task being run.
     * @param runMode  specifies the competition mode that is about to end (e.g. Autonomous, TeleOp, Test).
     */
    private void odometryTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = "odometryTask";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "taskType=%s,runMode=%s", taskType, runMode);
        }

        synchronized (odometry)
        {
            //
            // Update all motor states.
            //
            motorsState.prevTimestamp = motorsState.currTimestamp;
            motorsState.currTimestamp = TrcUtil.getCurrentTime();

            for (int i = 0; i < motors.length; i++)
            {
                motorsState.prevPositions[i] = motorsState.currPositions[i];

                try
                {
                    motorsState.currPositions[i] = motors[i].getPosition();
                }
                catch (UnsupportedOperationException e)
                {
                    motorsState.currPositions[i] = 0;
                }

                motorsState.motorPosDiffs[i] = motorsState.currPositions[i] - motorsState.prevPositions[i];

                try
                {
                    motorsState.currVelocities[i] = motors[i].getVelocity();
                }
                catch (UnsupportedOperationException e)
                {
                    motorsState.currVelocities[i] = 0;
                }

                if (motorsState.currPositions[i] != motorsState.prevPositions[i] || motors[i].getPower() == 0.0)
                {
                    motorsState.stallStartTimes[i] = motorsState.currTimestamp;
                }
            }
            //
            // Calculate pose delta from last pose and update odometry accordingly.
            //
            TrcPose2D poseDelta = getPoseDelta(motorsState);
            if (gyro != null)
            {
                // Overwrite the heading/turnrate values if gyro present, since that's more accurate
                poseDelta.heading = gyro.getZHeading().value - odometry.heading;
                poseDelta.turnRate = gyro.getZRotationRate().value;
            }

            updateOdometry(poseDelta, odometry.heading);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //odometryTask

    /**
     * This method is called when the competition mode is about to end to stop the drive base.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode  specifies the competition mode that is about to end (e.g. Autonomous, TeleOp, Test).
     */
    private void stopTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = "stopTask";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "taskType=%s,runMode=%s", taskType, runMode);
        }

        stop();

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //stopTask

    /**
     * This method updates the current robot pose with a poseDelta either using 0 or 1st order dynamics, depending on
     * the value of <code>USE_CURVED_PATH</code>. If true, use a curved path (with nonzero curvature) otherwise model
     * path as a bunch of straight lines. The curved path is more accurate.
     *
     * @param poseDelta The pose delta since the last update.
     * @param heading   The robot heading in the last update.
     */
    private void updateOdometry(TrcPose2D poseDelta, double heading)
    {
        if (USE_CURVED_PATH)
        {
            // The math below uses a different coordinate system (NWU) so we have to convert
            double[] posArr = enuToNwuChangeOfBasis.operate(new double[] { poseDelta.x, poseDelta.y });
            double x = posArr[0];
            double y = posArr[1];
            // Convert clockwise degrees to counter-clockwise radians
            double theta = Math.toRadians(-poseDelta.heading);
            double headingRad = Math.toRadians(-heading);

            // The derivation of the following math is here in section 11.1
            // (https://file.tavsys.net/control/state-space-guide.pdf)
            // A is a transformation matrix representing a CCW rotation by headingRad radians
            // This is used to bring the change in pose into the global reference frame
            RealMatrix A = MatrixUtils.createRealMatrix(
                new double[][] { { Math.cos(headingRad), -Math.sin(headingRad), 0 },
                                 { Math.sin(headingRad), Math.cos(headingRad), 0 },
                                 { 0, 0, 1 } });
            // B is used to apply a nonzero curvature to the path. When the curvature is zero, B resolves to the
            // identity matrix.
            // The math involved isn't immediately intuitive, but it's basically the integration of the forward odometry
            // matrix equation.
            RealMatrix B;
            if (Math.abs(theta) <= 1E-9)
            {
                // Use the taylor series approximations, since some values are indeterminate
                B = MatrixUtils.createRealMatrix(new double[][] { { 1 - theta * theta / 6.0, -theta / 2.0, 0 },
                    { theta / 2.0, 1 - theta * theta / 6.0, 0 }, { 0, 0, 1 } });
            }
            else
            {
                B = MatrixUtils.createRealMatrix(new double[][] { { Math.sin(theta), Math.cos(theta) - 1, 0 },
                    { 1 - Math.cos(theta), Math.sin(theta), 0 }, { 0, 0, theta } });
                B = B.scalarMultiply(1.0 / theta);
            }
            // C is the column vector containing the "raw" change in pose. This is the immediate output of the forward
            // odometry multiplied by timestep
            RealVector C = MatrixUtils.createRealVector(new double[] { x, y, theta });
            // Get the change in global pose
            RealVector globalPose = A.multiply(B).operate(C);
            // Convert back to our (ENU) reference frame
            RealVector pos = nwuToEnuChangeOfBasis.operate(globalPose.getSubVector(0, 2));
            // Convert back to clockwise degrees for heading
            theta = Math.toDegrees(-globalPose.getEntry(2));

            // Rotate the velocity vector into the global reference frame
            RealVector vel = MatrixUtils.createRealVector(new double[] { poseDelta.xVel, poseDelta.yVel });
            vel = TrcUtil.rotateCW(vel, heading);

            // Update the odometry values
            odometry.x += pos.getEntry(0);
            odometry.y += pos.getEntry(1);
            odometry.xVel = vel.getEntry(0);
            odometry.yVel = vel.getEntry(1);
            odometry.heading += theta;
            odometry.turnRate = poseDelta.turnRate;
        }
        else
        {
            RealVector pos = MatrixUtils.createRealVector(new double[] { poseDelta.x, poseDelta.y });
            RealVector vel = MatrixUtils.createRealVector(new double[] { poseDelta.xVel, poseDelta.yVel });

            pos = TrcUtil.rotateCW(pos, odometry.heading);
            vel = TrcUtil.rotateCW(vel, odometry.heading);

            odometry.x += pos.getEntry(0);
            odometry.y += pos.getEntry(1);
            odometry.xVel = vel.getEntry(0);
            odometry.yVel = vel.getEntry(1);
            odometry.heading += poseDelta.heading;
            odometry.turnRate = poseDelta.turnRate;
        }
    }   //updateOdometry

}   //class TrcDriveBase
