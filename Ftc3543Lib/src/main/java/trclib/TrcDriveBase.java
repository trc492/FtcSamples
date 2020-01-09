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

import java.util.Arrays;
import java.util.Locale;
import java.util.Stack;

/**
 * This class implements a platform independent drive base. It is intended to be extended by subclasses that
 * implements different drive base configurations (e.g. SimpleDriveBase, MecanumDriveBase and SwerveDriveBase).
 * The subclasses must provide the tankDrive and holonomicDrive methods. If the subclass cannot support a certain
 * driving strategy (e.g. holonomicDrive), it should throw an UnsupportedOperationException. They must also provide
 * the getOdometryDelta method where it will calculate the drive base position and velocity info according to sensors
 * such as encoders and gyro.
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
    protected TrcDbgTrace dbgTrace = null;
    //
    // If true, the change in pose is a twist, and is applied to the current pose using a non-zero curvature
    // (non-zero rotation velocity).
    // If false, use zero curvature (assume path is a bunch of straight lines). This is less accurate.
    //
    private static final boolean USE_CURVED_PATH = true;
    private static final boolean SYNC_GYRO_DATA = false;

    /**
     * This class implements the drive base odometry. It consists of the position as well as velocity info in all
     * three degrees of movement (x, y, angle).
     */
    public static class Odometry
    {
        TrcPose2D position;
        TrcPose2D velocity;

        /**
         * Constructor: Create an instance of the object.
         */
        Odometry()
        {
            position = new TrcPose2D();
            velocity = new TrcPose2D();
        }   //Odometry

        /**
         * Constructor: Create an instance of the object.
         *
         * @param position specifies the initial position.
         * @param velocity specifies the initial velocity.
         */
        Odometry(TrcPose2D position, TrcPose2D velocity)
        {
            this.position = position;
            this.velocity = velocity;
        }   //Odometry

        /**
         * This method returns the string representation of the object.
         *
         * @return string representation of the object.
         */
        @Override
        public String toString()
        {
            return "position=" + position.toString() + ", velocity=" + velocity.toString();
        }   //toString

        /**
         * This method creates and returns a copy of this odometry.
         *
         * @return a copy of this odometry.
         */
        public Odometry clone()
        {
            return new Odometry(position.clone(), velocity.clone());
        }   //clone

        /**
         * This method sets the position info of the odometry to the given pose.
         *
         * @param pose specifies the pose to set the position info to.
         */
        void setPositionAs(TrcPose2D pose)
        {
            this.position.setAs(pose);
        }   //setPositionAs

        /**
         * This method sets the velocity info of the odometry to the given pose.
         *
         * @param pose specifies the pose to set the velocity info to.
         */
        void setVelocityAs(TrcPose2D pose)
        {
            this.velocity.setAs(pose);
        }   //setVelocityAs

    }   //class Odometry

    /**
     * This class stores the states of all motors of the drivebase. This is used for calculating the drive base
     * odometry.
     */
    protected class MotorsState
    {
        TrcOdometrySensor.Odometry[] prevMotorOdometries;
        TrcOdometrySensor.Odometry[] currMotorOdometries;
        double[] stallStartTimes;

        public String toString()
        {
            return String.format(Locale.US, "odometry=%s", Arrays.toString(currMotorOdometries));
        }   //toString

    }   //class MotorsState

    /**
     * This method is called periodically to calculate the delta between the previous and current motor odometries.
     *
     * @param prevOdometries specifies the previous motor odometries.
     * @param currOdometries specifies the current motor odometries.
     * @return an Odometry object describing the odometry changes since the last update.
     */
    protected abstract Odometry getOdometryDelta(
            TrcOdometrySensor.Odometry prevOdometries[], TrcOdometrySensor.Odometry currOdometries[]);

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
    protected final Odometry odometry;
    private final MotorsState motorsState;
    protected double xScale, yScale, angleScale;
    private TrcTaskMgr.TaskObject odometryTaskObj;
    private TrcTaskMgr.TaskObject stopTaskObj;
    private TrcDriveBaseOdometry driveBaseOdometry = null;
    protected MotorPowerMapper motorPowerMapper = null;
    private double sensitivity = DEF_SENSITIVITY;
    private double maxOutput = DEF_MAX_OUTPUT;
    private double gyroMaxRotationRate = 0.0;
    private double gyroAssistKp = 1.0;
    private boolean gyroAssistEnabled = false;
    private Stack<Odometry> referenceOdometryStack = new Stack<>();
    private Odometry referenceOdometry = null;
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

        odometry = new Odometry();
        motorsState = new MotorsState();
        motorsState.prevMotorOdometries = new TrcOdometrySensor.Odometry[motors.length];
        motorsState.currMotorOdometries = new TrcOdometrySensor.Odometry[motors.length];
        motorsState.stallStartTimes = new double[motors.length];
        for (int i = 0; i < motors.length; i++)
        {
            motorsState.prevMotorOdometries[i] = null;
            motorsState.currMotorOdometries[i] = new TrcOdometrySensor.Odometry(motors[i]);
        }
        resetOdometry(true, true);
        resetStallTimers();
        xScale = yScale = angleScale = 1.0;

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
            odometryTaskObj.registerTask(TrcTaskMgr.TaskType.STANDALONE_TASK, TrcTaskMgr.INPUT_THREAD_INTERVAL);
//            odometryTaskObj.registerTask(TrcTaskMgr.TaskType.INPUT_TASK);
        }
        else
        {
            odometryTaskObj.unregisterTask(TrcTaskMgr.TaskType.STANDALONE_TASK);
//            odometryTaskObj.unregisterTask(TrcTaskMgr.TaskType.INPUT_TASK);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setOdometryEnabled

    /**
     * This method returns the robot position in reference to the field origin. By default, the field origin is the
     * robot's starting position.
     *
     * @return a copy of the robot position relative to the field origin.
     */
    public TrcPose2D getFieldPosition()
    {
        synchronized (odometry)
        {
            return odometry.position.clone();
        }
    }   //getFieldPosition

    /**
     * This method returns the robot velocity in reference to the field origin. By default, the field origin is the
     * robot's starting position.
     *
     * @return a copy of the robot velocity relative to the field origin.
     */
    public TrcPose2D getFieldVelocity()
    {
        synchronized (odometry)
        {
            return odometry.velocity.clone();
        }
    }   //getFieldVelocity

    /**
     * This method sets the robot's absolute field position to the given pose. This can be used to set the robot's
     * starting position relative to the field origin.
     *
     * @param pose specifies the absolute position of the robot relative to the field origin.
     */
    public void setFieldPosition(TrcPose2D pose)
    {
        synchronized (odometry)
        {
            odometry.setPositionAs(pose);
        }
    }   //setFieldPosition

    /**
     * This method returns the robot position relative to <code>pose</code>.
     *
     * @param posPose specifies the position to be referenced to.
     * @param transformAngle specifies true to also transform angle, false to leave it alone.
     * @return position transformed into the new reference pose.
     */
    public TrcPose2D getPositionRelativeTo(TrcPose2D posPose, boolean transformAngle)
    {
        synchronized (odometry)
        {
            return odometry.position.relativeTo(posPose, transformAngle);
        }
    }   //getPositionRelativeTo

    /**
     * This method returns the robot position relative to <code>pose</code>.
     *
     * @param posPose specifies the position to be referenced to.
     * @return position transformed into the new reference pose.
     */
    public TrcPose2D getPositionRelativeTo(TrcPose2D posPose)
    {
        return getPositionRelativeTo(posPose, true);
    }   //getPositionRelativeTo

    /**
     * This method returns the robot velocity relative to <code>pose</code>.
     *
     * @param velPose specifies the velocity to be referenced to.
     * @param refAngle specifies the reference angle to be relative to.
     * @return velocity transformed into the new reference pose.
     */
    public TrcPose2D getVelocityRelativeTo(TrcPose2D velPose, double refAngle)
    {
        synchronized (odometry)
        {
            //
            // relativeTo will transform the odometry velocity vector to be relative to the angle of pose but the
            // angle of velPose is really the angular velocity not an angle, so we must duplicate velPose to a new
            // pose and change the angle member to be the refAngle and let the caller provide that angle.
            //
            TrcPose2D pose = velPose.clone();
            pose.angle = refAngle;
            return odometry.velocity.relativeTo(pose, false);
        }
    }   //getVelocityRelativeTo

    /**
     * This method returns the robot position relative to the reference position, or the robot field position if
     * there is no reference odometry set.
     *
     * @return robot position relative to the reference position, or robot field position if no reference odometry
     *         set.
     */
    public TrcPose2D getRelativePosition()
    {
        synchronized (odometry)
        {
            return referenceOdometry == null ?
                    getFieldPosition() : getPositionRelativeTo(referenceOdometry.position, true);
        }
    }   //getRelativePosition

    /**
     * This method returns the robot velocity relative to the reference velocity, or the robot field velocity if
     * there is no reference odometry set.
     *
     * @return robot velocity relative to the reference velocity, or robot field velocity if no reference odometry
     *         set.
     */
    public TrcPose2D getRelativeVelocity()
    {
        synchronized (odometry)
        {
            return referenceOdometry == null ?
                    getFieldVelocity() :
                    getVelocityRelativeTo(referenceOdometry.velocity, referenceOdometry.position.angle);
        }
    }   //getRelativeVelocity

    /**
     * This method returns the reference odometry if there is any.
     *
     * @return the reference odometry.
     */
    public Odometry getReferenceOdometry()
    {
        synchronized (odometry)
        {
            return referenceOdometry;
        }
    }   //getReferenceOdometry

    /**
     * This method sets the current robot position and velocity as the reference odometry. All relative positions
     * and velocities will be relative to this reference odometry.
     */
    public void setReferenceOdometry()
    {
        synchronized (odometry)
        {
            referenceOdometry = odometry.clone();
        }
    }   //setReferenceOdometry

    /**
     * This method clears the reference odometry. All relative positions and velocities will instead be absolute
     * positions and velocities.
     */
    public void clearReferenceOdometry()
    {
        synchronized (odometry)
        {
            referenceOdometry = null;
        }
    }   //clearReferenceOdometry

    /**
     * This method pushes the existing reference odometry onto the stack if there is one and set the given robot's
     * absolute odometry as the new reference odometry.
     */
    public void pushReferenceOdometry()
    {
        synchronized (odometry)
        {
            if (referenceOdometry != null)
            {
                referenceOdometryStack.push(referenceOdometry);
            }

            setReferenceOdometry();
        }
    }   //pushReferenceOdometry

    /**
     * This method returns the current reference odometry. If the reference stack is not empty, it pops an odometry
     * from the stack as the new reference odometry, otherwise the reference odometry is cleared.
     *
     * @return current reference odometry.
     */
    public Odometry popReferenceOdometry()
    {
        synchronized (odometry)
        {
            Odometry oldReferenceOdometry = referenceOdometry;

            if (!referenceOdometryStack.empty())
            {
                referenceOdometry = referenceOdometryStack.pop();
            }
            else
            {
                referenceOdometry = null;
            }

            return oldReferenceOdometry;
        }
    }   //popReferenceOdometry

    /**
     * This method sets the position scales. The raw position from the encoder is in encoder counts. By setting the
     * scale factor, one could make getPosition to return unit in inches, for example.
     *
     * @param xScale   specifies the X position scale.
     * @param yScale   specifies the Y position scale.
     * @param angleScale specifies the angle scale.
     */
    public void setOdometryScales(double xScale, double yScale, double angleScale)
    {
        final String funcName = "setOdometryScales";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "xScale=%f,yScale=%f,angleScale=%f",
                    xScale, yScale, angleScale);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        synchronized (odometry)
        {
            this.xScale = xScale;
            this.yScale = yScale;
            this.angleScale = angleScale;

            if (driveBaseOdometry != null)
            {
                driveBaseOdometry.setOdometryScales(xScale, yScale, angleScale);
            }
        }
    }   //setOdometryScales

    /**
     * This method sets the position scales. The raw position from the encoder is in encoder counts. By setting the
     * scale factor, one could make getPosition to return unit in inches, for example.
     *
     * @param xScale specifies the X position scale.
     * @param yScale specifies the Y position scale.
     */
    public void setOdometryScales(double xScale, double yScale)
    {
        setOdometryScales(xScale, yScale, 1.0);
    }   //setOdometryScales

    /**
     * This method sets the position scales. The raw position from the encoder is in encoder counts. By setting the
     * scale factor, one could make getPosition to return unit in inches, for example.
     *
     * @param yScale specifies the Y position scale.
     */
    public void setOdometryScales(double yScale)
    {
        setOdometryScales(1.0, yScale, 1.0);
    }   //setOdometryScales

    /**
     * This method returns the X position in scaled unit.
     *
     * @return X position.
     */
    public double getXPosition()
    {
        final String funcName = "getXPosition";
        final double pos;

        pos = getRelativePosition().x;

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

        pos = getRelativePosition().y;

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
     * @return drive base heading.
     */
    public double getHeading()
    {
        final String funcName = "getHeading";
        final double heading;

        synchronized (odometry)
        {
            heading = odometry.position.angle;
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

        vel = getRelativeVelocity().x;

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

        vel = getRelativeVelocity().y;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", vel);
        }

        return vel;
    }   //getYVelocity

    /**
     * This method returns the drive base turn rate.
     *
     * @return turn rate.
     */
    public double getTurnRate()
    {
        final String funcName = "getTurnRate";
        final double turnRate;

        synchronized (odometry)
        {
            turnRate = odometry.velocity.angle;
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
     * @param resetHardware  specifies true for resetting hardware position, false for resetting software position.
     * @param resetAngle specifies true to also reset the angle odometry, false otherwise.
     */
    public void resetOdometry(boolean resetHardware, boolean resetAngle)
    {
        final String funcName = "resetOdometry";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        synchronized (odometry)
        {
            clearReferenceOdometry();

            if (driveBaseOdometry != null)
            {
                driveBaseOdometry.resetOdometry(resetHardware, resetAngle);
            }
            else
            {
                for (int i = 0; i < motors.length; i++)
                {
                    motors[i].resetPosition(resetHardware);
                    motorsState.prevMotorOdometries[i] = null;
                    motorsState.currMotorOdometries[i].prevTimestamp
                            = motorsState.currMotorOdometries[i].currTimestamp
                            = motorsState.stallStartTimes[i]
                            = TrcUtil.getCurrentTime();
                    motorsState.currMotorOdometries[i].prevPos
                            = motorsState.currMotorOdometries[i].currPos
                            = motorsState.currMotorOdometries[i].velocity = 0.0;
                }

                if (resetAngle)
                {
                    if (gyro != null)
                    {
                        gyro.resetZIntegrator();
                    }
                    odometry.position.angle = odometry.velocity.angle = 0.0;
                }
            }

            odometry.position.x = odometry.position.y = 0.0;
            odometry.velocity.x = odometry.velocity.y = 0.0;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //resetOdometry

    /**
     * This method resets the drive base odometry. This includes the motor encoders, drive base position, velocity and
     * gyro heading.
     *
     * @param resetHardware specifies true for resetting hardware position, false for resetting software position.
     */
    public void resetOdometry(boolean resetHardware)
    {
        resetOdometry(resetHardware, true);
    }   //resetOdometry

    /**
     * This method resets the drive base odometry. This includes the motor encoders, drive base position, velocity and
     * gyro heading.
     */
    public void resetOdometry()
    {
        resetOdometry(false, true);
    }   //resetOdometry

    /**
     * This method sets the given odometry device as the drive base's odometry device overriding the built-in odometry.
     *
     * @param driveBaseOdometry specifies the drive base odometry device.
     */
    public void setDriveBaseOdometry(TrcDriveBaseOdometry driveBaseOdometry)
    {
        synchronized (odometry)
        {
            this.driveBaseOdometry = driveBaseOdometry;
            resetOdometry(false, true);
        }
    }   //setDriveBaseOdometry

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
    public void resetStallTimers()
    {
        final String funcName = "resetStallTimers";

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
    }   //resetStallTimers

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
        holonomicDrive(
                owner, magnitude * Math.sin(dirInRads), magnitude * Math.cos(dirInRads), rotation, inverted,
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
        holonomicDrive(
                owner, magnitude * Math.sin(dirInRads), magnitude * Math.cos(dirInRads), rotation, false,
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
        holonomicDrive(
                owner, magnitude * Math.sin(dirInRads), magnitude * Math.cos(dirInRads), rotation, false,
                0.0);
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
     * This method is called periodically to update the drive base odometry (position and velocity of both x, y and
     * angle).
     *
     * @param taskType specifies the type of task being run.
     * @param runMode  specifies the competition mode that is about to end (e.g. Autonomous, TeleOp, Test).
     */
    private void odometryTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = "odometryTask";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "taskType=%s,runMode=%s",
                    taskType, runMode);
        }

        synchronized (odometry)
        {
            Odometry odometryDelta;

            if (driveBaseOdometry != null)
            {
                odometryDelta = driveBaseOdometry.getOdometryDelta();
                updateOdometry(odometryDelta, odometry.position.angle);
            }
            else
            {
                //
                // Update all motor states.
                //
                for (int i = 0; i < motors.length; i++)
                {
                    motorsState.prevMotorOdometries[i] = motorsState.currMotorOdometries[i];
                    motorsState.currMotorOdometries[i] = motors[i].getOdometry();

                    if (motorsState.currMotorOdometries[i].currPos != motorsState.currMotorOdometries[i].prevPos ||
                        motors[i].getPower() == 0.0)
                    {
                        motorsState.stallStartTimes[i] = motorsState.currMotorOdometries[i].currTimestamp;
                    }
                }

                synchronizeOdometries(motorsState.currMotorOdometries);
                //
                // Calculate pose delta from last pose and update odometry accordingly.
                //
                odometryDelta = getOdometryDelta(motorsState.prevMotorOdometries, motorsState.currMotorOdometries);
                if (gyro != null)
                {
                    TrcOdometrySensor.Odometry gyroOdometry = gyro.getOdometry();

                    if (SYNC_GYRO_DATA)
                    {
                        if (debugEnabled)
                        {
                            dbgTrace.traceInfo(funcName, "Gyro Before: timestamp=%.3f, pos=%.1f, vel=%.1f",
                                    gyroOdometry.currTimestamp, gyroOdometry.currPos, gyroOdometry.velocity);
                        }

                        double refTimestamp = motorsState.currMotorOdometries[0].currTimestamp;
                        gyroOdometry.currPos -= gyroOdometry.velocity * (gyroOdometry.currTimestamp - refTimestamp);
                        gyroOdometry.currTimestamp = refTimestamp;
                        if (debugEnabled)
                        {
                            dbgTrace.traceInfo(funcName, "Gyro After: timestamp=%.3f, pos=%.1f, vel=%.1f",
                                    gyroOdometry.currTimestamp, gyroOdometry.currPos, gyroOdometry.velocity);
                        }
                    }
                    // Overwrite the angle/turnrate values if gyro present, since that's more accurate
                    odometryDelta.position.angle = gyroOdometry.currPos - odometry.position.angle;
                    odometryDelta.velocity.angle = gyroOdometry.velocity;
                }

                updateOdometry(odometryDelta, odometry.position.angle);

                if (debugEnabled)
                {
                    dbgTrace.traceInfo(funcName, "motorsState: %s", motorsState);
                    dbgTrace.traceInfo(funcName, "delta: %s", odometryDelta);
                    dbgTrace.traceInfo(funcName, "odometry: %s", odometry);
                }
            }
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
     * This method synchronizes the odometries of all the drive base motors. Since all motors are read at different
     * times, the delay may cause inconsistencies on the odometry data which will affect the accuracy of the drive
     * base odometry calculation. We will make all motors based off of the timestamp of the last motor and their
     * position data will be interpolated accordingly. In effect, we are fast forwarding the other motors to sync
     * with the latest timestamp and predicting their positions with their velocity info.
     *
     * @param odometries specifies the array of odometries of all drive base motors.
     */
    private void synchronizeOdometries(TrcOdometrySensor.Odometry[] odometries)
    {
        final String funcName = "synchronizeOdometries";
        int lastIndex = odometries.length - 1;
        double refTimestamp = odometries[lastIndex].currTimestamp;

        for (int i = 0; i < lastIndex; i++)
        {
            if (debugEnabled)
            {
                dbgTrace.traceInfo(funcName, "[%d] Before: name=%s, timestamp=%.3f, pos=%.1f, vel=%.1f",
                        i, odometries[i].sensor, odometries[i].currTimestamp, odometries[i].currPos,
                        odometries[i].velocity);
            }

            odometries[i].currPos -= odometries[i].velocity * (odometries[i].currTimestamp - refTimestamp);
            odometries[i].currTimestamp = refTimestamp;

            if (debugEnabled)
            {
                dbgTrace.traceInfo(funcName, "[%d] After: name=%s, timestamp=%.3f, pos=%.1f, vel=%.1f",
                        i, odometries[i].sensor, odometries[i].currTimestamp, odometries[i].currPos,
                        odometries[i].velocity);
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceInfo(funcName, "[%d] Reference: name=%s, timestamp=%.3f, pos=%.1f, vel=%.1f",
                    lastIndex, odometries[lastIndex].sensor, odometries[lastIndex].currTimestamp,
                    odometries[lastIndex].currPos, odometries[lastIndex].velocity);
        }
    }   //synchronizeOdometries

    /**
     * This method updates the current robot odometry with the delta either using 0 or 1st order dynamics depending on
     * the value of <code>USE_CURVED_PATH</code>. If true, use a curved path (with nonzero curvature) otherwise model
     * path as a bunch of straight lines. The curved path is more accurate.
     *
     * @param delta specifies the odometry delta since the last update.
     * @param angle specifies the robot angle in the last update.
     */
    private void updateOdometry(Odometry delta, double angle)
    {
        final String funcName = "updateOdometry";

        if (USE_CURVED_PATH)
        {
            // The math below uses a different coordinate system (NWU) so we have to convert
            double[] posArr = enuToNwuChangeOfBasis.operate(new double[] { delta.position.x, delta.position.y });
            double x = posArr[0];
            double y = posArr[1];
            // Convert clockwise degrees to counter-clockwise radians
            double theta = Math.toRadians(-delta.position.angle);
            double headingRad = Math.toRadians(-angle);

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
            // Convert back to clockwise degrees for angle
            theta = Math.toDegrees(-globalPose.getEntry(2));

            // Rotate the velocity vector into the global reference frame
            RealVector vel = MatrixUtils.createRealVector(new double[] { delta.velocity.x, delta.velocity.y });
            vel = TrcUtil.rotateCW(vel, angle);

            // Update the odometry values
            odometry.position.x += pos.getEntry(0);
            odometry.position.y += pos.getEntry(1);
            odometry.position.angle += theta;
            odometry.velocity.x = vel.getEntry(0);
            odometry.velocity.y = vel.getEntry(1);
            odometry.velocity.angle = delta.velocity.angle;
        }
        else
        {
            RealVector pos = MatrixUtils.createRealVector(new double[] { delta.position.x, delta.position.y });
            RealVector vel = MatrixUtils.createRealVector(new double[] { delta.velocity.x, delta.velocity.y });

            pos = TrcUtil.rotateCW(pos, odometry.position.angle);
            vel = TrcUtil.rotateCW(vel, odometry.position.angle);

            odometry.position.x += pos.getEntry(0);
            odometry.position.y += pos.getEntry(1);
            odometry.velocity.x = vel.getEntry(0);
            odometry.velocity.y = vel.getEntry(1);
            odometry.position.angle += delta.position.angle;
            odometry.velocity.angle = delta.velocity.angle;
        }
    }   //updateOdometry

}   //class TrcDriveBase
