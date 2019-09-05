/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
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

import java.util.ArrayList;

import trclib.TrcTaskMgr.TaskType;

/**
 * This class implements a platform independent motor controller. Typically, this class is extended by a platform
 * dependent motor controller class. Not all motor controllers are created equal. Some have more features than the
 * others. This class attempts to emulate some of the features in software. If the platform dependent motor controller
 * supports some features in hardware it should override the corresponding methods and call the hardware directly.
 * For some features that there is no software emulation, this class will throw an UnsupportedOperationException.
 * If the motor controller hardware support these features, the platform dependent class should override these methods
 * to provide the support in hardware.
 */
public abstract class TrcMotor implements TrcMotorController
{
    protected static final String moduleName = "TrcMotor";
    protected static final TrcDbgTrace globalTracer = TrcDbgTrace.getGlobalTracer();
    protected static final boolean debugEnabled = false;
    protected static final boolean tracingEnabled = false;
    protected static final boolean useGlobalTracer = false;
    protected static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    protected static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    protected TrcDbgTrace dbgTrace = null;

    /**
     * This interface, if provided, is called if a digital input device is registered to reset the motor position
     * on trigger and a trigger event occurred.
     */
    public interface DigitalTriggerHandler
    {
        /**
         * This method is called when a digital trigger event occurred.
         *
         * @param active specifies true if the digital input is active, false if inactive.
         */
        void digitalTriggerEvent(boolean active);

    }   //interface DigitalTriggerHandler

    /**
     * This method returns the motor position by reading the position sensor. The position sensor can be an encoder
     * or a potentiometer.
     *
     * @return current motor position.
     */
    public abstract double getMotorPosition();

    /**
     * This method sets the raw motor power. It is called by the Velocity Control task. If the subclass is
     * implementing its own native velocity control, it does not really need to do anything for this method.
     * But for completeness, it can just set the raw motor power in the motor controller.
     *
     * @param power specifies the percentage power (range -1.0 to 1.0) to be set.
     */
    public abstract void setMotorPower(double power);

    private class Odometry
    {
        double prevTimestamp;
        double currTimestamp;
        double prevPos;
        double currPos;
        double velocity;
    }   //class Odometry

    private static final ArrayList<TrcMotor> odometryMotors = new ArrayList<>();
    private static TrcTaskMgr.TaskObject odometryTaskObj = null;
    private final Odometry odometry = new Odometry();

    private final String instanceName;
    private final TrcTaskMgr.TaskObject velocityCtrlTaskObj;
    private TrcDigitalTrigger digitalTrigger = null;
    private boolean odometryEnabled = false;
    private double maxMotorVelocity = 0.0;
    private TrcPidController velocityPidCtrl = null;
    private DigitalTriggerHandler digitalTriggerHandler = null;
    private boolean calibrating = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public TrcMotor(final String instanceName)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer ?
                TrcDbgTrace.getGlobalTracer() :
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;

        TrcTaskMgr taskMgr = TrcTaskMgr.getInstance();
        if (odometryTaskObj == null)
        {
            //
            // Odometry task is global. There is only one instance that manages odometry of all motors.
            // This allows us to move this task to a STANDALONE_TASK if it turns out degrading the INPUT_TASK too
            // much. If we create individual task for each motor, moving them to STANDALONE_TASK will create too
            // many threads.
            //
            odometryTaskObj = taskMgr.createTask(moduleName + ".odometryTask", TrcMotor::odometryTask);
        }
        velocityCtrlTaskObj = taskMgr.createTask(instanceName + ".velCtrlTask", this::velocityCtrlTask);
    }   //TrcMotor

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @Override
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method returns the number of motors in the list registered for odometry monitoring.
     *
     * @return number of motors in the list.
     */
    public static int getNumOdometryMotors()
    {
        int numMotors;

        synchronized (odometryMotors)
        {
            numMotors = odometryMotors.size();
        }

        return numMotors;
    }   //getNumOdometryMotors

    /**
     * This method clears the list of motors that register for odometry monitoring. This method should only be called
     * by the task scheduler.
     */
    public static void clearOdometryMotorsList()
    {
        synchronized (odometryMotors)
        {
            if (odometryMotors.size() > 0)
            {
                odometryMotors.clear();
                odometryTaskObj.unregisterTask(TaskType.INPUT_TASK);
            }
        }
    }   //clearOdometryMotorsList

    /**
     * This method resets the odometry data for the motor.
     */
    private void resetOdometry()
    {
        synchronized (odometry)
        {
            odometry.prevTimestamp = odometry.currTimestamp = TrcUtil.getCurrentTime();
            odometry.prevPos = odometry.currPos = getMotorPosition();
            odometry.velocity = 0.0;
        }
    }   //resetOdometry

    /**
     * This method enables/disables the task that monitors the motor odometry. Since odometry task takes up CPU cycle,
     * it should not be enabled if the user doesn't need motor odometry info.
     *
     * @param enabled specifies true to enable odometry task, disable otherwise.
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
            resetOdometry();
            synchronized (odometryMotors)
            {
                //
                // Add only if this motor is not already on the list.
                //
                if (!odometryMotors.contains(this))
                {
                    odometryMotors.add(this);
                    if (odometryMotors.size() == 1)
                    {
                        //
                        // We are the first one on the list, start the task.
                        //
                        odometryTaskObj.registerTask(TaskType.INPUT_TASK);
                    }
                }
            }
        }
        else
        {
            synchronized (odometryMotors)
            {
                odometryMotors.remove(this);
                if (odometryMotors.isEmpty())
                {
                    //
                    // We were the only one on the list, stop the task.
                    //
                    odometryTaskObj.unregisterTask(TaskType.INPUT_TASK);
                }
            }
        }
        odometryEnabled = enabled;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setOdometryEnabled

    /**
     * This method checks if the odometry of this motor is enabled.
     *
     * @return true if odometry of this motor is enabled, false if disabled.
     */
    public boolean isOdometryEnabled()
    {
        return odometryEnabled;
    }   //isOdometryEnabled

    /**
     * This method is called periodically to update motor odometry data. Odometry data includes position and velocity
     * data. By using this task to update odometry at a periodic rate, it allows robot code to obtain odometry data
     * from the cached data maintained by this task instead of repeatedly reading it directly from the motor
     * controller which may impact performance because it may involve initiating USB/CAN/I2C bus cycles. So even
     * though some motor controller hardware may keep track of its own velocity, it may be beneficial to just let the
     * odometry task to calculate it.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode  specifies the competition mode that is running.
     */
    public static void odometryTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = "odometryTask";

        if (debugEnabled)
        {
            globalTracer.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "taskType=%s,runMode=%s", taskType, runMode);
        }

        synchronized (odometryMotors)
        {
            for (TrcMotor motor : odometryMotors)
            {
                synchronized (motor.odometry)
                {
                    motor.odometry.prevTimestamp = motor.odometry.currTimestamp;
                    motor.odometry.prevPos = motor.odometry.currPos;
                    motor.odometry.currTimestamp = TrcUtil.getCurrentTime();
                    motor.odometry.currPos = motor.getMotorPosition();

                    double deltaTime = motor.odometry.currTimestamp - motor.odometry.prevTimestamp;
                    if (deltaTime > 0.0)
                    {
                        motor.odometry.velocity = (motor.odometry.currPos - motor.odometry.prevPos) / deltaTime;
                    }

                    if (debugEnabled)
                    {
                        globalTracer.traceInfo(funcName, "[%.3f]: %s encPos=%.0f",
                                motor.odometry.currTimestamp, motor, motor.odometry.currPos);
                    }
                }
            }
        }

        if (debugEnabled)
        {
            globalTracer.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //odometryTask

    /**
     * This method sets the motor controller to velocity mode with the specified maximum velocity.
     *
     * @param maxVelocity     specifies the maximum velocity the motor can run, in sensor units per second.
     * @param pidCoefficients specifies the PID coefficients to use to compute a desired torque value for the motor.
     *                        E.g. these coefficients go from velocity error percent to desired stall torque percent.
     */
    public synchronized void enableVelocityMode(double maxVelocity, TrcPidController.PidCoefficients pidCoefficients)
    {
        final String funcName = "enableVelocityMode";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "maxVel=%f,pidCoef=%s", maxVelocity,
                pidCoefficients.toString());
        }

        if (pidCoefficients == null)
        {
            throw new IllegalArgumentException("PidCoefficient must not be null.");
        }

        if (!isOdometryEnabled())
        {
            throw new IllegalStateException("Odometry must be enabled to use velocity mode.");
        }

        this.maxMotorVelocity = maxVelocity;
        velocityPidCtrl = new TrcPidController(
                instanceName + ".velocityCtrl", pidCoefficients, 1.0, this::getNormalizedVelocity);
        velocityPidCtrl.setAbsoluteSetPoint(true);

        velocityCtrlTaskObj.registerTask(TaskType.OUTPUT_TASK);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //enableVelocityMode

    /**
     * This method disables velocity mode returning it to power mode.
     */
    public synchronized void disableVelocityMode()
    {
        final String funcName = "disableVelocityMode";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (velocityPidCtrl != null)
        {
            velocityCtrlTaskObj.unregisterTask(TaskType.OUTPUT_TASK);
            velocityPidCtrl = null;
        }
    }   //disableVelocityMode

    /**
     * This method returns the motor velocity normalized to the range of -1.0 to 1.0, essentially a percentage of the
     * maximum motor velocity.
     *
     * @return normalized motor velocity.
     */
    private double getNormalizedVelocity()
    {
        final String funcName = "getNormalizedVelocity";
        double normalizedVel = getVelocity() / maxMotorVelocity;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", normalizedVel);
        }

        return normalizedVel;
    }   //getNormalizedVelocity

    /**
     * This method overrides the motorSpeedTask in TrcMotor which is called periodically to calculate he speed of
     * the motor. In addition to calculate the motor speed, it also calculates and sets the motor power required
     * to maintain the set speed if speed control mode is enabled.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode  specifies the competition mode that is running.
     */
    public synchronized void velocityCtrlTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = "velocityCtrlTask";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "taskType=%s,runMode=%s", taskType, runMode);
        }

        if (velocityPidCtrl != null)
        {
            double desiredStallTorquePercentage = velocityPidCtrl.getOutput();
            double motorPower = transformTorqueToMotorPower(desiredStallTorquePercentage);

            setMotorPower(motorPower);
            if (debugEnabled)
            {
                dbgTrace.traceInfo(instanceName,
                    "targetSpeed=%.2f, currSpeed=%.2f, desiredStallTorque=%.2f, motorPower=%.2f",
                    velocityPidCtrl.getTarget(), getVelocity(), desiredStallTorquePercentage, motorPower);
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //velocityCtrlTask

    /**
     * Transforms the desired percentage of motor stall torque to the motor duty cycle (aka power)
     * that would give us that amount of torque at the current motor speed.
     *
     * @param desiredStallTorquePercentage specifies the desired percentage of motor torque to receive in percent of
     *                                     motor stall torque.
     * @return power percentage to apply to the motor to generate the desired torque (to the best ability of the motor).
     */
    private double transformTorqueToMotorPower(double desiredStallTorquePercentage)
    {
        final String funcName = "transformTorqueToMotorPower";
        double power;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC, "torque=%f", desiredStallTorquePercentage);
        }
        //
        // Leverage motor curve information to linearize torque output across varying RPM
        // as best we can. We know that max torque is available at 0 RPM and zero torque is
        // available at max RPM - use that relationship to proportionately boost voltage output
        // as motor speed increases.
        //
        final double currSpeedSensorUnitPerSec = Math.abs(getVelocity());
        final double currNormalizedSpeed = currSpeedSensorUnitPerSec / maxMotorVelocity;

        // Max torque percentage declines proportionally to motor speed.
        final double percentMaxTorqueAvailable = 1 - currNormalizedSpeed;

        if (percentMaxTorqueAvailable > 0)
        {
            power = desiredStallTorquePercentage / percentMaxTorqueAvailable;
        }
        else
        {
            // When we exceed max motor speed (and the correction factor is undefined), apply 100% voltage.
            power = Math.signum(desiredStallTorquePercentage);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC, "=%f", power);
        }

        return power;
    }   //transformTorqueToMotorPower

    /**
     * This method creates a digital trigger on the given digital input sensor. It resets the position sensor
     * reading when the digital input is triggered.
     *
     * @param digitalInput specifies the digital input sensor that will trigger a position reset.
     * @param triggerHandler specifies an event callback if the trigger occurred, null if none specified.
     */
    public void resetPositionOnDigitalInput(TrcDigitalInput digitalInput, DigitalTriggerHandler triggerHandler)
    {
        final String funcName = "resetPositionOnDigitalInput";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "digitalInput=%s", digitalInput);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        digitalTriggerHandler = triggerHandler;
        digitalTrigger = new TrcDigitalTrigger(instanceName, digitalInput, this::triggerEvent);
        digitalTrigger.setEnabled(true);
    }   //resetPositionOnDigitalInput

    /**
     * This method creates a digital trigger on the given digital input sensor. It resets the position sensor
     * reading when the digital input is triggered.
     *
     * @param digitalInput specifies the digital input sensor that will trigger a position reset.
     */
    public void resetPositionOnDigitalInput(TrcDigitalInput digitalInput)
    {
        resetPositionOnDigitalInput(digitalInput, null);
    }   //resetPositionOnDigitalInput

    /**
     * This method resets the motor position sensor, typically an encoder.
     */
    public void resetPosition()
    {
        resetPosition(false);
    }   //resetPosition

    /**
     * This method performs a zero calibration on the motor by slowly turning in reverse. When the lower limit switch
     * is triggered, it stops the motor and resets the motor position.
     *
     * @param calibratePower specifies the motor power to perform zero calibration (must be positive value).
     */
    public void zeroCalibrate(double calibratePower)
    {
        //
        // Only do this if there is a digital trigger.
        //
        if (digitalTrigger != null && digitalTrigger.isEnabled())
        {
            set(-Math.abs(calibratePower));
            calibrating = true;
        }
    }   //zeroCalibrate

    /**
     * This method sets this motor to follow another motor. This method should be overridden by the subclass. If the
     * subclass is not capable of following another motor, this method will be called instead and will throw an
     * UnsupportedOperation exception.
     */
    public void follow(TrcMotor motor)
    {
        throw new UnsupportedOperationException(
            String.format("This motor does not support following motors of type: %s!", motor.getClass().toString()));
    }   //follow

    //
    // Implements the TrcMotorController interface.
    //

    /**
     * This method returns the motor position by reading the position sensor. The position sensor can be an encoder
     * or a potentiometer.
     *
     * @return current motor position.
     */
    @Override
    public double getPosition()
    {
        final String funcName = "getPosition";
        final double currPos;

        synchronized (odometry)
        {
            currPos = odometry.currPos;
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%d", currPos);
        }

        return currPos;
    }   //getPosition

    /**
     * This method returns the velocity of the motor rotation. It keeps track of the rotation velocity by using a
     * periodic task to monitor the position sensor value. If the motor controller has hardware monitoring velocity,
     * it can override this method and access the hardware instead. However, accessing hardware may impact
     * performance because it may involve initiating USB/CAN/I2C bus cycles. Therefore, it may be beneficial to
     * just let the the periodic task calculate the velocity here.
     *
     * @return motor velocity in sensor units per second.
     */
    @Override
    public double getVelocity()
    {
        final String funcName = "getVelocity";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (!odometryEnabled)
        {
            throw new UnsupportedOperationException("Odometry is not enabled.");
        }

        final double velocity;
        synchronized (odometry)
        {
            velocity = odometry.velocity;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%.3f", velocity);
        }

        return velocity;
    }   //getVelocity

    /**
     * This method sets the motor output value. The value can be power or velocity percentage depending on whether
     * the motor controller is in power mode or velocity mode.
     *
     * @param value specifies the percentage power or velocity (range -1.0 to 1.0) to be set.
     */
    @Override
    public void set(double value)
    {
        final String funcName = "set";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "value=%f", value);
        }

        calibrating = false;
        if (velocityPidCtrl != null)
        {
            velocityPidCtrl.setTarget(value);
        }
        else
        {
            setMotorPower(value);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "! (value=%f)", value);
        }
    }   //set

    //
    // Implements TrcDigitalTrigger.TriggerHandler.
    //

    /**
     * This method is called when the digital input device has changed state.
     *
     * @param active specifies true if the digital device state is active, false otherwise.
     */
    private void triggerEvent(boolean active)
    {
        final String funcName = "triggerEvent";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.CALLBK, "trigger=%s,active=%s", digitalTrigger,
                Boolean.toString(active));
        }

        TrcDbgTrace.getGlobalTracer()
            .traceInfo("triggerEvent", "TrcMotor encoder reset! motor=%s,pos=%.2f", instanceName, getMotorPosition());

        if (calibrating)
        {
            //
            // set(0.0) will turn off calibration mode.
            //
            set(0.0);
        }

        resetPosition(false);

        if (digitalTriggerHandler != null)
        {
            digitalTriggerHandler.digitalTriggerEvent(active);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.CALLBK);
        }
    }   //triggerEvent

}   //class TrcMotor
