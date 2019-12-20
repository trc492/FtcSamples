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

/**
 * This class implements a platform independent PID controlled motor. A PID controlled motor may consist of one or
 * two physical motors, a position sensor, typically an encoder (or could be a potentiometer). Optionally, it supports
 * a lower limit switch or even an upper limit switch. In addition, it has stall protection support which will detect
 * motor stall condition and will cut power to the motor preventing it from burning out.
 */
public class TrcPidMotor
{
    protected static final String moduleName = "TrcPidMotor";
    protected static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private static final boolean verbosePidInfo = false;
    protected TrcDbgTrace dbgTrace = null;
    private TrcDbgTrace msgTracer = null;
    private TrcRobotBattery battery = null;
    private boolean tracePidInfo = false;

    /**
     * Some actuators are non-linear. The load may vary depending on the position. For example, raising an arm
     * against gravity will have the maximum load when the arm is horizontal and zero load when vertical. This
     * caused problem when applying PID control on this kind of actuator because PID controller is only good at
     * controlling linear actuators. To make PID controller works for non-linear actuators, we need to add power
     * compensation that counteracts the non-linear component of the load so that PID only deals with the resulting
     * linear load. However, a generic PID controller doesn't understand the actuator and has no way to come up
     * with the compensation. Therefore, it is up to the user of the TrcPIDMotor to provide this interface for
     * computing the output compensation.
     */
    public interface PowerCompensation
    {
        /**
         * This method is called to compute the power compensation to counteract the varying non-linear load.
         *
         * @return compensation value of the actuator.
         */
        double getCompensation();

    }   //interface PowerCompensation

    private static final double MIN_MOTOR_POWER = -1.0;
    private static final double MAX_MOTOR_POWER = 1.0;

    private static final double DEF_BEEP_LOW_FREQUENCY = 440.0;     //in Hz
    private static final double DEF_BEEP_HIGH_FREQUECY = 880.0;     //in Hz
    private static final double DEF_BEEP_DURATION = 0.2;            //in seconds

    private final String instanceName;
    private final TrcMotor motor1;
    private final TrcMotor motor2;
    private final TrcPidController pidCtrl;
    private final double defCalPower;
    private final PowerCompensation powerCompensation;
    private final TrcTaskMgr.TaskObject pidMotorTaskObj;
    private final TrcTaskMgr.TaskObject stopMotorTaskObj;
    private boolean active = false;
    private double syncGain;
    private double positionScale = 1.0;
    private double positionOffset = 0.0;
    private boolean holdTarget = false;
    private TrcEvent notifyEvent = null;
    private double expiredTime = 0.0;
    private boolean calibrating = false;
    private double calPower = 0.0;
    private double motorPower = 0.0;
    private double prevPos = 0.0;
    private double prevTime = 0.0;
    private double prevTarget = 0.0;
    private boolean motor1ZeroCalDone = false;
    private boolean motor2ZeroCalDone = false;
    //
    // Beep device.
    //
    private TrcTone beepDevice = null;
    private double beepLowFrequency = DEF_BEEP_LOW_FREQUENCY;
    private double beepHighFrequency = DEF_BEEP_HIGH_FREQUECY;
    private double beepDuration = DEF_BEEP_DURATION;
    //
    // Stall protection.
    //
    private boolean stalled = false;
    private double stallMinPower = 0.0;
    private double stallTimeout = 0.0;
    private double resetTimeout = 0.0;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param motor1 specifies motor1 object.
     * @param motor2 specifies motor2 object. If there is only one motor, this can be set to null.
     * @param syncGain specifies the gain constant for synchronizing motor1 and motor2.
     * @param pidCtrl specifies the PID controller object.
     * @param defCalPower specifies the default motor power for the calibration.
     * @param powerCompensation specifies the object that implements the PowerCompensation interface, null if none
     *                          provided.
     */
    public TrcPidMotor(
        String instanceName, TrcMotor motor1, TrcMotor motor2, double syncGain, TrcPidController pidCtrl,
        double defCalPower, PowerCompensation powerCompensation)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        if (motor1 == null && motor2 == null)
        {
            throw new IllegalArgumentException("Must have at least one motor.");
        }

        if (pidCtrl == null)
        {
            throw new IllegalArgumentException("Must provide a PID controller.");
        }

        this.instanceName = instanceName;
        this.motor1 = motor1;
        this.motor2 = motor2;
        this.syncGain = syncGain;
        this.pidCtrl = pidCtrl;
        this.defCalPower = -Math.abs(defCalPower);
        this.powerCompensation = powerCompensation;
        TrcTaskMgr taskMgr = TrcTaskMgr.getInstance();
        pidMotorTaskObj = taskMgr.createTask(instanceName + ".pidMotorTask", this::pidMotorTask);
        stopMotorTaskObj = taskMgr.createTask(instanceName + ".stopMotorTask", this::stopMotorTask);
    }   //TrcPidMotor

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param motor1 specifies motor1 object.
     * @param motor2 specifies motor2 object. If there is only one motor, this can be set to null.
     * @param pidCtrl specifies the PID controller object.
     * @param defCalPower specifies the default motor power for the calibration.
     * @param powerCompensation specifies the object that implements the PowerCompensation interface, null if none
     *                          provided.
     */
    public TrcPidMotor(
        String instanceName, TrcMotor motor1, TrcMotor motor2, TrcPidController pidCtrl, double defCalPower,
        PowerCompensation powerCompensation)
    {
        this(instanceName, motor1, motor2, 0.0, pidCtrl, defCalPower, powerCompensation);
    }   //TrcPidMotor

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param motor specifies motor object.
     * @param pidCtrl specifies the PID controller object.
     * @param defCalPower specifies the default motor power for the calibration.
     * @param powerCompensation specifies the object that implements the PowerCompensation interface, null if none
     *                          provided.
     */
    public TrcPidMotor(
        String instanceName, TrcMotor motor, TrcPidController pidCtrl, double defCalPower,
        PowerCompensation powerCompensation)
    {
        this(instanceName, motor, null, 0.0, pidCtrl, defCalPower, powerCompensation);
    }   //TrcPidMotor

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param motor1 specifies motor1 object.
     * @param motor2 specifies motor2 object. If there is only one motor, this can be set to null.
     * @param syncGain specifies the gain constant for synchronizing motor1 and motor2.
     * @param pidCtrl specifies the PID controller object.
     * @param defCalPower specifies the default motor power for the calibration.
     */
    public TrcPidMotor(
        String instanceName, TrcMotor motor1, TrcMotor motor2, double syncGain, TrcPidController pidCtrl,
        double defCalPower)
    {
        this(instanceName, motor1, motor2, syncGain, pidCtrl, defCalPower, null);
    }   //TrcPidMotor

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param motor1 specifies motor1 object.
     * @param motor2 specifies motor2 object. If there is only one motor, this can be set to null.
     * @param pidCtrl specifies the PID controller object.
     * @param defCalPower specifies the default motor power for the calibration.
     */
    public TrcPidMotor(
            String instanceName, TrcMotor motor1, TrcMotor motor2, TrcPidController pidCtrl, double defCalPower)
    {
        this(instanceName, motor1, motor2, 0.0, pidCtrl, defCalPower, null);
    }   //TrcPidMotor

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param motor specifies motor object.
     * @param pidCtrl specifies the PID controller object.
     * @param defCalPower specifies the default motor power for the calibration.
     */
    public TrcPidMotor(String instanceName, TrcMotor motor, TrcPidController pidCtrl, double defCalPower)
    {
        this(instanceName, motor, null, 0.0, pidCtrl, defCalPower, null);
    }   //TrcPidMotor

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
     * This method sets the message tracer for logging trace messages.
     *
     * @param tracer specifies the tracer for logging messages.
     * @param tracePidInfo specifies true to enable tracing of PID info, false otherwise.
     * @param battery specifies the battery object to get battery info for the message.
     */
    public synchronized void setMsgTracer(TrcDbgTrace tracer, boolean tracePidInfo, TrcRobotBattery battery)
    {
        this.msgTracer = tracer;
        this.tracePidInfo = tracePidInfo;
        this.battery = battery;
    }   //setMsgTracer

    /**
     * This method sets the message tracer for logging trace messages.
     *
     * @param tracer specifies the tracer for logging messages.
     * @param tracePidInfo specifies true to enable tracing of PID info, false otherwise.
     */
    public void setMsgTracer(TrcDbgTrace tracer, boolean tracePidInfo)
    {
        setMsgTracer(tracer, tracePidInfo, null);
    }   //setMsgTracer

    /**
     * This method sets the message tracer for logging trace messages.
     *
     * @param tracer specifies the tracer for logging messages.
     */
    public void setMsgTracer(TrcDbgTrace tracer)
    {
        setMsgTracer(tracer, false, null);
    }   //setMsgTracer

    /**
     * This method returns the specified motor object.
     *
     * @param primary specifies true to get the primary motor object, false to get the secondary.
     * @return specified motor object.
     */
    public TrcMotor getMotor(boolean primary)
    {
        final String funcName = "getMotor";
        TrcMotor motor = primary? motor1: motor2;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "primary=%b", primary);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", motor);
        }

        return motor;
    }   //getMotor

    /**
     * This method returns the primary motor object.
     *
     * @return primary motor object.
     */
    public TrcMotor getMotor()
    {
        return getMotor(true);
    }   //getMotor

    /**
     * This method returns the state of the PID motor.
     *
     * @return true if PID motor is active, false otherwise.
     */
    public synchronized boolean isActive()
    {
        final String funcName = "isActive";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", active);
        }

        return active;
    }   //isActive

    /**
     * This method cancels a previous active PID motor operation.
     */
    public synchronized void cancel()
    {
        final String funcName = "cancel";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (active)
        {
            //
            // Stop the physical motor(s). If there is a notification event, signal it canceled.
            //
            stop(true);
            if (notifyEvent != null)
            {
                notifyEvent.cancel();
                notifyEvent = null;
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //cancel

    /**
     * This method sets the position scale. Instead of setting PID target with units such as encoder count, one could
     * set the scale to convert the unit to something meaningful such as inches or degrees.
     *
     * @param positionScale specifies the position scale value.
     * @param positionOffset specifies the optional offset that adds to the final position value.
     */
    public void setPositionScale(double positionScale, double positionOffset)
    {
        final String funcName = "setPositionScale";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                    "scale=%f,offset=%f", positionScale, positionOffset);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.positionScale = positionScale;
        this.positionOffset = positionOffset;
    }   //setPositionScale

    /**
     * This method sets the position scale. Instead of setting PID target with units such as encoder count, one could
     * set the scale to convert the unit to something meaningful such as inches or degrees.
     *
     * @param positionScale specifies the position scale value.
     */
    public void setPositionScale(double positionScale)
    {
        setPositionScale(positionScale, 0.0);
    }   //setPositionScale

    /**
     * This method returns the current scaled motor position.
     *
     * @return scaled motor position.
     */
    public double getPosition()
    {
        final String funcName = "getPosition";
        int n = 1;
        double pos = motor1.getPosition();

        if (motor2 != null && syncGain != 0.0)
        {
            pos += motor2.getPosition();
            n++;
        }
        pos *= positionScale/n;
        pos += positionOffset;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", pos);
        }

        return pos;
    }   //getPosition

    /**
     * This method sets the beep device and the beep tones so that it can play beeps when motor stalled or if the
     * limit switches are activated/deactivated.
     *
     * @param beepDevice specifies the beep device object.
     * @param beepLowFrequency specifies the low frequency beep.
     * @param beepHighFrequency specifies the high frequency beep.
     * @param beepDuration specifies the beep duration.
     */
    public synchronized void setBeep(TrcTone beepDevice, double beepLowFrequency, double beepHighFrequency, double beepDuration)
    {
        final String funcName = "setBeep";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                                "beep=%s,lowFreq=%.0f,hiFreq=%.0f,duration=%.3f",
                                beepDevice.toString(), beepLowFrequency, beepHighFrequency, beepDuration);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.beepDevice = beepDevice;
        this.beepLowFrequency = beepLowFrequency;
        this.beepHighFrequency = beepHighFrequency;
        this.beepDuration = beepDuration;
    }   //setBeep

    /**
     * This method sets the beep device so that it can play beeps at default frequencies and duration when motor
     * stalled or if the limit switches are activated/deactivated.
     *
     * @param beepDevice specifies the beep device object.
     */
    public void setBeep(TrcTone beepDevice)
    {
        setBeep(beepDevice, DEF_BEEP_LOW_FREQUENCY, DEF_BEEP_HIGH_FREQUECY, DEF_BEEP_DURATION);
    }   //setBeep

    /**
     * This method sets stall protection. When stall protection is turned ON, it will monitor the motor movement for
     * stalled condition. A motor is considered stalled if:
     * - the power applied to the motor is above or equal to stallMinPower.
     * - the motor has not moved for at least stallTimeout.
     *
     * Note: By definition, holding target position is stalling. If you decide to enable stall protection while
     *       holding target, please make sure to set a stallMinPower much greater the power necessary to hold
     *       position against gravity, for example.
     *
     * @param stallMinPower specifies the minimum motor power to detect stalled condition. If the motor power is
     *                      below stallMinPower, it won't consider it as a stalled condition even if the motor does
     *                      not move.
     * @param stallTimeout specifies the time in seconds that the motor must stopped before it is declared stalled.
     * @param resetTimeout specifies the time in seconds the motor must be set to zero power after it is declared
     *                     stalled will the stalled condition be reset. If this is set to zero, the stalled condition
     *                     won't be cleared.
     */
    public synchronized void setStallProtection(double stallMinPower, double stallTimeout, double resetTimeout)
    {
        final String funcName = "setStallProtection";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                                "stallMinPower=%f,stallTimeout=%f,resetTimeout=%f",
                                stallMinPower, stallTimeout, resetTimeout);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.stallMinPower = stallMinPower;
        this.stallTimeout = stallTimeout;
        this.resetTimeout = resetTimeout;
    }   //setStallProtection

    /**
     * This method starts a PID operation by setting the PID target.
     *
     * @param target specifies the PID target.
     * @param holdTarget specifies true to hold target after PID operation is completed.
     * @param event specifies an event object to signal when done.
     * @param timeout specifies a timeout value in seconds. If the operation is not completed without the specified
     *                timeout, the operation will be canceled and the event will be signaled. If no timeout is
     *                specified, it should be set to zero.
     */
    private synchronized void setTarget(double target, boolean holdTarget, TrcEvent event, double timeout)
    {
        final String funcName = "setTarget";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                                "target=%f,hold=%s,event=%s,timeout=%f",
                                target, Boolean.toString(holdTarget), event != null? event.toString(): "null", timeout);
        }

        if (active)
        {
            //
            // A previous PID operation in progress, stop it but don't stop the motor to prevent jerkiness.
            //
            stop(false);
        }

        //
        // Set a new PID target.
        //
        pidCtrl.setTarget(target);

        //
        // If a notification event is provided, clear it.
        //
        if (event != null)
        {
            event.clear();
        }

        notifyEvent = event;
        expiredTime = timeout;
        this.holdTarget = holdTarget;
        //
        // If a timeout is provided, set the expired time.
        //
        if (timeout != 0.0)
        {
            expiredTime += TrcUtil.getCurrentTime();
        }
        //
        // Set the PID motor task enabled.
        //
        setTaskEnabled(true);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setTarget

    /**
     * This method starts a PID operation by setting the PID target.
     *
     * @param target specifies the PID target.
     * @param event specifies an event object to signal when done.
     * @param timeout specifies a timeout value in seconds. If the operation is not completed without the specified
     *                timeout, the operation will be canceled and the event will be signaled. If no timeout is
     *                specified, it should be set to zero.
     */
    public void setTarget(double target, TrcEvent event, double timeout)
    {
        setTarget(target, false, event, timeout);
    }   //setTarget

    /**
     * This method starts a PID operation by setting the PID target.
     *
     * @param target specifies the PID target.
     * @param holdTarget specifies true to hold target after PID operation is completed.
     */
    public void setTarget(double target, boolean holdTarget)
    {
        setTarget(target, holdTarget, null, 0.0);
    }   //setTarget

    /**
     * This method performs motor stall detection to protect the motor from burning out. A motor is considered stalled
     * when at least stallMinPower is applied to the motor and the motor is not turning for at least stallTimeout.
     *
     * @param power specifies the power applying to the motor.
     */
    private void performStallDetection(double power)
    {
        final String funcName = "performStallDetection";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC, "power=%f", power);
        }

        if (stallMinPower > 0.0 && stallTimeout > 0.0)
        {
            //
            // Stall protection is ON, check for stall condition.
            // - power is above stallMinPower
            // - motor has not moved for at least stallTimeout.
            //
            double currPos = getPosition();
            if (Math.abs(power) < Math.abs(stallMinPower) || currPos != prevPos || prevTime == 0.0)
            {
                prevPos = currPos;
                prevTime = TrcUtil.getCurrentTime();
            }

            if (TrcUtil.getCurrentTime() - prevTime > stallTimeout)
            {
                //
                // We have detected a stalled condition for at least stallTimeout. Kill power to protect
                // the motor.
                //
                motorPower = 0.0;
                stalled = true;
                if (beepDevice != null)
                {
                    beepDevice.playTone(beepHighFrequency, beepDuration);
                }

                if (msgTracer != null)
                {
                    msgTracer.traceInfo(funcName, "%s: stalled", instanceName);
                }
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC);
        }
    }   //performStallDetection

    /**
     * This method checks if the motor was stalled and if power has been removed for at least resetTimeout, the
     * stalled condition is then cleared.
     *
     * @param power specifies the power applying to the motor.
     * @return true if the motor was stalled, false otherwise.
     */
    private boolean resetStall(double power)
    {
        final String funcName = "resetStall";
        boolean wasStalled = stalled;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC);
        }

        if (stalled)
        {
            if (power == 0.0)
            {
                //
                // We had a stalled condition but if power is removed for at least reset timeout, we clear the
                // stalled condition.
                //
                if (resetTimeout == 0.0 || TrcUtil.getCurrentTime() - prevTime > resetTimeout)
                {
                    prevPos = getPosition();
                    prevTime = TrcUtil.getCurrentTime();
                    stalled = false;
                    if (beepDevice != null)
                    {
                        beepDevice.playTone(beepLowFrequency, beepDuration);
                    }
                }
            }
            else
            {
                prevTime = TrcUtil.getCurrentTime();
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC, "=%s", wasStalled);
        }

        return wasStalled;
    }   //resetStall

    /**
     * This method sets the PID motor power. It will also check for stalled condition and cut motor power if stalled
     * detected. It will also check to reset the stalled condition if reset timeout was specified.
     *
     * @param power specifies the motor power.
     * @param rangeLow specifies the power range low limit.
     * @param rangeHigh specifies the power range high limit.
     * @param stopPid specifies true to stop previous PID operation, false otherwise.
     */
    private synchronized void setPower(double power, double rangeLow, double rangeHigh, boolean stopPid)
    {
        final String funcName = "setPower";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC,
                                "power=%f,rangeLow=%f,rangeHigh=%f,stopPid=%s",
                                power, rangeLow, rangeHigh, Boolean.toString(stopPid));
        }
        //
        // Note: this method does not handle zero calibration, so do not call this method in zero calibration mode.
        //
        if (active && stopPid)
        {
            //
            // A previous PID operation is still in progress, cancel it. Don't stop the motor to prevent jerkiness.
            //
            stop(false);
        }

        if (!resetStall(power))
        {
            //
            // Motor was not stalled. Do the normal processing.
            //
            if (powerCompensation != null)
            {
                power += powerCompensation.getCompensation();
            }
            power = TrcUtil.clipRange(power, rangeLow, rangeHigh);
            motorPower = power;
            performStallDetection(power);
            setMotorPower(motorPower);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC);
        }
    }   //setPower

    /**
     * This method sets the PID motor power. It will check for the limit switches. If activated, it won't allow the
     * motor to go in that direction. It will also check for stalled condition and cut motor power if stalled detected.
     * It will also check to reset the stalled condition if reset timeout was specified.
     *
     * @param power specifies the motor power.
     * @param rangeLow specifies the range low limit.
     * @param rangeHigh specifies the range high limit.
     */
    public void setPower(double power, double rangeLow, double rangeHigh)
    {
        setPower(power, rangeLow, rangeHigh, true);
    }   //setPower

    /**
     * This method sets the PID motor power. It will check for the limit switches. If activated, it won't allow the
     * motor to go in that direction. It will also check for stalled condition and cut motor power if stalled detected.
     * It will also check to reset the stalled condition if reset timeout was specified.
     *
     * @param power specifies the motor power.
     */
    public void setPower(double power)
    {
        setPower(power, MIN_MOTOR_POWER, MAX_MOTOR_POWER, true);
    }   //setPower

    /**
     * This method sets the motor power with PID control. The motor will be under PID control and the power specifies
     * the upper bound of how fast the motor will spin. The actual motor power is controlled by a PID controller with
     * the target either set to minPos or maxPos depending on the direction of the motor. This is very useful in
     * scenarios such as an elevator where you want to have the elevator controlled by a joystick but would like PID
     * control to pay attention to the upper and lower limits and slow down when approaching those limits. The joystick
     * value will specify the upper bound of the elevator power. So if the joystick is only pushed half way, the
     * elevator will only go half power even though it is far away from the target.
     *
     * @param power specifies the upper bound power of the motor.
     * @param minPos specifies the minimum of the position range.
     * @param maxPos specifies the maximum of the position range.
     * @param holdTarget specifies true to hold target when speed is set to 0, false otherwise.
     */
    public void setPowerWithinPosRange(double power, double minPos, double maxPos, boolean holdTarget)
    {
        final String funcName = "setPowerWithinPosRange";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                                "power=%.2f,minPos=%.1f,maxPos=%.1f,hold=%s",
                                power, minPos, maxPos, Boolean.toString(holdTarget));
        }
        //
        // If power is negative, set the target to minPos. If power is positive, set the target to maxPos. We only
        // set a new target if the target has changed. (i.e. either the motor changes direction, starting or stopping).
        //
        double currTarget = power < 0.0? minPos: power > 0.0? maxPos: minPos - 1.0;
        if (currTarget != prevTarget)
        {
            if (power == 0.0)
            {
                //
                // We are stopping, Relax the power range to max range so we have full power to hold target if
                // necessary.
                //
                pidCtrl.setOutputRange(MIN_MOTOR_POWER, MAX_MOTOR_POWER);
                if (holdTarget)
                {
                    //
                    // Hold target at current position.
                    //
                    setTarget(getPosition(), true, null, 0.0);
                }
                else
                {
                    //
                    // We reached target and no holding target, we are done.
                    //
                    cancel();
                }
            }
            else
            {
                //
                // We changed direction, change the target.
                //
                power = Math.abs(power);
                pidCtrl.setOutputRange(-power, power);
                setTarget(currTarget, holdTarget, null, 0.0);
            }
            prevTarget = currTarget;
        }
        else if (power == 0.0)
        {
            //
            // We remain stopping, keep the power range relaxed in case we are holding previous target.
            //
            pidCtrl.setOutputRange(MIN_MOTOR_POWER, MAX_MOTOR_POWER);
        }
        else
        {
            //
            // Direction did not change but we need to update the power range.
            //
            power = Math.abs(power);
            pidCtrl.setOutputRange(-power, power);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setPowerWithinPosRange

    /**
     * This method starts zero calibration mode by moving the motor with specified calibration power until a limit
     * switch is hit.
     *
     * @param calPower specifies the motor power to use for the zero calibration overriding the default calibration
     *                 power specified in the constructor.
     */
    public synchronized void zeroCalibrate(double calPower)
    {
        final String funcName = "zeroCalibrate";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "calPower=%f", calPower);
        }

        //
        // Calibration power is always negative. Motor 1 always has a lower limit switch. If there is a motor 2,
        // motor 2 has a lower limit switch only if it is independent of motor 1 and needs synchronizing with motor 1.
        //
        this.calPower = -Math.abs(calPower);
        calibrating = true;
        motor1ZeroCalDone = false;
        motor2ZeroCalDone = motor2 == null || syncGain == 0.0;
        setTaskEnabled(true);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //zeroCalibrate

    /**
     * This method starts zero calibration mode by moving the motor with specified calibration power until a limit
     * switch is hit.
     */
    public synchronized void zeroCalibrate()
    {
        zeroCalibrate(defCalPower);
    }   //zeroCalibrate

    /**
     * This method sets the motor power. If there are two motors, it will set both.
     *
     * @param power specifies the motor power.
     */
    private void setMotorPower(double power)
    {
        final String funcName = "setMotorPower";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "power=%f", power);
        }

        power = TrcUtil.clipRange(power, MIN_MOTOR_POWER, MAX_MOTOR_POWER);

        if (power == 0.0 || syncGain == 0.0)
        {
            //
            // If we are not sync'ing or stopping, just set the motor power. If we are stopping the motor, even if
            // we are sync'ing, we should just stop. But we should still observe the limit switches.
            //
            motor1.set(power);
            if (motor2 != null)
            {
                motor2.set(power);
            }
        }
        else
        {
            //
            // We are sync'ing the two motors and the motor power is not zero.
            //
            double pos1 = motor1.getPosition();
            double pos2 = motor2.getPosition();
            double deltaPower = TrcUtil.clipRange((pos2 - pos1)*syncGain);
            double power1 = power + deltaPower;
            double power2 = power - deltaPower;
            double minPower = Math.min(power1, power2);
            double maxPower = Math.max(power1, power2);
            double scale = maxPower > MAX_MOTOR_POWER? maxPower: minPower < MIN_MOTOR_POWER? -minPower: 1.0;
            //
            // We don't want the motors to switch direction in order to sync. It will worsen oscillation.
            // So make sure the motor powers are moving in the same direction.
            //
            if (power > 0.0)
            {
                power1 = TrcUtil.clipRange(power1/scale, 0.0, MAX_MOTOR_POWER);
                power2 = TrcUtil.clipRange(power2/scale, 0.0, MAX_MOTOR_POWER);
            }
            else
            {
                power1 = TrcUtil.clipRange(power1, MIN_MOTOR_POWER, 0.0);
                power2 = TrcUtil.clipRange(power2, MIN_MOTOR_POWER, 0.0);
            }

            motor1.set(power1);
            motor2.set(power2);

            if (debugEnabled)
            {
                dbgTrace.traceInfo(funcName, "P=%.2f,dP=%.2f,pos1=%.0f,pos2=%.0f,P1=%.2f,P2=%.2f",
                                   power, deltaPower, pos1, pos2, power1, power2);
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setMotorPower

    /**
     * This method stops the PID motor. Stopping a PID motor consists of two things: canceling PID and stopping
     * the physical motor(s).
     *
     * @param stopMotor specifies true if also stopping the physical motor(s), false otherwise.
     */
    private synchronized void stop(boolean stopMotor)
    {
        final String funcName = "stop";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC, "stopMotor=%s", Boolean.toString(stopMotor));
        }
        //
        // Canceling previous PID operation if any.
        //
        setTaskEnabled(false);
        pidCtrl.reset();

        if (stopMotor)
        {
            setMotorPower(0.0);
        }

        motorPower = 0.0;
        calibrating = false;
        stalled = false;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC);
        }
    }   //stop

    /**
     * This method activates/deactivates a PID motor operation by enabling/disabling the PID motor task.
     *
     * @param enabled specifies true to activate a PID motor operation, false otherwise.
     */
    private void setTaskEnabled(boolean enabled)
    {
        final String funcName = "setTaskEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC, "enabled=%b", enabled);
        }

        if (enabled)
        {
            pidMotorTaskObj.registerTask(TrcTaskMgr.TaskType.OUTPUT_TASK);
            stopMotorTaskObj.registerTask(TrcTaskMgr.TaskType.STOP_TASK);
        }
        else
        {
            pidMotorTaskObj.unregisterTask(TrcTaskMgr.TaskType.OUTPUT_TASK);
            stopMotorTaskObj.unregisterTask(TrcTaskMgr.TaskType.STOP_TASK);
        }
        this.active = enabled;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC);
        }
    }   //setTaskEnabled

    /**
     * This method is called by the calibration task to zero calibrate of a motor.
     *
     * @param motor specifies the motor being calibrated.
     * @return true if calibration is done, false otherwise.
     */
    private boolean zeroCalibratingMotor(TrcMotor motor)
    {
        final String funcName = "zeroCalibratingMotor";
        boolean done = motor.isLowerLimitSwitchActive() || stalled;

        if (done)
        {
            //
            // Done with zero calibration of the motor. Call the motor directly to stop, do not call any of
            // the setPower or setMotorPower because they do not handle zero calibration mode.
            //
            if (stalled)
            {
                if (beepDevice != null)
                {
                    beepDevice.playTone(beepLowFrequency, beepDuration);
                }
                TrcDbgTrace.getGlobalTracer().traceWarn(
                    funcName, "%s is stalled, lower limit switch might have failed!", instanceName);
            }
            stalled = false;
            motor.set(0.0);
            motor.resetPosition(false);
        }
        else
        {
            performStallDetection(calPower);
            motor.set(calPower);
        }

        return done;
    }   //zeroCalibratingMotor

    /**
     * This method is called periodically to perform the PID motor task. The PID motor task can be in one of two
     * modes: zero calibration mode and normal mode. In zero calibration mode, it will drive the motor with the
     * specified calibration power until it hits the lower limit switch. Then it will stop the motor and reset the
     * motor position sensor. In normal mode, it calls the PID control to calculate and set the motor power. It also
     * checks if the motor has reached the set target and disables the task.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is running.
     */
    private synchronized void pidMotorTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = "pidMotorTask";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "taskType=%s,runMode=%s", taskType, runMode);
        }

        if (calibrating)
        {
            //
            // We are in zero calibration mode.
            //
            if (!motor1ZeroCalDone)
            {
                motor1ZeroCalDone = zeroCalibratingMotor(motor1);
            }

            if (!motor2ZeroCalDone)
            {
                motor2ZeroCalDone = zeroCalibratingMotor(motor2);
            }

            if (motor1ZeroCalDone && motor2ZeroCalDone)
            {
                //
                // Done with zero calibration.
                //
                calibrating = false;
                setTaskEnabled(false);
            }
        }
        else
        {
            if (stalled ||
                !holdTarget && pidCtrl.isOnTarget() ||
                expiredTime != 0.0 && TrcUtil.getCurrentTime() >= expiredTime)
            {
                //
                // We stop the motor if we either:
                // - are stalled
                // - have reached target and not holding target position
                // - set a timeout and it has expired.
                //
                stop(true);
                if (notifyEvent != null)
                {
                    notifyEvent.set(true);
                    notifyEvent = null;
                }
            }
            else
            {
                //
                // We are still in business. Call PID controller to calculate the motor power and set it.
                //
                motorPower = pidCtrl.getOutput();
                setPower(motorPower, MIN_MOTOR_POWER, MAX_MOTOR_POWER, false);

                if (msgTracer != null && tracePidInfo)
                {
                    pidCtrl.printPidInfo(msgTracer, verbosePidInfo, battery);
                }
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //pidMotorTask

    /**
     * This method is called when the competition mode is about to end to stop the PID motor operation if any.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is running.
     */
    private void stopMotorTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = "stopMotorTask";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "taskType=%s,runMode=%s", taskType, runMode);
        }


        stop(true);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //stopMotorTask

}   //class TrcPidMotor
