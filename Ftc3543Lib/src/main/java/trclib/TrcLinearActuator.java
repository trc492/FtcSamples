/*
 * Copyright (c) 2017 Titan Robotics Club (http://www.titanrobotics.com)
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
 * This class implements a platform independent linear actuator which consists of a motor, an encoder to keep track
 * of its position, a lower limit switch to detect the zero position and a PID controller allowing accurate movement
 * to a set position. It provides methods to allow a joystick to control the actuator to extend and retract within
 * its limited range of movement and will slow down and finally stop when lower or upper limit has been reached. It
 * also provides methods to move the actuator to a specified position and hold it there under load if necessary.
 */
public class TrcLinearActuator implements TrcDigitalTrigger.TriggerHandler, TrcPidController.PidInput
{
    private static final String moduleName = "TrcLinearActuator";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private final String instanceName;
    private TrcMotor motor;
    private TrcDigitalInput lowerLimitSwitch;
    private TrcPidController pidCtrl;
    private TrcDigitalTrigger lowerLimitTrigger;
    private TrcPidMotor pidMotor;
    private double minPos = 0.0;
    private double maxPos = 0.0;
    private boolean manualOverride = false;

    /**
     * This method initializes the linear actuator with a motor, a limit switch and a PID controller.
     *
     * @param motor specifies the motor in the actuator.
     * @param lowerLimitSwitch specifies the lower limit switch keeping track of the zero position.
     * @param pidCtrl specifies the PID controller for PID controlled movement.
     */
    public void initialize(TrcMotor motor, TrcDigitalInput lowerLimitSwitch, TrcPidController pidCtrl)
    {
        final String funcName = "initialize";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                    "motor=%s,limitSw=%s,pidCtrl=%s", motor, lowerLimitSwitch, pidCtrl);
        }

        this.motor = motor;
        this.lowerLimitSwitch = lowerLimitSwitch;
        this.pidCtrl = pidCtrl;
        lowerLimitTrigger = new TrcDigitalTrigger("actuatorLowerLimit", lowerLimitSwitch, this);
        lowerLimitTrigger.setEnabled(true);
        pidMotor = new TrcPidMotor("LinearActuator", motor, pidCtrl);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //initialize

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public TrcLinearActuator(final String instanceName)
    {
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
    }   //TrcLinearActuator

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method sets the position scale scaling the encoder count into proper position units.
     *
     * @param scale specifies the position scale.
     */
    public void setPositionScale(double scale)
    {
        final String funcName = "setPositionScale";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "scale=%f", scale);
        }

        pidMotor.setPositionScale(scale);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setPositionScale

    /**
     * This method sets the position range of the actuator.
     *
     * @param minPos specifies the minimum position of the actuator range.
     * @param maxPos specifies the maximum position of the actuator range.
     */
    public void setPositionRange(double minPos, double maxPos)
    {
        final String funcName = "setPositionRange";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "min=%f,max=%f", minPos, maxPos);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.minPos = minPos;
        this.maxPos = maxPos;
    }   //setPositionRange

    /**
     * This method does a zero calibration on the actuator by slowly retracting until it hits the lower limit switch.
     * Then it stops and resets the motor encoder. This is typically called in the robot initialization or before
     * competition start.
     *
     * @param calPower specifies the motor power to retract the actuator.
     */
    public void zeroCalibrate(double calPower)
    {
        final String funcName = "zeroCalibrate";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "calPower=%f", calPower);
        }

        pidMotor.zeroCalibrate(calPower);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //zeroCalibrate

    /**
     * This method sets manual override mode. This is useful to override PID control of the actuator in situations
     * where the encoder is not zero calibrated or malfunctioning. Note that this only overrides the encoder but not
     * the limit switch. So if the lower limit switch is engaged, the actuator will not retract even though manual
     * override is true.
     *
     * @param manualOverride specifies true for manual override, false otherwise.
     */
    public void setManualOverride(boolean manualOverride)
    {
        final String funcName = "setManualOverride";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                    "manualOverrid=%s", Boolean.toString(manualOverride));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.manualOverride = manualOverride;
    }   //setManualOverride

    /**
     * This method runs the actuator with the specified power. It will hold the current position even if power is zero.
     * Note that if position range is not set, PID control will be disabled.
     *
     * @param power specifies the power to run the actuator.
     */
    public void setPower(double power)
    {
        final String funcName = "setPower";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "power=%s", power);
        }

        if (manualOverride || minPos == 0.0 && maxPos == 0.0)
        {
            motor.setPower(power);
        }
        else
        {
            pidMotor.setSpeed(power, minPos, maxPos, true);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setPower

    /**
     * This method runs the actuator to the specified position and hold it there.
     *
     * @param pos specifies the target position.
     */
    public void setPosition(double pos)
    {
        final String funcName = "setPosition";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "pos=%s", pos);
        }

        pidMotor.setTarget(pos, true);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setPosition

    /**
     * This method runs the actuator to the specified position and signal the given event when done. Optionally, if a
     * timeout is specified, the operation will be aborted when specified time has expired. Note that this method
     * does not hold the actuator in place after reaching target.
     *
     * @param pos specifies the target position.
     * @param event specifies the event to signal when done.
     * @param timeout specifies timeout for the operation. It can be zero, if no timeout is used.
     */
    public void setPosition(double pos, TrcEvent event, double timeout)
    {
        final String funcName = "setPosition";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                    "pos=%s,event=%s,timeout=%f", pos, event, timeout);
        }

        pidMotor.setTarget(pos, event, timeout);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setPosition

    /**
     * This method returns the current position of the actuator.
     *
     * @return current position of the actuator.
     */
    public double getPosition()
    {
        final String funcName = "getPosition";
        double pos = pidMotor.getPosition();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", pos);
        }

        return pos;
    }   //getPosition

    /**
     * This method returns the state of the lower limit switch.
     *
     * @return true if lower limit switch is engaged, false otherwise.
     */
    public boolean isLowerLimitSwitchPressed()
    {
        final String funcName = "isLowerLimitSwitchPressed";
        boolean isPressed = lowerLimitSwitch.isActive();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Boolean.toString(isPressed));
        }

        return isPressed;
    }   //isLowerLimitSwitchPressed

    /**
     * This method displays the actuator PID information to the dashboard on the given line number. Note that the
     * information occupies two dashboard lines.
     *
     * @param lineNum specifies the dashboard line number to display the first line.
     */
    public void displayDebugInfo(int lineNum)
    {
        pidCtrl.displayPidInfo(lineNum);
    }   //displayPidInfo

    //
    // Implements TrcDigitalTrigger.TriggerHandler
    //

    /**
     * This method is called by the DigitalTrigger object notifying us the lower limit switch has changed state.
     * Note that we are resetting the encoders on both pressed and released events. This ensures the encoder starts
     * counting only when the limit switch is disengaged.
     *
     * @param digitalTrigger specifies the DigitalTrigger object that gives us the notification.
     * @param active specifies true if the lower limit switch is pressed, false otherwise.
     */
    @Override
    public void triggerEvent(TrcDigitalTrigger digitalTrigger, boolean active)
    {
        final String funcName = "triggerEvent";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                    "trigger=%s,active=%s", digitalTrigger, Boolean.toString(active));
        }

        if (digitalTrigger == lowerLimitTrigger)
        {
            motor.resetPosition(false);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //triggerEvent

    //
    // Implements TrcPidController.PidInput.
    //

    /**
     * This method is called by the PID controller to get the current height of the elevator.
     *
     * @param pidCtrl specifies the PID controller who is inquiring.
     *
     * @return current elevator height.
     */
    @Override
    public double getInput(TrcPidController pidCtrl)
    {
        double value = 0.0;

        if (pidCtrl == this.pidCtrl)
        {
            value = getPosition();
        }

        return value;
    }   //getInput

}   //class TrcLinearActuator
