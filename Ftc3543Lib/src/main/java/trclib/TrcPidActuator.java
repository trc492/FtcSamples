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
 * This class implements a platform independent PID controlled actuator extending TrcPidMotor. It consists of a motor,
 * an encoder to keep track of its position, a lower limit switch to detect the zero position and a PID controller
 * allowing accurate movement to a set position. It provides methods to allow a joystick to control the actuator
 * to extend/retract or rotate within its limited range of movement and will slow down and finally stop when lower or
 * upper limit has been reached. It also provides methods to move the actuator to a specified position and hold it
 * there under load if necessary.
 * The PID controlled actuator class supports both linear and non-linear actuators. Elevator is an example of linear
 * actuators. Rotational arm is an example of non-linear actuators where raising and lowering it against gravity
 * presents non-linear load as the arm angle changes. PID control is good at controlling load with linear relationship.
 * Therefore, PID control will yield terrible result in non-linear situation. However, if we add a compensation factor
 * to linearize the load, then we can still achieve good result with PID control. The power compensation factor is
 * provided by the caller and is a function of the actuator position.
 */
public class TrcPidActuator extends TrcPidMotor
{
    private double minPos;
    private double maxPos;
    private boolean manualOverride = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param motor1 specifies the primary motor in the actuator.
     * @param motor2 specifies the secondary motor in the actuator.
     * @param syncGain specifies the sync gain between the primary and secondary motors (i.e. Kp).
     * @param lowerLimitSwitch specifies the optional lower limit switch. Required only for auto zero calibration
     *        whenever it is active.
     * @param pidCtrl specifies the PID controller for PID controlled movement.
     * @param calPower specifies the motor power for the calibration.
     * @param minPos specifies the minimum position of the actuator range.
     * @param maxPos specifies the maximum position of the actuator range.
     * @param powerCompensation specifies the object that implements the PowerCompensation interface, null if none
     *                          provided.
     */
    public TrcPidActuator(
            String instanceName, TrcMotor motor1, TrcMotor motor2, double syncGain, TrcDigitalInput lowerLimitSwitch,
            TrcPidController pidCtrl, double calPower, double minPos, double maxPos,
            PowerCompensation powerCompensation)
    {
        super(instanceName, motor1, motor2, syncGain, pidCtrl, calPower, powerCompensation);
        this.minPos = minPos;
        this.maxPos = maxPos;
        if (lowerLimitSwitch != null)
        {
            motor1.resetPositionOnDigitalInput(lowerLimitSwitch);
        }
        pidCtrl.setAbsoluteSetPoint(true);
    }   //TrcPidActuator

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param motor specifies the motor in the actuator.
     * @param lowerLimitSwitch specifies the optional lower limit switch. Required only for auto zero calibration
     *        whenever it is active.
     * @param pidCtrl specifies the PID controller for PID controlled movement.
     * @param calPower specifies the motor power for the calibration.
     * @param minPos specifies the minimum position of the actuator range.
     * @param maxPos specifies the maximum position of the actuator range.
     * @param powerCompensation specifies the object that implements the PowerCompensation interface, null if none
     *                          provided.
     */
    public TrcPidActuator(
            String instanceName, TrcMotor motor, TrcDigitalInput lowerLimitSwitch, TrcPidController pidCtrl,
            double calPower, double minPos, double maxPos, PowerCompensation powerCompensation)
    {
        this(instanceName, motor, null, 0.0, lowerLimitSwitch, pidCtrl, calPower, minPos, maxPos, powerCompensation);
    }   //TrcPidActuator

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param motor specifies the motor in the actuator.
     * @param lowerLimitSwitch specifies the optional lower limit switch. Required only for auto zero calibration
     *        whenever it is active.
     * @param pidCtrl specifies the PID controller for PID controlled movement.
     * @param calPower specifies the motor power for the calibration.
     * @param minPos specifies the minimum position of the actuator range.
     * @param maxPos specifies the maximum position of the actuator range.
     */
    public TrcPidActuator(
            String instanceName, TrcMotor motor, TrcDigitalInput lowerLimitSwitch, TrcPidController pidCtrl,
            double calPower, double minPos, double maxPos)
    {
        this(instanceName, motor, null, 0.0, lowerLimitSwitch, pidCtrl, calPower, minPos, maxPos, null);
    }   //TrcPidActuator

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param motor specifies the motor in the actuator.
     * @param lowerLimitSwitch specifies the optional lower limit switch. Required only for auto zero calibration
     *        whenever it is active.
     * @param pidCtrl specifies the PID controller for PID controlled movement.
     * @param calPower specifies the motor power for the calibration.
     */
    public TrcPidActuator(
            String instanceName, TrcMotor motor, TrcDigitalInput lowerLimitSwitch, TrcPidController pidCtrl,
            double calPower)
    {
        this(instanceName, motor, null, 0.0, lowerLimitSwitch, pidCtrl, calPower, 0.0, 0.0, null);
    }   //TrcPidActuator

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param motor specifies the motor in the actuator.
     * @param pidCtrl specifies the PID controller for PID controlled movement.
     * @param calPower specifies the motor power for the calibration.
     * @param minPos specifies the minimum position of the actuator range.
     * @param maxPos specifies the maximum position of the actuator range.
     * @param powerCompensation specifies the object that implements the PowerCompensation interface, null if none
     *                          provided.
     */
    public TrcPidActuator(
            String instanceName, TrcMotor motor, TrcPidController pidCtrl, double calPower, double minPos,
            double maxPos, PowerCompensation powerCompensation)
    {
        this(instanceName, motor, null, 0.0, null, pidCtrl, calPower, minPos, maxPos, powerCompensation);
    }   //TrcPidActuator

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param motor specifies the motor in the actuator.
     * @param pidCtrl specifies the PID controller for PID controlled movement.
     * @param calPower specifies the motor power for the calibration.
     * @param minPos specifies the minimum position of the actuator range.
     * @param maxPos specifies the maximum position of the actuator range.
     */
    public TrcPidActuator(
            String instanceName, TrcMotor motor, TrcPidController pidCtrl, double calPower, double minPos,
            double maxPos)
    {
        this(instanceName, motor, null, 0.0, null, pidCtrl, calPower, minPos, maxPos, null);
    }   //TrcPidActuator

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param motor specifies the motor in the actuator.
     * @param pidCtrl specifies the PID controller for PID controlled movement.
     * @param calPower specifies the motor power for the calibration.
     */
    public TrcPidActuator(
            String instanceName, TrcMotor motor, TrcPidController pidCtrl, double calPower)
    {
        this(instanceName, motor, null, 0.0, null, pidCtrl, calPower, 0.0, 0.0, null);
    }   //TrcPidActuator

    /**
     * This method returns the state of manual override.
     *
     * @return true if manual override is ON, false otherwise.
     */
    public boolean isManualOverride()
    {
        final String funcName = "isManualOverride";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", manualOverride);
        }

        return manualOverride;
    }   //isManualOverride

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
     * @param hold specifies true to hold position when power is zero, false otherwise.
     */
    public void setPower(double power, boolean hold)
    {
        final String funcName = "setPower";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "power=%s", power);
        }

        if (manualOverride || minPos == 0.0 && maxPos == 0.0)
        {
            cancel();
            super.setPower(power);
        }
        else
        {
            setPowerWithinPosRange(power, minPos, maxPos, hold);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setPower

    /**
     * This method runs the actuator with the specified power. It will hold the current position even if power is zero.
     * Note that if position range is not set, PID control will be disabled.
     *
     * @param power specifies the power to run the actuator.
     */
    @Override
    public void setPower(double power)
    {
        setPower(power, false);
    }   //setPower

}   //class TrcPidActuator
