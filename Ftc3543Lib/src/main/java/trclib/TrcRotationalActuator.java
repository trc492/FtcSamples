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
 * This class implements a platform independent rotational actuator that extends TrcLinearActuator. Rotational
 * actuator may not be linear. For example, if the rotational actuator is an arm raising and lowering against
 * gravity, the load will be changing according to the angle of the arm. PID control is good at controlling
 * load with linear relationship. Therefore, PID control will yield terrible result in this situation. However,
 * if we add a compensation factor to linearize the load, then we can still achieve good result with PID control.
 * This class is doing just that.
 */
public class TrcRotationalActuator extends TrcLinearActuator
{
    private static final String moduleName = "TrcRotationalActuator";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    public interface PowerCompensation
    {
        /**
         * This method is called to get compensation power that adds to the power value when doing a set power on
         * the motor.
         *
         * @return compensation power value.
         */
        double getCompensation();

    }   //interface PowerCompensation

    private PowerCompensation powerCompensation;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param motor specifies the motor in the actuator.
     * @param lowerLimitSwitch specifies the lower limit switch. Required for zero calibration.
     * @param pidCtrl specifies the PID controller for PID controlled movement.
     * @param powerCompensation specifies who to call to get power compensation value.
     */
    public TrcRotationalActuator(
            final String instanceName, TrcMotor motor, TrcDigitalInput lowerLimitSwitch,
            TrcPidController pidCtrl, PowerCompensation powerCompensation)
    {
        super(instanceName, motor, lowerLimitSwitch, pidCtrl);

        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(
                    moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.powerCompensation = powerCompensation;
    }   //TrcRotationalActuator

    /**
     * This method runs the actuator with the specified power. It will hold the current position even if power is zero.
     * Note that if position range is not set, PID control will be disabled.
     *
     * @param power specifies the power to run the actuator.
     */
    @Override
    public void setPower(double power)
    {
        final String funcName = "setPower";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "power=%s", power);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        super.setPower(power + powerCompensation.getCompensation());
    }   //setPower

}   //class TrcRotationalActuator
