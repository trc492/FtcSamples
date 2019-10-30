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

package ftclib;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDcMotorController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import trclib.TrcAnalogInput;
import trclib.TrcDigitalInput;
import trclib.TrcDbgTrace;
import trclib.TrcMotor;

/**
 * This class implements the generic DC Motor Controller extending TrcMotor. It provides implementation of the
 * abstract methods in TrcMotor. It supports limit switches. When this class is constructed with limit switches,
 * setPower will respect them and will not move the motor into the direction where the limit switch is activated.
 * It also provides a software encoder reset without switching the Modern Robotics motor controller mode which is
 * problematic.
 */
public class FtcDcMotor extends TrcMotor
{
    private static final String moduleName = "FtcDcMotor";

    private final String instanceName;
    private final TrcDigitalInput lowerLimitSwitch;
    private final TrcDigitalInput upperLimitSwitch;
    private final TrcAnalogInput analogSensor;
    public final DcMotor motor;
    private int prevEncPos;
    private int zeroEncoderValue;
    private int positionSensorSign = 1;
    private double prevPower = 0.0;
    private boolean softLowerLimitEnabled = false;
    private boolean softUpperLimitEnabled = false;
    private double softLowerLimit = 0.0;
    private double softUpperLimit = 0.0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     * @param lowerLimitSwitch specifies the lower limit switch object.
     * @param upperLimitSwitch specifies the upper limit switch object.
     * @param analogSensor specifies an analog position sensor instead of the motor encoder.
     */
    public FtcDcMotor(HardwareMap hardwareMap, String instanceName,
                      TrcDigitalInput lowerLimitSwitch, TrcDigitalInput upperLimitSwitch, TrcAnalogInput analogSensor)
    {
        super(instanceName);

        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer? globalTracer:
                    new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
        this.lowerLimitSwitch = lowerLimitSwitch;
        this.upperLimitSwitch = upperLimitSwitch;
        this.analogSensor = analogSensor;
        motor = hardwareMap.dcMotor.get(instanceName);
        prevEncPos = motor.getCurrentPosition();
        zeroEncoderValue = prevEncPos;
    }   //FtcDcMotor

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param lowerLimitSwitch specifies the lower limit switch object.
     * @param upperLimitSwitch specifies the upper limit switch object.
     * @param analogSensor specifies an analog position sensor instead of the motor encoder.
     */
    public FtcDcMotor(String instanceName, TrcDigitalInput lowerLimitSwitch, TrcDigitalInput upperLimitSwitch,
                      TrcAnalogInput analogSensor)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, lowerLimitSwitch, upperLimitSwitch, analogSensor);
    }   //FtcDcMotor

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param lowerLimitSwitch specifies the lower limit switch object.
     * @param upperLimitSwitch specifies the upper limit switch object.
     */
    public FtcDcMotor(String instanceName, TrcDigitalInput lowerLimitSwitch, TrcDigitalInput upperLimitSwitch)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, lowerLimitSwitch, upperLimitSwitch, null);
    }   //FtcDcMotor

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param lowerLimitSwitch specifies the lower limit switch object.
     */
    public FtcDcMotor(String instanceName, TrcDigitalInput lowerLimitSwitch)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, lowerLimitSwitch, null, null);
    }   //FtcDcMotor

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FtcDcMotor(String instanceName)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, null, null, null);
    }   //FtcDcMotor

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
     * This method returns the battery voltage that powers the motor. This method is specific to FTC.
     *
     * @return battery voltage.
     */
    public double getVoltage()
    {
        final String funcName = "getVoltage";
        double voltage = ((ModernRoboticsUsbDcMotorController)motor.getController()).getVoltage();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%.3f", voltage);
        }

        return voltage;
    }   //getVoltage

    /**
     * Ensures that the given requested motor power does not violate limit switch constraints (e.g. if the upper limit
     * switch is pressed, the motor may not be commanded forward).
     *
     * If the given value does not violate limit switch constraints, it is returned as-is; otherwise, 0;
     *
     * @param motorPower The requested motor power value.
     * @return The given motor power value respecting limit switch constraints.
     */
    private double constrainMotorPowerByLimitSwitches(double motorPower)
    {
        final String funcName = "constrainMotorPowerByLimitSwitches";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "power=%f", motorPower);
        }

        if (motorPower > 0.0 && (upperLimitSwitch != null && upperLimitSwitch.isActive() ||
                softUpperLimitEnabled && getPosition() >= softUpperLimit) ||
                motorPower < 0.0 && (lowerLimitSwitch != null && lowerLimitSwitch.isActive() ||
                        softLowerLimitEnabled && getPosition() <= softLowerLimit))
        {
            motorPower = 0.0;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", motorPower);
        }

        return motorPower;
    }   //constrainMotorPowerByLimitSwitches

    //
    // Implements TrcMotor abstract methods.
    //

    /**
     * This method returns the motor position by reading the position sensor. The position sensor can be an encoder
     * or a potentiometer.
     *
     * @return current motor position.
     */
    @Override
    public double getMotorPosition()
    {
        final String funcName = "getMotorPosition";
        double currPos = analogSensor == null?
                motor.getCurrentPosition(): analogSensor.getRawData(0, TrcAnalogInput.DataType.INPUT_DATA).value;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (analogSensor == null)
        {
            //
            // Somebody said if motor controller got disconnected, we may get a zero. Let's detect this and see if this
            // really happened.
            //
            if (currPos == 0.0 && Math.abs(prevEncPos) > 1000)
            {
                globalTracer.traceWarn(
                        funcName, "Detected possible motor controller disconnect for %s (prevEncPos=%d).",
                        instanceName, prevEncPos);
                currPos = prevEncPos;
            }
            else
            {
                prevEncPos = (int)currPos;
            }

            currPos -= zeroEncoderValue;
        }

        currPos *= positionSensorSign;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%d", currPos);
        }

        return currPos;
    }   //getMotorPosition

    /**
     * This method sets the raw motor power.
     *
     * @param power specifies the percentage power (range -1.0 to 1.0) to be set.
     */
    @Override
    public void setMotorPower(double power)
    {
        final String funcName = "setMotorPower";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "value=%f", power);
        }

        power = constrainMotorPowerByLimitSwitches(power);

        if (power != prevPower)
        {
            motor.setPower(power);
            prevPower = power;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "! (value=%f)", power);
        }
    }   //setMotorPower

    //
    // Implements TrcMotorController interface.
    //

    /**
     * This method returns the state of the motor controller direction.
     *
     * @return true if the motor direction is inverted, false otherwise.
     */
    @Override
    public boolean getInverted()
    {
        final String funcName = "getInverted";
        boolean inverted = motor.getDirection() == DcMotor.Direction.REVERSE;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", inverted);
        }

        return inverted;
    }   //getInverted

    /**
     * This method gets the last set power.
     *
     * @return the last setPower value.
     */
    @Override
    public double getPower()
    {
        final String funcName = "getPower";
        double power = prevPower;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%.3f", power);
        }

        return power;
    }   //getPower

    /**
     * This method returns the state of the lower limit switch.
     *
     * @return true if lower limit switch is active, false otherwise.
     */
    @Override
    public boolean isLowerLimitSwitchActive()
    {
        final String funcName = "isLowerLimitSwitchActive";
        boolean isActive = lowerLimitSwitch != null && lowerLimitSwitch.isActive();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Boolean.toString(isActive));
        }

        return isActive;
    }   //isLowerLimitSwitchActive

    /**
     * This method returns the state of the upper limit switch.
     *
     * @return true if upper limit switch is active, false otherwise.
     */
    @Override
    public boolean isUpperLimitSwitchActive()
    {
        final String funcName = "isUpperLimitSwitchActive";
        boolean isActive = upperLimitSwitch != null && upperLimitSwitch.isActive();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Boolean.toString(isActive));
        }

        return isActive;
    }   //isUpperLimitSwitchActive

    /**
     * This method resets the motor position sensor, typically an encoder.
     *
     * @param hardware specifies true for resetting hardware position, false for resetting software position.
     */
    @Override
    public void resetPosition(boolean hardware)
    {
        final String funcName = "resetPosition";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
        //
        // Modern Robotics motor controllers supports resetting encoders by setting the motor controller mode. This
        // is a long operation and has side effect of disabling the motor controller unless you do another setMode
        // to re-enable it. Therefore, resetPosition with hardware set to true is a synchronous call. This should
        // only be called at robotInit time. For other times, it should call resetPosition with hardware set to false
        // (software reset).
        //
        if (hardware)
        {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            while (motor.getCurrentPosition() != 0.0)
            {
                Thread.yield();
            }
            zeroEncoderValue = 0;
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        else
        {
            zeroEncoderValue = motor.getCurrentPosition();
        }

        prevEncPos = zeroEncoderValue;
    }   //resetPosition

    /**
     * This method inverts the position sensor direction. This may be rare but there are scenarios where the motor
     * encoder may be mounted somewhere in the power train that it rotates opposite to the motor rotation. This will
     * cause the encoder reading to go down when the motor is receiving positive power. This method can correct this
     * situation.
     *
     * @param inverted specifies true to invert position sensor direction, false otherwise.
     */
    @Override
    public void setPositionSensorInverted(boolean inverted)
    {
        final String funcName = "setPositionSensorInverted";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "inverted=%s", Boolean.toString(inverted));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        positionSensorSign = inverted? -1: 1;
    }   //setPositionSensorInverted

    /**
     * This method enables/disables motor brake mode. In motor brake mode, set power to 0 would stop the motor very
     * abruptly by shorting the motor wires together using the generated back EMF to stop the motor. When brakMode
     * is false (i.e. float/coast mode), the motor wires are just disconnected from the motor controller so the motor
     * will stop gradually.
     *
     * @param enabled specifies true to enable brake mode, false otherwise.
     */
    @Override
    public void setBrakeModeEnabled(boolean enabled)
    {
        final String funcName = "setBrakeModeEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enabled=%s", Boolean.toString(enabled));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        motor.setZeroPowerBehavior(enabled? DcMotor.ZeroPowerBehavior.BRAKE: DcMotor.ZeroPowerBehavior.FLOAT);
    }   //setBrakeModeEnabled

    /**
     * This method inverts the motor direction.
     *
     * @param inverted specifies true to invert motor direction, false otherwise.
     */
    @Override
    public void setInverted(boolean inverted)
    {
        final String funcName = "setInverted";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "inverted=%s", Boolean.toString(inverted));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        motor.setDirection(inverted? DcMotor.Direction.REVERSE: DcMotor.Direction.FORWARD);
    }   //setInverted

    /**
     * This method enables/disables soft limit switches.
     *
     * @param lowerLimitEnabled specifies true to enable lower soft limit switch, false otherwise.
     * @param upperLimitEnabled specifies true to enable upper soft limit switch, false otherwise.
     */
    public synchronized void setSoftLimitEnabled(boolean lowerLimitEnabled, boolean upperLimitEnabled)
    {
        final String funcName = "setSoftLimitEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "lowerEnabled=%s,upperEnabled=%s",
                                Boolean.toString(lowerLimitEnabled), Boolean.toString(upperLimitEnabled));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        softLowerLimitEnabled = lowerLimitEnabled;
        softUpperLimitEnabled = upperLimitEnabled;
    }   //setSoftLimitEnabled

    /**
     * This method sets the lower soft limit.
     *
     * @param position specifies the position of the lower limit.
     */
    public synchronized void setSoftLowerLimit(double position)
    {
        final String funcName = "setSoftLowerLimit";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "position=%f", position);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        softLowerLimit = position;
    }   //setSoftLowerLimit

    /**
     * This method sets the upper soft limit.
     *
     * @param position specifies the position of the upper limit.
     */
    public synchronized void setSoftUpperLimit(double position)
    {
        final String funcName = "setSoftUpperLimit";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "position=%f", position);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        softUpperLimit = position;
    }   //setSoftUpperLimit

}   //class FtcDcMotor
