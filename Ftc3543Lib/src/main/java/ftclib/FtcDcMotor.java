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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import hallib.HalMotorController;
import hallib.HalUtil;
import trclib.TrcDigitalInput;
import trclib.TrcDbgTrace;
import trclib.TrcRobot;
import trclib.TrcTaskMgr;

/**
 * This class implements the Modern Robotics Motor Controller extending
 * TrcMotorController. It provides implementation of the abstract methods
 * in TrcMotorController. It supports limit switches. When this class is
 * constructed with limit switches, setPower will respect them and will
 * not move the motor into the direction where the limit switch is activated.
 * It also provides a software encoder reset without switching the Modern
 * Robotics motor controller mode which is problematic.
 */
public class FtcDcMotor implements HalMotorController, TrcTaskMgr.Task
{
    private static final String moduleName = "FtcDcMotor";
    private static final boolean debugEnabled = false;
    private TrcDbgTrace dbgTrace = null;

    private String instanceName;
    private TrcDigitalInput lowerLimitSwitch = null;
    private TrcDigitalInput upperLimitSwitch = null;
    public DcMotor motor;
    private int zeroEncoderValue;
    private int positionSensorSign = 1;
    private boolean speedTaskEnabled = false;
    private double speed = 0.0;
    private double prevTime = 0.0;
    private double prevPos = 0.0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     * @param lowerLimitSwitch specifies the lower limit switch object.
     * @param upperLimitSwitch specifies the upper limit switch object.
     */
    public FtcDcMotor(
            HardwareMap hardwareMap,
            String instanceName,
            TrcDigitalInput lowerLimitSwitch,
            TrcDigitalInput upperLimitSwitch)
    {
        this.instanceName = instanceName;

        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(
                    moduleName + "." + instanceName,
                    false,
                    TrcDbgTrace.TraceLevel.API,
                    TrcDbgTrace.MsgLevel.INFO);
        }

        this.lowerLimitSwitch = lowerLimitSwitch;
        this.upperLimitSwitch = upperLimitSwitch;
        motor = hardwareMap.dcMotor.get(instanceName);
        zeroEncoderValue = motor.getCurrentPosition();
    }   //FtcDcMotor

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param lowerLimitSwitch specifies the lower limit switch object.
     * @param upperLimitSwitch specifies the upper limit switch object.
     */
    public FtcDcMotor(
            String instanceName,
            TrcDigitalInput lowerLimitSwitch,
            TrcDigitalInput upperLimitSwitch)
    {
        this(FtcOpMode.getInstance().hardwareMap,
             instanceName,
             lowerLimitSwitch,
             upperLimitSwitch);
    }   //FtcDcMotor

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param lowerLimitSwitch specifies the lower limit switch object.
     */
    public FtcDcMotor(
            String instanceName,
            TrcDigitalInput lowerLimitSwitch)
    {
        this(instanceName, lowerLimitSwitch, null);
    }   //FtcDcMotor

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FtcDcMotor(String instanceName)
    {
        this(instanceName, null, null);
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

    //
    // Implements HalMotorController interface.
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
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=%s", Boolean.toString(inverted));
        }

        return inverted;
    }   //getInverted

    /**
     * This method returns the motor position by reading the position sensor. The position
     * sensor can be an encoder or a potentiometer.
     *
     * @return current motor position.
     */
    @Override
    public double getPosition()
    {
        final String funcName = "getPosition";
        int position = positionSensorSign*(motor.getCurrentPosition() - zeroEncoderValue);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%d", position);
        }

        return (double)position;
    }   //getPosition

    /**
     * This method gets the last set power.
     *
     * @return the last setPower value.
     */
    @Override
    public double getPower()
    {
        final String funcName = "getPower";
        double power = motor.getPower();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%.3f", power);
        }

        return power;
    }   //getPower

    /**
     * This method returns the speed of the motor rotation which is not
     * supported by the Modern Robotics motor controller.
     *
     * @throws UnsupportedOperationException exception.
     */
    @Override
    public double getSpeed()
    {
        final String funcName = "getSpeed";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%.3f", speed);
        }

        if (speedTaskEnabled)
        {
            return speed;
        }
        else
        {
            throw new UnsupportedOperationException("SpeedTask is not enabled.");
        }
    }   //getSpeed

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
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=%s", Boolean.toString(isActive));
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
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=%s", Boolean.toString(isActive));
        }

        return isActive;
    }   //isUpperLimitSwitchActive

    /**
     * This method resets the motor position sensor, typically an encoder.
     */
    @Override
    public void resetPosition()
    {
        final String funcName = "resetPosition";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        //
        // Modern Robotics motor controllers supports resetting encoders
        // by setting the motor controller mode. This is a long operation
        // and has side effect of disabling the motor controller unless
        // you do another setMode to re-enable it. For example:
        //      motor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        //      motor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        // It is a lot more efficient doing it in software.
        //
        zeroEncoderValue = motor.getCurrentPosition();
    }   //resetPosition

    /**
     * This method enables/disables motor brake mode. In motor brake mode, set power to 0 would
     * stop the motor very abruptly by shorting the motor wires together using the generated
     * back EMF to stop the motor. When brakMode is false (i.e. float/coast mode), the motor wires
     * are just disconnected from the motor controller so the motor will stop gradually.
     *
     * @param enabled specifies true to enable brake mode, false otherwise.
     */
    @Override
    public void setBrakeModeEnabled(boolean enabled)
    {
        final String funcName = "setBrakeModeEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                                "enabled=%s", Boolean.toString(enabled));
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
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                                "inverted=%s", Boolean.toString(inverted));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        motor.setDirection(inverted? DcMotor.Direction.REVERSE: DcMotor.Direction.FORWARD);
    }   //setInverted

    /**
     * This method sets the output power of the motor controller.
     *
     * @param power specifies the output power for the motor controller in the range of
     *              -1.0 to 1.0.
     */
    @Override
    public void setPower(double power)
    {
        final String funcName = "setPower";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "power=%f", power);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        //
        // If we have limit switches, respect them.
        //
        if (power > 0.0 && upperLimitSwitch != null && upperLimitSwitch.isActive() ||
            power < 0.0 && lowerLimitSwitch != null && lowerLimitSwitch.isActive())
        {
            power = 0.0;
        }

        motor.setPower(power);
    }   //setPower

    /**
     * This method inverts the position sensor direction. This may be rare but
     * there are scenarios where the motor encoder may be mounted somewhere in
     * the power train that it rotates opposite to the motor rotation. This will
     * cause the encoder reading to go down when the motor is receiving positive
     * power. This method can correct this situation.
     *
     * @param inverted specifies true to invert position sensor direction,
     *                 false otherwise.
     */
    @Override
    public void setPositionSensorInverted(boolean inverted)
    {
        final String funcName = "setPositionSensorInverted";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                                "inverted=%s", Boolean.toString(inverted));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        positionSensorSign = inverted? -1: 1;
    }   //setPositionSensorInverted

    /**
     * This method enables/disables the task that monitors the motor speed. To determine the motor speed,
     * the task runs periodically and determines the delta encoder reading over delta time to calculate
     * the speed. Since the task takes up CPU cycle, it should not be enabled if the user doesn't need
     * motor speed info.
     *
     * @param enabled specifies true to enable speed monitor task, disable otherwise.
     */
    public void setSpeedTaskEnabled(boolean enabled)
    {
        final String funcName = "setSpeedTaskEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enabled=%s", Boolean.toString(enabled));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        speedTaskEnabled = enabled;
        TrcTaskMgr taskMgr = TrcTaskMgr.getInstance();
        if (enabled)
        {
            prevTime = HalUtil.getCurrentTime();
            prevPos = getPosition();
            taskMgr.registerTask(
                    instanceName,
                    this,
                    TrcTaskMgr.TaskType.STOP_TASK);
            taskMgr.registerTask(
                    instanceName,
                    this,
                    TrcTaskMgr.TaskType.PRECONTINUOUS_TASK);
        }
        else
        {
            taskMgr.unregisterTask(
                    this,
                    TrcTaskMgr.TaskType.STOP_TASK);
            taskMgr.unregisterTask(
                    this,
                    TrcTaskMgr.TaskType.PRECONTINUOUS_TASK);
        }
    }   //setSpeedEnabled

    //
    // Implements TrcTaskMgr.Task
    //

    @Override
    public void startTask(TrcRobot.RunMode runMode)
    {
    }   //startTask

    @Override
    public void stopTask(TrcRobot.RunMode runMode)
    {
        final String funcName = "stopTask";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.TASK,
                    "mode=%s", runMode.toString());
        }

        setSpeedTaskEnabled(false);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //stopTask

    @Override
    public void prePeriodicTask(TrcRobot.RunMode runMode)
    {
    }   //prePeriodicTask

    @Override
    public void postPeriodicTask(TrcRobot.RunMode runMode)
    {
    }   //postPeriodicTask

    /**
     * This task is run periodically to calculate he speed of the motor.
     *
     * @param runMode specifies the competition mode that is running.
     */
    @Override
    public void preContinuousTask(TrcRobot.RunMode runMode)
    {
        final String funcName = "preContinuousTask";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.TASK,
                    "mode=%s", runMode.toString());
        }

        double currTime = HalUtil.getCurrentTime();
        double currPos = getPosition();
        if (prevTime != 0.0)
        {
            speed = (currPos - prevPos)/(currTime - prevTime);
        }
        prevTime = currTime;
        prevPos = currPos;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //preContinuousTask

    @Override
    public void postContinuousTask(TrcRobot.RunMode runMode)
    {
    }   //postContinuousTask

}   //class FtcDcMotor
