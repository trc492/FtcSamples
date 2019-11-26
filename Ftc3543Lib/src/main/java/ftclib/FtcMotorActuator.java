/*
 * Copyright (c) 2019 Titan Robotics Club (http://www.titanrobotics.com)
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

import trclib.TrcEvent;
import trclib.TrcPidActuator;
import trclib.TrcPidController;

/**
 * This class implements a platform dependent motor actuator. A motor actuator consists of a DC motor, a lower limit
 * switch and an optional upper limit switch. It creates all the necessary components for a PID controlled actuator
 * which includes a PID controller and a PID controlled actuator.
 */
public class FtcMotorActuator
{
    /**
     * This class contains all the parameters related to the motor actuator.
     */
    public static class Parameters
    {
        double minPos, maxPos;
        double scale, offset;
        double kP, kI, kD, tolerance;
        boolean inverted;
        boolean hasUpperLimitSwitch;
        double calPower;
        double[] posPresets;

        /**
         * This method sets the position range limits of the motor actuator.
         *
         * @param minPos specifies the minimum position of the actuator in scaled unit.
         * @param maxPos specifies the maximum position of the actuator in scaled unit.
         * @return this parameter object.
         */
        public Parameters setPosRange(double minPos, double maxPos)
        {
            this.minPos = minPos;
            this.maxPos = maxPos;
            return this;
        }   //setPosRange

        /**
         * This method sets the scale and offset of the motor actuator. It allows the actuator to report real world
         * position units such as inches or degrees instead of sensor units.
         *
         * @param scale specifies the scale multiplier to convert position sensor unit to real world unit.
         * @param offset specifies the offset value to add to the scaled real world unit.
         * @return this parameter object.
         */
        public Parameters setScaleOffset(double scale, double offset)
        {
            this.scale = scale;
            this.offset = offset;
            return this;
        }   //setScaleOffset

        /**
         * This method sets the PID parameters of the PID controller used for PID controlling the motor actuator.
         *
         * @param kP specifies the Proportional constant.
         * @param kI specifies the Integral constant.
         * @param kD specifies the Differential constant.
         * @param tolerance specifies the tolerance in scaled unit.
         * @return this parameter object.
         */
        public Parameters setPidParams(double kP, double kI, double kD, double tolerance)
        {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.tolerance = tolerance;
            return this;
        }   //setPidParams

        /**
         * This method sets the motor parameters of the motor actuator.
         *
         * @param inverted specifies true if the motor direction should be reverse, false otherwise.
         * @param hasUpperLimitSwitch specifies true if it has an upper limit switch, false otherwise.
         * @param calPower specifies the motor power to use for zero calibration.
         * @return this parameter object.
         */
        public Parameters setMotorParams(boolean inverted, boolean hasUpperLimitSwitch, double calPower)
        {
            this.inverted = inverted;
            this.hasUpperLimitSwitch = hasUpperLimitSwitch;
            this.calPower = calPower;
            return this;
        }   //setMotorParams

        /**
         * This method sets an array of preset positions for the motor actuator.
         *
         * @param posPresets specifies an array of preset positions in scaled unit.
         * @return this parameter object.
         */
        public Parameters setPosPresets(double... posPresets)
        {
            this.posPresets = posPresets;
            return this;
        }   //setPosPresets

    }   //class Parameters

    private FtcDigitalInput lowerLimitSwitch;
    private FtcDigitalInput upperLimitSwitch;
    private TrcPidActuator pidActuator;

    private double[] posPresets;
    private int posLevel;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param params specifies the parameters to set up the motor actuator.
     */
    public FtcMotorActuator(String instanceName, Parameters params)
    {
        lowerLimitSwitch = new FtcDigitalInput(instanceName + "LowerLimit");
        if (params.hasUpperLimitSwitch)
        {
            upperLimitSwitch = new FtcDigitalInput(instanceName + "UpperLimit");
        }

        FtcDcMotor actuatorMotor = new FtcDcMotor(
                instanceName + "Motor", lowerLimitSwitch, upperLimitSwitch);
        actuatorMotor.setBrakeModeEnabled(true);
        actuatorMotor.setOdometryEnabled(true);
        actuatorMotor.setInverted(params.inverted);

        TrcPidController pidController = new TrcPidController(
                instanceName + "PidController",
                new TrcPidController.PidCoefficients(params.kP, params.kI, params.kD),
                params.tolerance, this::getPosition);

        pidActuator = new TrcPidActuator(
                "pid" + instanceName, actuatorMotor, lowerLimitSwitch, pidController,
                params.calPower, params.minPos, params.maxPos);
        pidActuator.setPositionScale(params.scale, params.offset);

        this.posPresets = params.posPresets;
        this.posLevel = 0;
    }   //FtcMotorActuator

    /**
     * This method starts the zero calibration process by operating the motor with calibration power moving it towards
     * the lower limit switch. When the lower limit switch is activated, the position sensor is reset and the motor
     * stops.
     */
    public void zeroCalibrate()
    {
        pidActuator.zeroCalibrate();
    }   //zeroCalibrate

    /**
     * This method enables/disables manual override on the motor actuator. In normal mode, the movement of the
     * actuator is PID controlled so it can slow down when it is close to the lower or upper position limits. It
     * also enables soft upper limit if there is no upper limit switch. In manual override mode, the actuator is
     * no longer PID controlled so the speed of the actuator movement is completely controlled by the caller. However,
     * manual override mode will still respect the physical limit switches and will not allow motor movement to pass
     * the limit switches. This means the actuator can bump into the limit switches full speed in manual override
     * mode. Therefore, it is the operator's responsibility to pay attention to the actuator position. Manual override
     * mode is very useful when the actuator is not properly zero calibrated. For example, if the actuator is in some
     * mid position when the robot is started, the robot will think that position is the zero position and will not
     * allow the actuator to go below it. In this situation, you either have to start a zero calibration or use
     * manual override mode to force the actuator to move towards the lower limit switch. Manual override also allows
     * the actuator to go pass the upper soft limit. This is useful if the robot thinks the actuator is already at
     * the upper limit but physically it can still move a little bit more.
     *
     * @param enabled specifies true for enabling manual override mode, false to disable.
     */
    public void setManualOverride(boolean enabled)
    {
        pidActuator.setManualOverride(enabled);
    }   //setManualOverride

    /**
     * This method moves the actuator at the given power level. However, if the actuator is close to the limits and
     * manual override mode is not active, the power will be reduced by the PID controller.
     *
     * @param power specifies the power level to move the actuator.
     */
    public void setPower(double power)
    {
        pidActuator.setPower(power, true);
    }   //setPower

    /**
     * This method starts moving the actuator to the specified position.
     *
     * @param target specifies the target position to set the actuator.
     * @param event specifies the event to be signal when the actuator reaches the specified position.
     * @param timeout specifies the maximum timeout for the actuator movement. If the actuator does not reach target
     *                within timeout time, the event will be signaled so autonomous state machine will not be hung
     *                if for some reason the actuator never reaches target.
     */
    public void setPosition(double target, TrcEvent event, double timeout)
    {
        pidActuator.setTarget(target, event, timeout);
    }   //setPosition

    /**
     * This method starts moving the actuator to the specified position. The actuator will maintain the target position
     * after target is reached.
     *
     * @param target specifies the target position to set the actuator.
     */
    public void setPosition(double target)
    {
        pidActuator.setTarget(target, true);
    }   //setPosition

    /**
     * This method sets the actuator to the specified preset position.
     *
     * @param level specifies the index to the preset position array.
     */
    public void setLevel(int level)
    {
        if (posPresets != null)
        {
            if (level < 0)
            {
                posLevel = 0;
            }
            else if (level >= posPresets.length)
            {
                posLevel = posPresets.length - 1;
            }
            else
            {
                posLevel = level;
            }

            setPosition(posPresets[posLevel]);
        }
    }   //setLevel

    /**
     * This method sets the actuator to the next preset position up.
     */
    public void levelUp()
    {
        setLevel(posLevel + 1);
    }   //levelUp

    /**
     * This method sets the actuator to the next preset position down.
     */
    public void levelDown()
    {
        setLevel(posLevel - 1);
    }   //levelDown

    /**
     * This method returns the last preset level that is set to the actuator.
     *
     * @return last preset level set.
     */
    public int getLevel()
    {
        return posLevel;
    }   //getLevel

    /**
     * This method returns the current actuator position in scaled unit.
     *
     * @return current actuator position in scaled unit.
     */
    public double getPosition()
    {
        return pidActuator.getPosition();
    }   //getPosition

    /**
     * This method checks if the lower limit switch is activated.
     *
     * @return true if the lower limit switch is activated, false otherwise.
     */
    public boolean isLowerLimitSwitchActive()
    {
        return lowerLimitSwitch.isActive();
    }   //isLowerLimitSwitchActive

    /**
     * This method checks if the upper limit switch is activated.
     *
     * @return true if the upper limit switch is activated, false otherwise.
     */
    public boolean isUpperLimitSwitchActive()
    {
        return upperLimitSwitch != null ? upperLimitSwitch.isActive() : false;
    }   //isUpperLimitSwitchActive

}   //class FtcMotorActuator

