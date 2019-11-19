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

package samples;

import ftclib.FtcDcMotor;
import ftclib.FtcDigitalInput;
import trclib.TrcEvent;
import trclib.TrcPidActuator;
import trclib.TrcPidController;

public class Elevator
{
    //
    // Elevator constants.
    //
    private static final double ELEVATOR_KP                     = 0.5;
    private static final double ELEVATOR_KI                     = 0.0;
    private static final double ELEVATOR_KD                     = 0.0;
    private static final double ELEVATOR_TOLERANCE              = 0.2;
    private static final double ELEVATOR_MIN_HEIGHT             = 0.0;
    private static final double ELEVATOR_MAX_HEIGHT             = 23.5;
    private static final double ELEVATOR_INCHES_PER_COUNT       = (23.5/9700.0);
    private static final double ELEVATOR_CAL_POWER              = 0.3;

    //
    // This subsystem consists of an elevator motor, a lower limit switch, and a PID controller to control the
    // movement of the elevator.
    //

    private FtcDigitalInput lowerLimitSwitch, upperLimitSwitch;
    private FtcDcMotor motor;
    private TrcPidController pidCtrl;
    private TrcPidActuator pidElevator;

    /**
     * Constructor: Create an instance of the object. This elevator consists of an upper and lower limit switches
     * and a DC motor with encoder.
     */
    public Elevator()
    {
        lowerLimitSwitch = new FtcDigitalInput("elevatorLowerLimitSwitch");
        upperLimitSwitch = new FtcDigitalInput("elevatorUpperLimitSwitch");
        motor = new FtcDcMotor("elevatorMotor", lowerLimitSwitch, upperLimitSwitch);
        motor.setBrakeModeEnabled(true);
        motor.setOdometryEnabled(true);

        pidCtrl = new TrcPidController(
                "elevatorPidCtrl",
                new TrcPidController.PidCoefficients(ELEVATOR_KP, ELEVATOR_KI, ELEVATOR_KD),
                ELEVATOR_TOLERANCE, this::getHeight);

        pidElevator = new TrcPidActuator(
                "pidElevator", motor, lowerLimitSwitch, pidCtrl, ELEVATOR_CAL_POWER,
                ELEVATOR_MIN_HEIGHT, ELEVATOR_MAX_HEIGHT);
        pidElevator.setPositionScale(ELEVATOR_INCHES_PER_COUNT);
    }   //Elevator

    /**
     * This method starts zero calibration of the elevator. It causes the elevator to go down slowly until the lower
     * limit switch is activated and zeros the encoder to establish the elevator's zero position.
     */
    public void zeroCalibrate()
    {
        pidElevator.zeroCalibrate();
    }   //zeroCalibrate

    /**
     * This method enables/disables manual override. Enabling manual override allows the user to bypass PID control.
     * Since PID control monitors the lower and upper heights of the elevator, it will not allow the elevator to go
     * beyond these soft limits. If for some reason the elevator is not zero calibrated properly, it may refuse to
     * move thinking one of the soft limits has been reached. If that's the case, manual override allows the operator
     * to bypass PID and ignore the soft limits. However, the elevator will always respect the limit switch hard
     * limits. If one of the limit switches is activated, the elevator will not move in that direction.
     *
     * @param enabled specifies true to enable manual override, false to disable.
     */
    public void setManualOverride(boolean enabled)
    {
        pidElevator.setManualOverride(enabled);
    }   //setManualOverride

    /**
     * This method controls how fast the elevator will move. However, if the elevator is approaching the lower or
     * upper limits, it will slow down by using PID control. If manual override is enabled, PID control is bypass
     * and the specified power will be applied to the elevator motor unmodified.
     *
     * @param power specifies the power to apply to the elevator motor.
     */
    public void setPower(double power)
    {
        pidElevator.setPower(power, true);
    }   //setPower

    /**
     * This method sets the elevator to the given height using PID control. The given event will be signaled when the
     * specified height is reached. An optional timeout value can be specified in case PID target takes forever to be
     * reached.
     *
     * @param height specifies the height to set the elevator to.
     * @param event specifies the event to be signaled when the specified height is reached.
     * @param timeout specifies timeout value in seconds, can be zero if no timeout.
     */
    public void setHeight(double height, TrcEvent event, double timeout)
    {
        pidElevator.setTarget(height, event, timeout);
    }   //setPosition

    /**
     * This method sets the elevator to the given height using PID control and holds the position at that height.
     *
     * @param height specifies the height to set the elevator to.
     */
    public void setHeight(double height)
    {
        pidElevator.setTarget(height, true);
    }   //setHeight

    /**
     * This method is called by the PID controller to get the current height of the elevator.
     *
     * @return current elevator height.
     */
    public double getHeight()
    {
        return pidElevator.getPosition();
    }   //getHeight

    /**
     * This method checks if the lower limit switch is active.
     *
     * @return true if the limit switch is pressed.
     */
    public boolean isLowerLimitSwitchActive()
    {
        return lowerLimitSwitch.isActive();
    }   //isLowerLimitSwitchActive

    /**
     * This method checks if the upper limit switch is active.
     *
     * @return true if the limit switch is pressed.
     */
    public boolean isUpperLimitSwitchActive()
    {
        return upperLimitSwitch.isActive();
    }   //isUpperLimitSwitchActive

}   //class Elevator
