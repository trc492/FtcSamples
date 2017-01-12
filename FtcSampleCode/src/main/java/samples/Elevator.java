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
import ftclib.FtcTouchSensor;
import trclib.TrcDigitalTrigger;
import trclib.TrcEvent;
import trclib.TrcPidController;
import trclib.TrcPidMotor;

public class Elevator implements TrcPidController.PidInput, TrcDigitalTrigger.TriggerHandler
{
    //
    // Elevator constants.
    //
    private static final double ELEVATOR_KP                     = 0.5;
    private static final double ELEVATOR_KI                     = 0.0;
    private static final double ELEVATOR_KD                     = 0.0;
    private static final double ELEVATOR_KF                     = 0.0;
    private static final double ELEVATOR_TOLERANCE              = 0.2;
    private static final double ELEVATOR_SETTLING               = 0.2;
    private static final double ELEVATOR_MIN_HEIGHT             = 0.0;
    private static final double ELEVATOR_MAX_HEIGHT             = 23.5;
    private static final double ELEVATOR_INCHES_PER_COUNT       = (23.5/9700.0);
    private static final double ELEVATOR_CAL_POWER              = 0.3;

    //
    // This subsystem consists of an elevator motor, a lower limit switch, and an encoder to keep track of the
    // position of the elevator.
    //

    private FtcTouchSensor lowerLimitSwitch;
    private TrcDigitalTrigger lowerLimitTrigger;
    private FtcDcMotor motor;
    private TrcPidController pidCtrl;
    private TrcPidMotor pidMotor;
    private boolean manualOverride = false;

    /**
     * Constructor: Create an instance of the object.
     */
    public Elevator()
    {
        lowerLimitSwitch = new FtcTouchSensor("lowerLimitSwitch");
        lowerLimitTrigger = new TrcDigitalTrigger("elevatorLowerLimit", lowerLimitSwitch, this);
        lowerLimitTrigger.setEnabled(true);
        motor = new FtcDcMotor("elevator", lowerLimitSwitch);
        pidCtrl = new TrcPidController(
                "elevator",
                ELEVATOR_KP, ELEVATOR_KI, ELEVATOR_KD, ELEVATOR_KF,
                ELEVATOR_TOLERANCE, ELEVATOR_SETTLING,
                this);
        pidCtrl.setAbsoluteSetPoint(true);
        pidMotor = new TrcPidMotor("elevator", motor, pidCtrl);
        pidMotor.setPositionScale(ELEVATOR_INCHES_PER_COUNT);
    }   //Elevator

    /**
     * This method does a zero calibration on the elevator by slowly lowering it until it hits the lower limit switch.
     * Then it stops and resets the motor encoder. This is typically called in the robot initialization or before
     * competition start.
     */
    public void zeroCalibrate()
    {
        pidMotor.zeroCalibrate(ELEVATOR_CAL_POWER);
    }   //zeroCalibrate

    /**
     * This method sets manual override mode. This is useful to override PID control of the elevator in situations
     * where the encoder is not zero calibrated or malfunctioning. Note that this only overrides the encoder but not
     * the limit switch. So if the lower limit switch is engaged, the elevator will not go down even manual override
     * is true.
     *
     * @param manualOverride specifies true for manual override, false otherwise.
     */
    public void setManualOverride(boolean manualOverride)
    {
        this.manualOverride = manualOverride;
    }   //setManualOverride

    /**
     * This method runs the elevator with the specified power. It will hold the current position even if power is zero.
     *
     * @param power specifies the power to run the elevator.
     */
    public void setPower(double power)
    {
        if (manualOverride)
        {
            motor.setPower(power);
        }
        else
        {
            pidMotor.setSpeed(power, ELEVATOR_MIN_HEIGHT, ELEVATOR_MAX_HEIGHT, true);
        }
    }   //setPower

    /**
     * This method runs the elevator to the specified height and hold it there.
     *
     * @param height specifies the target height.
     */
    public void setHeight(double height)
    {
        pidMotor.setTarget(height, true);
    }   //setHeight

    /**
     * This method runs the elevator to the specified height and signal the given event when done. Optionally, if a
     * timeout is specified, the operation will be aborted when specified time has expired. Note that this method
     * does not hold the elevator in place when reaching target.
     *
     * @param height specifies the target height.
     * @param event specifies the event to signal when done.
     * @param timeout specifies timeout for the operation. It can be zero, if no timeout is used.
     */
    public void setHeight(double height, TrcEvent event, double timeout)
    {
        pidMotor.setTarget(height, event, timeout);
    }   //setHeight

    /**
     * This method returns the current height of the elevator.
     *
     * @return current height of elevator.
     */
    public double getHeight()
    {
        return pidMotor.getPosition();
    }   //getHeight

    /**
     * This method returns the state of the lower limit switch.
     *
     * @return true if lower limit switch is engaged, false otherwise.
     */
    public boolean isLowerLimitSwitchPressed()
    {
        return lowerLimitSwitch.isActive();
    }   //isLowerLimitSwitchPressed

    /**
     * This method displays the elevator PID information to the dashboard on the given line number. Note that the
     * information occupies two dashboard lines.
     *
     * @param lineNum specifies the dashboard line number to display the first line.
     */
    public void displayDebugInfo(int lineNum)
    {
        pidCtrl.displayPidInfo(lineNum);
    }   //displayPidInfo

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
            value = getHeight();
        }

        return value;
    }   //getInput

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
    public void DigitalTriggerEvent(TrcDigitalTrigger digitalTrigger, boolean active)
    {
        if (digitalTrigger == lowerLimitTrigger)
        {
            motor.resetPosition();
        }
    }   //DigitalTriggerEvent

}   //class Elevator
