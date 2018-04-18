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

    //
    // This subsystem consists of an elevator motor, a lower limit switch, and a PID controller to control the
    // movement of the elevator.
    //

    private FtcTouchSensor lowerLimitSwitch;
    private FtcDcMotor motor;
    private TrcPidController pidCtrl;
    public TrcPidActuator actuator;

    /**
     * Constructor: Create an instance of the object.
     */
    public Elevator()
    {
        lowerLimitSwitch = new FtcTouchSensor("elevatorLowerLimitSwitch");
        motor = new FtcDcMotor("elevatorMotor", lowerLimitSwitch);
        pidCtrl = new TrcPidController(
                "elevatorPidCtrl",
                new TrcPidController.PidCoefficients(ELEVATOR_KP, ELEVATOR_KI, ELEVATOR_KD),
                ELEVATOR_TOLERANCE, this::getPosition);
        actuator = new TrcPidActuator(
                "elevator", motor, lowerLimitSwitch, pidCtrl, ELEVATOR_MIN_HEIGHT, ELEVATOR_MAX_HEIGHT);
        actuator.setPositionScale(ELEVATOR_INCHES_PER_COUNT);
    }   //Elevator

    /**
     * This method is called by the PID controller to get the current height of the elevator.
     *
     * @return current elevator height.
     */
    public double getPosition()
    {
        return actuator.getPosition();
    }   //getPosition

}   //class Elevator
