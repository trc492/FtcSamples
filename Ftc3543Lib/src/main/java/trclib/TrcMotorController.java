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
 * This interface should be implemented by any motor controllers that want to be compatible with the TRC framework.
 * The methods in this interface allow the framework to access any compatible motor controllers independent of the
 * platform. Not all motor controllers are created equal. Some have more features than the others. For some methods,
 * the motor controllers may just past the requests to the underlying hardware. For other methods, it may need to
 * simulate the features in software. Yet for some other methods, it may just throw an UnsupportedOperationException.
 */
public interface TrcMotorController
{
    /**
     * This method returns the state of the motor controller direction.
     *
     * @return true if the motor direction is inverted, false otherwise.
     */
    boolean getInverted();

    /**
     * This method returns the motor position by reading the position sensor. The position sensor can be an encoder
     * or a potentiometer.
     *
     * @return current motor position.
     */
    double getPosition();

    /**
     * This method gets the last set power.
     *
     * @return the last setPower value.
     */
    double getPower();

    /**
     * This method returns the velocity of the motor rotation in sensor unit per second.
     *
     * @return motor rotation velocity in sensor unit per second.
     */
    double getVelocity();

    /**
     * This method returns the state of the lower limit switch.
     *
     * @return true if lower limit switch is active, false otherwise.
     */
    boolean isLowerLimitSwitchActive();

    /**
     * This method returns the state of the upper limit switch.
     *
     * @return true if upper limit switch is active, false otherwise.
     */
    boolean isUpperLimitSwitchActive();

    /**
     * This method resets the motor position sensor, typically an encoder.
     *
     * @param hardware specifies true for resetting hardware position, false for resetting software position.
     */
    void resetPosition(boolean hardware);

    /**
     * This method sets the motor output value. The value can be power or velocity percentage depending on whether
     * the motor controller is in power mode or velocity mode.
     *
     * @param value specifies the percentage power or velocity (range -1.0 to 1.0) to be set.
     */
    void set(double value);

    /**
     * This method enables/disables motor brake mode. In motor brake mode, set power to 0 would stop the motor very
     * abruptly by shorting the motor wires together using the generated back EMF to stop the motor. When brakMode
     * is false (i.e. float/coast mode), the motor wires are just disconnected from the motor controller so the motor
     * will stop gradually.
     *
     * @param enabled specifies true to enable brake mode, false otherwise.
     */
    void setBrakeModeEnabled(boolean enabled);

    /**
     * This method inverts the motor direction.
     *
     * @param inverted specifies true to invert motor direction, false otherwise.
     */
    void setInverted(boolean inverted);

    /**
     * This method inverts the position sensor direction. This may be rare but there are scenarios where the motor
     * encoder may be mounted somewhere in the power train that it rotates opposite to the motor rotation. This will
     * cause the encoder reading to go down when the motor is receiving positive power. This method can correct this
     * situation.
     *
     * @param inverted specifies true to invert position sensor direction, false otherwise.
     */
    void setPositionSensorInverted(boolean inverted);

    /**
     * This method enables/disables soft limit switches.
     *
     * @param lowerLimitEnabled specifies true to enable lower soft limit switch, false otherwise.
     * @param upperLimitEnabled specifies true to enable upper soft limit switch, false otherwise.
     */
    void setSoftLimitEnabled(boolean lowerLimitEnabled, boolean upperLimitEnabled);

    /**
     * This method sets the lower soft limit.
     *
     * @param position specifies the position of the lower limit.
     */
    void setSoftLowerLimit(double position);

    /**
     * This method sets the upper soft limit.
     *
     * @param position specifies the position of the upper limit.
     */
    void setSoftUpperLimit(double position);

    /**
     * This method checks if the motor controller is connected to the robot. Note that this does NOT guarantee the
     * connection status of the motor to the motor controller. If detecting the motor presence is impossible (i.e. the
     * motor controller is connected via PWM) this method will always return true.
     *
     * @return True if the motor is connected or if it's impossible to know, false otherwise.
     */
    default boolean isConnected()
    {
        return true;
    }   //isConnected

}   //interface TrcMotorController
