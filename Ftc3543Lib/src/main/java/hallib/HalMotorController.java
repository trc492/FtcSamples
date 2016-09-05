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

package hallib;

public interface HalMotorController
{
    /**
     * This method returns the state of the motor controller direction.
     *
     * @return true if the motor direction is inverted, false otherwise.
     */
    public boolean getInverted();

    /**
     * This method returns the motor position by reading the position sensor. The position
     * sensor can be an encoder or a potentiometer.
     *
     * @return current motor position.
     */
    public double getPosition();

    /**
     * This method returns the speed of the motor rotation.
     *
     * @return motor rotation speed.
     */
    public double getSpeed();

    /**
     * This method returns the state of the lower limit switch.
     *
     * @return true if lower limit switch is active, false otherwise.
     */
    public boolean isLowerLimitSwitchActive();

    /**
     * This method returns the state of the upper limit switch.
     *
     * @return true if upper limit switch is active, false otherwise.
     */
    public boolean isUpperLimitSwitchActive();

    /**
     * This method resets the motor position sensor, typically an encoder.
     */
    public void resetPosition();

    /**
     * This method enables/disables motor brake mode. In motor brake mode, set power to 0 would
     * stop the motor very abruptly by shorting the motor wires together using the generated
     * back EMF to stop the motor. When brakMode is false (i.e. float/coast mode), the motor wires
     * are just disconnected from the motor controller so the motor will stop gradually.
     *
     * @param enabled specifies true to enable brake mode, false otherwise.
     */
    public void setBrakeModeEnabled(boolean enabled);

    /**
     * This method inverts the motor direction.
     *
     * @param inverted specifies true to invert motor direction, false otherwise.
     */
    public void setInverted(boolean inverted);

    /**
     * This method sets the output power of the motor controller.
     *
     * @param power specifies the output power for the motor controller in the range of
     *              -1.0 to 1.0.
     */
    public void setPower(double power);

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
    public void setPositionSensorInverted(boolean inverted);

}   //interface HalMotorController
