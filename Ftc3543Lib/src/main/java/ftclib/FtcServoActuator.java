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

import trclib.TrcEnhancedServo;
import trclib.TrcEvent;

/**
 * This class implements a platform dependent servo actuator. A servo actuator consists of one or two servos. Servo
 * actuator and supports speed control the servo actuator.
 */
public class FtcServoActuator
{
    /**
     * This class contains all the parameters related to the servo actuator.
     */
    public static class Parameters
    {
        double maxStepRate = 0.0;
        double minPos = 0.0;
        double maxPos = 1.0;
        boolean servo1Inverted = false;
        boolean servo2Inverted = false;
        double retractPos = 0.0;
        double retractTime = 0.5;
        double extendPos = 1.0;
        double extendTime = 0.5;

        /**
         * This method sets Step Mode parameters of the servo actuator. Step Mode allows a servo to be speed
         * controlled.
         *
         * @param maxStepRate specifies the maximum step rate in physical unit per second.
         * @param minPos specifies the minimum position limit of the servo actuator.
         * @param maxPos specifies the maximum position limit of the servo actuator.
         * @return this parameter object.
         */
        public Parameters setStepParams(double maxStepRate, double minPos, double maxPos)
        {
            this.maxStepRate = maxStepRate;
            this.minPos = minPos;
            this.maxPos = maxPos;
            return this;
        }   //setStepParams

        public Parameters setInverted(boolean servo1Inverted, boolean servo2Inverted)
        {
            this.servo1Inverted = servo1Inverted;
            this.servo2Inverted = servo2Inverted;
            return this;
        }   //setInverted

        /**
         * This method sets the retract parameters of the servo actuator.
         *
         * @param retractPos specifies the retract position in physical unit
         * @param retractTime specifies the time in seconds required to retract from a full extend position.
         * @return this parameter object.
         */
        public Parameters setRetractParams(double retractPos, double retractTime)
        {
            this.retractPos = retractPos;
            this.retractTime = retractTime;
            return this;
        }   //setRetractParams

        /**
         * This method sets the extend parameters of the servo actuator.
         *
         * @param extendPos specifies the extend position in physical unit
         * @param extendTime specifies the time in seconds required to extend from a full retract position.
         * @return this parameter object.
         */
        public Parameters setExtendParams(double extendPos, double extendTime)
        {
            this.extendPos = extendPos;
            this.extendTime = extendTime;
            return this;
        }   //setExtendParams

    }   //class Parameters

    private Parameters params;
    protected FtcServo servo1, servo2 = null;
    protected TrcEnhancedServo enhancedServo;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param servo1Name specifies the name of the first servo.
     * @param servo2Name specifies the name of the second servo, can be null if none.
     * @param params specifies the parameters to set up the servo actuator.
     */
    public FtcServoActuator(String servo1Name, String servo2Name, Parameters params)
    {
        if (servo1Name == null)
        {
            throw new IllegalArgumentException("Servo1 must not be null.");
        }

        this.params = params;
        servo1 = new FtcServo(servo1Name);
        servo1.setInverted(params.servo1Inverted);
        if (servo2Name != null)
        {
            servo2 = new FtcServo(servo2Name);
            servo2.setInverted(params.servo2Inverted);
        }
        enhancedServo = new TrcEnhancedServo("servoActuator." + servo1Name, servo1, servo2);
        if (params.maxStepRate != 0.0)
        {
            enhancedServo.setStepMode(params.maxStepRate, params.minPos, params.maxPos);
        }
    }   //FtcServoActuator

    /**
     * Constructor: Create an instance of the object.
     *
     * @param servoName specifies the name of the servo.
     * @param params specifies the parameters to set up the servo actuator.
     */
    public FtcServoActuator(String servoName, Parameters params)
    {
        this(servoName, null, params);
    }   //FtcServoActuator

    /**
     * This method sets the servo actuator to its retract position.
     */
    public void retract()
    {
        enhancedServo.setPosition(params.retractPos);
    }   //retract

    /**
     * This method sets the servo actuator to its retract position and signals the given event after retract time
     * has expired.
     *
     * @param event specifies the event to be signaled after retract time has expired.
     */
    public void retract(TrcEvent event)
    {
        enhancedServo.setPosition(params.retractPos, params.retractTime, event);
    }   //retract

    /**
     * This method sets the servo actuator to its retract position and signals the given event after the specified
     * time has expired.
     *
     * @param time specifies the time to wait before the event is signaled.
     * @param event specifies the event to be signaled after specified time has expired.
     */
    public void retract(double time, TrcEvent event)
    {
        enhancedServo.setPosition(params.retractPos, time, event);
    }   //retract

    /**
     * This method sets the servo actuator to its extend position.
     */
    public void extend()
    {
        enhancedServo.setPosition(params.extendPos);
    }   //extend

    /**
     * This method sets the servo actuator to its extend position and signals the given event after extend time
     * has expired.
     *
     * @param event specifies the event to be signaled after extend time has expired.
     */
    public void extend(TrcEvent event)
    {
        enhancedServo.setPosition(params.extendPos, params.extendTime, event);
    }   //extend

    /**
     * This method sets the servo actuator to its extend position and signals the given event after the specified
     * time has expired.
     *
     * @param time specifies the time to wait before the event is signaled.
     * @param event specifies the event to be signaled after specified time has expired.
     */
    public void extend(double time, TrcEvent event)
    {
        enhancedServo.setPosition(params.extendPos, time, event);
    }   //extend

    /**
     * This method moves the servo actuator as if it is speed controlled. The speed of the actuator movement is
     * proportional to the specified power.
     *
     * @param power specifies the power value from -1.0 to 1.0 where the magnitude proportional to its speed.
     */
    public void setPower(double power)
    {
        enhancedServo.setPower(power);
    }   //setPower

    /**
     * This method returns the last set servo actuator physical position.
     *
     * @return last set position in physical unit.
     */
    public double getPosition()
    {
        return enhancedServo.getPosition();
    }   //getPosition

    /**
     * This method sets the servo actuator position in physical unit.
     *
     * @param position specifies the position in physical unit.
     */
    public void setPosition(double position)
    {
        enhancedServo.setPosition(position);
    }   //setPosition

    /**
     * This method sets the servo actuator position in physical unit and signal the given event after the given time
     * has expired.
     *
     * @param position specifies the position in physical unit.
     * @param time specifies the time to wait before the event is signaled.
     * @param event specifies the event to be signaled after specified time has expired.
     */
    public void setPosition(double position, double time, TrcEvent event)
    {
        enhancedServo.setPosition(position, time, event);
    }   //setPosition

}   //class FtcServoActuator
