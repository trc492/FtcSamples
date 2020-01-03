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

package trclib;

import java.util.Locale;

/**
 * This interface provides the definitions and methods for a sensor to be an odometry sensor. An odometry sensor will
 * report odometry data which contains both timestamp, position and velocity information.
 */
public interface TrcOdometrySensor
{
    /**
     * This class implements the generic sensor odometry. It consists of the position as well as velocity info. If
     * the sensor does not support velocity data. This class keeps track of the previous timestamp and position so we
     * can calculate the velocity ourselves.
     */
    public class Odometry
    {
        Object sensor;
        double prevTimestamp;
        double currTimestamp;
        double prevPos;
        double currPos;
        double velocity;

        /**
         * Constructor: Create an instance of the object.
         *
         * @param sensor specifies the sensor object.
         * @param prevTimestamp specifies the timestamp of the previous data.
         * @param currTimestamp specifies the timestamp of the current data.
         * @param prevPos specifies the previous position data.
         * @param currPos specifies the current position data.
         * @param velocity specifies the velocity data. This data may be considered redundant because one can derive
         *                 velocity from (deltaPosition/deltaTime). However, some sensors may support velocity data,
         *                 so this field may contain sensor reported velocity or derived velocity.
         */
        public Odometry(
                Object sensor, double prevTimestamp, double currTimestamp, double prevPos, double currPos,
                double velocity)
        {
            this.sensor = sensor;
            this.prevTimestamp = prevTimestamp;
            this.currTimestamp = currTimestamp;
            this.prevPos = prevPos;
            this.currPos = currPos;
            this.velocity = velocity;
        }   //Odometry

        /**
         * Constructor: Create an instance of the object.
         *
         * @param sensor specifies the sensor object.
         */
        public Odometry(Object sensor)
        {
            this.sensor = sensor;
            this.prevTimestamp = this.currTimestamp = 0.0;
            this.prevPos = this.currPos = this.velocity = 0.0;
        }   //Odometry

        /**
         * This method returns the string representation of the object.
         *
         * @return string representation of the object.
         */
        @Override
        public String toString()
        {
            return String.format(
                    Locale.US, "name=%s,prevTime=%.3f,currTime=%.3f,prevPos=%.1f,currPos=%.1f,vel=%.1f",
                    sensor, prevTimestamp, currTimestamp, prevPos, currPos, velocity);
        }   //toString

        /**
         * This method creates and returns a copy of this odometry.
         *
         * @return a copy of this odometry.
         */
        public Odometry clone()
        {
            return new Odometry(sensor, prevTimestamp, currTimestamp, prevPos, currPos, velocity);
        }   //clone

    }   //class Odometry

    /**
     * This method resets the odometry data and sensor.
     *
     * @param resetHardware specifies true to do a hardware reset, false to do a software reset. Hardware reset may
     *                      require some time to complete and will block this method from returning until finish.
     */
    void resetOdometry(boolean resetHardware);

    /**
     * This method returns a copy of the odometry data. It must be a copy so it won't change while the caller is
     * accessing the data fields.
     *
     * @return a copy of the odometry data.
     */
    Odometry getOdometry();

}   //interface TrcOdometrySensor
