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

/**
 * This class implements a platform independent drive base odometry device. A drive base odometry device consists of
 * two to five sensors, zero to two for the X direction and one to two for Y, and an angle sensor for orientation.
 * All sensors must be able to report position as well as velocity data. This class supports four different odometry
 * configurations.
 *
 * Configuration 1:
 * This configuration has 2 sensors, 1 for Y and one for angle. The Y sensor must be installed at the centroid
 * of the robot so that an in-place turn will not produce any displacement in Y. It is most likely an encoder on
 * passive omni-wheels. The angle sensor is most likely a gyro. This configuration is not for holonomic drive base
 * since it doesn't provide X odometry.
 *
 * Configuration 2:
 * This configuration has 3 sensors, two for Y and one for angle. The two Y sensors must be installed in-line
 * with and equidistant from the centroid of the robot and are most likely installed in the middle between the front
 * and back wheels, one on the left and one on the right. The installation locations of the Y sensors are such that
 * if the robot is doing an in-place turn, the two Y sensors will cancel each other out netting a zero displacement.
 * Just like the first configuration, this configuration is not for holonomic drive base since it doesn't provide X
 * odometry.
 *
 * Configuration 3:
 * This configuration has 4 sensors, two for Y, one for X and one for angle. The two Y sensors must be installed
 * just like the 3-sensor configuration. The X sensor must be installed at the centroid of the robot so in-place
 * turning will not produce any displacement in X. The X and Y sensors form an H-configuration. This is the minimum
 * configuration for drive base that's capable of holonomic drive.
 *
 * Configuration 4:
 * This configuration has 5 sensors, two for Y, two for X and one for angle. The two Y sensors must be installed
 * just like the configurations with 3 or 4 sensors. The two X sensors must also be installed in-line with and
 * equidistant from the centroid of the robot and it must be orthogonal to the two Y sensors most likely one in mid
 * front and the other in mid back. With this configuration, the X and Y sensor pairs will cancel each other out
 * during an in-place turn producing net zero X and Y displacements.
 */
public class TrcDriveBaseOdometry
{
    private final TrcOdometrySensor x1Sensor;
    private final TrcOdometrySensor y1Sensor;
    private final TrcOdometrySensor x2Sensor;
    private final TrcOdometrySensor y2Sensor;
    private final TrcOdometrySensor angleSensor;
    private double xScale = 1.0;
    private double yScale = 1.0;
    private double angleScale = 1.0;
    private double prevAvgXPos, prevAvgYPos;

    /**
     * Constructor: Create an instance of the object (Configuration 4).
     *
     * @param x1Sensor specifies the first X sensor if any.
     * @param y1Sensor specifies the first Y sensor.
     * @param x2Sensor specifies the second X sensor if any.
     * @param y2Sensor specifies the second Y sensor if any.
     * @param angleSensor specifies the angle sensor.
     */
    public TrcDriveBaseOdometry(
            TrcOdometrySensor x1Sensor, TrcOdometrySensor y1Sensor,
            TrcOdometrySensor x2Sensor, TrcOdometrySensor y2Sensor,
            TrcOdometrySensor angleSensor)
    {
        // y1 and angle sensors are the required minimum.
        if (y1Sensor == null || angleSensor == null)
        {
            throw new IllegalArgumentException("Must have at least one Y and an angle sensor.");
        }

        this.x1Sensor = x1Sensor;
        this.y1Sensor = y1Sensor;
        this.x2Sensor = x2Sensor;
        this.y2Sensor = y2Sensor;
        this.angleSensor = angleSensor;

        resetOdometry(true, true);
    }   //TrcDriveBaseOdometry

    /**
     * Constructor: Create an instance of the object (Configuration 3).
     *
     * @param x1Sensor specifies the first X sensor.
     * @param y1Sensor specifies the first Y sensor.
     * @param y2Sensor specifies the second Y sensor.
     * @param angleSensor specifies the angle sensor.
     */
    public TrcDriveBaseOdometry(
            TrcOdometrySensor x1Sensor, TrcOdometrySensor y1Sensor,
            TrcOdometrySensor y2Sensor, TrcOdometrySensor angleSensor)
    {
        this(x1Sensor, y1Sensor, null, y2Sensor, angleSensor);
    }   //TrcDriveBaseOdometry

    /**
     * Constructor: Create an instance of the object (Configuration 2).
     *
     * @param y1Sensor specifies the first Y sensor.
     * @param y2Sensor specifies the second Y sensor.
     * @param angleSensor specifies the angle sensor.
     */
    public TrcDriveBaseOdometry(
            TrcOdometrySensor y1Sensor, TrcOdometrySensor y2Sensor, TrcOdometrySensor angleSensor)
    {
        this(null, y1Sensor, null, y2Sensor, angleSensor);
    }   //TrcDriveBaseOdometry

    /**
     * Constructor: Create an instance of the object (Configuration 1).
     *
     * @param ySensor specifies the Y sensor.
     * @param angleSensor specifies the angle sensor.
     */
    public TrcDriveBaseOdometry(TrcOdometrySensor ySensor, TrcOdometrySensor angleSensor)
    {
        this(null, ySensor, null, null, angleSensor);
    }   //TrcDriveBaseOdometry

    /**
     * This method sets the scaling factors for both X, Y and angle data. This is typically used to scale encoder
     * counts to physical units such as inches. If the scale of a direction is not provided, it must be set to 1.0.
     *
     * @param xScale specifies the scale factor for the X direction.
     * @param yScale specifies the scale factor for the Y direction.
     * @param angleScale specifies the scale factor the the angle.
     */
    public synchronized void setOdometryScales(double xScale, double yScale, double angleScale)
    {
        this.xScale = xScale;
        this.yScale = yScale;
        this.angleScale = angleScale;
    }   //setOdometryScales

    /**
     * This method sets the scaling factors for Y and angle data. This is typically used to scale encoder
     * counts to physical units such as inches. If the scale of a direction is not provided, it must be set to 1.0.
     *
     * @param yScale specifies the scale factor for the Y direction.
     * @param angleScale specifies the scale factor the the angle.
     */
    public void setOdometryScales(double yScale, double angleScale)
    {
        this.setOdometryScales(1.0, yScale, angleScale);
    }   //setOdometryScales

    /**
     * This method sets the scaling factors for Y data. This is typically used to scale encoder counts to
     * physical units such as inches. If the scale of a direction is not provided, it must be set to 1.0.
     *
     * @param yScale specifies the scale factor for the Y direction.
     */
    public void setOdometryScales(double yScale)
    {
        this.setOdometryScales(1.0, yScale, 1.0);
    }   //setOdometryScales

    /**
     * This method resets the odometry device and data.
     *
     * @param resetHardware specifies true to do a hardware reset on the sensor devices, false otherwise.
     * @param resetAngle specifies true to reset the angle sensor, false to skip resetting angle sensor.
     */
    public synchronized void resetOdometry(boolean resetHardware, boolean resetAngle)
    {
        TrcOdometrySensor.Odometry x1Odometry = null;
        TrcOdometrySensor.Odometry x2Odometry = null;
        TrcOdometrySensor.Odometry y1Odometry = null;
        TrcOdometrySensor.Odometry y2Odometry = null;

        if (x1Sensor != null)
        {
            x1Sensor.resetOdometry(resetHardware);
            x1Odometry = x1Sensor.getOdometry();
        }

        if (y1Sensor != null)
        {
            y1Sensor.resetOdometry(resetHardware);
            y1Odometry = y1Sensor.getOdometry();
        }

        if (x2Sensor != null)
        {
            x2Sensor.resetOdometry(resetHardware);
            x2Odometry = x2Sensor.getOdometry();
        }

        if (y2Sensor != null)
        {
            y2Sensor.resetOdometry(resetHardware);
            y2Odometry = y2Sensor.getOdometry();
        }

        if (angleSensor != null && resetAngle)
        {
            angleSensor.resetOdometry(resetHardware);
        }

        prevAvgXPos = averageOdometry(x1Odometry, x2Odometry, true);
        prevAvgYPos = averageOdometry(y1Odometry, y2Odometry, true);
    }   //resetOdometry

    /**
     * This method reads all the sensors and calculates the delta displacement from the last odometry update. Only
     * position data are deltas but not velocities because we only integrate position data into absolute field
     * position.
     *
     * @return the delta odometry.
     */
    public synchronized TrcDriveBase.Odometry getOdometryDelta()
    {
        TrcOdometrySensor.Odometry x1Odometry = x1Sensor != null? x1Sensor.getOdometry(): null;
        TrcOdometrySensor.Odometry x2Odometry = x2Sensor != null? x2Sensor.getOdometry(): null;
        TrcOdometrySensor.Odometry y1Odometry = y1Sensor != null? y1Sensor.getOdometry(): null;
        TrcOdometrySensor.Odometry y2Odometry = y2Sensor != null? y2Sensor.getOdometry(): null;
        TrcOdometrySensor.Odometry angleOdometry = angleSensor.getOdometry();
        TrcDriveBase.Odometry odometryDelta = new TrcDriveBase.Odometry();

        double avgXPos = averageOdometry(x1Odometry, x2Odometry, true);
        double avgYPos = averageOdometry(y1Odometry, y2Odometry, true);
        double avgXVel = averageOdometry(x1Odometry, x2Odometry, false);
        double avgYVel = averageOdometry(y1Odometry, y2Odometry, false);

        odometryDelta.position.x = (avgXPos - prevAvgXPos)*xScale;
        odometryDelta.position.y = (avgYPos - prevAvgYPos)*yScale;
        odometryDelta.position.angle = (angleOdometry.currPos - angleOdometry.prevPos)*angleScale;
        odometryDelta.velocity.x = avgXVel*xScale;
        odometryDelta.velocity.y = avgYVel*yScale;
        odometryDelta.velocity.angle = angleOdometry.velocity*angleScale;

        prevAvgXPos = avgXPos;
        prevAvgYPos = avgYPos;

        return odometryDelta;
    }   //getOdometryDelta

    /**
     * This method is called to average two sensors of the same direction.
     *
     * @param odometry1 specifies the odometry of the first sensor.
     * @param odometry2 specifies the odometry of the second sensor.
     * @param position specifies true to average the position data, false to average velocity data.
     * @return sensor averaged value.
     */
    private double averageOdometry(
            TrcOdometrySensor.Odometry odometry1, TrcOdometrySensor.Odometry odometry2, boolean position)
    {
        double value = 0.0;
        int sensorCount = 0;

        if (odometry1 != null)
        {
            value += position? odometry1.currPos: odometry1.velocity;
            sensorCount++;
        }

        if (odometry2 != null)
        {
            value += position? odometry2.currPos: odometry2.velocity;
            sensorCount++;
        }

        return value/sensorCount;
    }   //averageOdometry

}   //class TrcDriveBaseOdometry
