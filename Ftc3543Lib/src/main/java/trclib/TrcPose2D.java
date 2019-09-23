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

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealVector;

/**
 * This class implements a 2D pose object that represents the positional state of an object.
 */
public class TrcPose2D
{
    public double x;
    public double y;
    public double heading;
    public double xVel;
    public double yVel;
    public double turnRate;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param x specifies the x component of the position.
     * @param y specifies the y component of the position.
     * @param heading specifies the heading.
     * @param xVel specifies the x component of the velocity.
     * @param yVel specifies the y component of the velocity.
     * @param turnRate specifies the turn velocity.
     */
    public TrcPose2D(double x, double y, double heading, double xVel, double yVel, double turnRate)
    {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.xVel = xVel;
        this.yVel = yVel;
        this.turnRate = turnRate;
    }   //TrcPose2D

    /**
     * Constructor: Create an instance of the object.
     *
     * @param x specifies the x coordinate of the position.
     * @param y specifies the y coordinate of the position.
     * @param heading specifies the heading.
     */
    public TrcPose2D(double x, double y, double heading)
    {
        this(x, y, heading, 0.0, 0.0, 0.0);
    }   //TrcPose2D

    /**
     * Constructor: Create an instance of the object.
     *
     * @param x specifies the x coordinate of the position.
     * @param y specifies the y coordinate of the position.
     */
    public TrcPose2D(double x, double y)
    {
        this(x, y, 0.0, 0.0, 0.0, 0.0);
    }   //TrcPose2D

    /**
     * Constructor: Create an instance of the object.
     */
    public TrcPose2D()
    {
        this(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }   //TrcPose2D

    /**
     * This method creates and returns a copy of the given pose.
     *
     * @return a copy of this pose.
     */
    public static TrcPose2D copyOf(TrcPose2D pose)
    {
        return new TrcPose2D(pose.x, pose.y, pose.heading, pose.xVel, pose.yVel, pose.turnRate);
    }   //copyOf

    /**
     * This method creates and returns a copy of this pose.
     *
     * @return a copy of this pose.
     */
    public TrcPose2D clone()
    {
        return TrcPose2D.copyOf(this);
    }   //clone

    /**
     * This method returns the position in vector form.
     *
     * @return position vector.
     */
    public RealVector getPositionVector()
    {
        return MatrixUtils.createRealVector(new double[] { x, y });
    }   //getPositionVector

    /**
     * This method returns the velocity in vector form.
     *
     * @return velocity vector.
     */
    public RealVector getVelocityVector()
    {
        return MatrixUtils.createRealVector(new double[] { xVel, yVel });
    }   //getVelocityVector

    /**
     * This method subtracts the given pose from this pose and returns a relative position from the given pose.
     *
     * @param pose specifies the pose to be subtracted from this one.
     * @return relative pose from the given pose.
     */
    public TrcPose2D poseDistance(TrcPose2D pose)
    {
        return new TrcPose2D(x - pose.x, y - pose.y, heading, xVel, yVel, turnRate);
    }   //poseDistance

    /**
     * This method returns a transformed pose relative to the given pose.
     *
     * @param pose specifies the reference pose.
     * @return pose relative to the given pose.
     */
    public TrcPose2D relativeTo(TrcPose2D pose)
    {
        TrcPose2D transformed = poseDistance(pose);
        RealVector newPos = TrcUtil.rotateCCW(transformed.getPositionVector(), pose.heading);
        RealVector newVel = TrcUtil.rotateCCW(transformed.getVelocityVector(), pose.heading);

        transformed.x = newPos.getEntry(0);
        transformed.y = newPos.getEntry(1);

        transformed.xVel = newVel.getEntry(0);
        transformed.yVel = newVel.getEntry(1);

        return transformed;
    }   //relativeTo

}   //class TrcPose2D
