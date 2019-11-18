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

import java.util.Locale;

/**
 * This class implements a 2D pose object that represents the positional state of an object.
 */
public class TrcPose2D
{
    private static final String moduleName = "TrcPose2D";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

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
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                    TrcDbgTrace.getGlobalTracer():
                    new TrcDbgTrace(moduleName, tracingEnabled, traceLevel, msgLevel);
        }

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
     * This method returns the string representation of the pose.
     *
     * @return string representation of the pose.
     */
    @Override
    public String toString()
    {
        return String.format(Locale.US, "(x=%.1f,y=%.1f,angle=%.1f,xVel=%.1f,yVel=%.1f,turnRate=%.1f)",
                x, y, heading, xVel, yVel, turnRate);
    }   //toString

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
     * This method sets this pose to be the same as the given pose.
     *
     * @param pose specifies the pose to make this pose equal to.
     */
    public void setAs(TrcPose2D pose)
    {
        this.x = pose.x;
        this.y = pose.y;
        this.heading = pose.heading;
        this.xVel = pose.xVel;
        this.yVel = pose.yVel;
        this.turnRate = pose.turnRate;
    }   //setAs

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
        transformed.heading = heading - pose.heading;

        transformed.xVel = newVel.getEntry(0);
        transformed.yVel = newVel.getEntry(1);

        return transformed;
    }   //relativeTo

    /**
     * This method translates this pose with the x and y offset in reference to the heading of the pose.
     *
     * @param xOffset specifies the x offset in reference to the heading of the pose.
     * @param yOffset specifies the y offset in reference to the heading of the pose.
     * @return translated pose.
     */
    public TrcPose2D translatePose(double xOffset, double yOffset)
    {
        final String funcName = "translatePose";
        TrcPose2D newPose = clone();
        double headingRadians = Math.toRadians(newPose.heading);
        double cosHeading = Math.cos(headingRadians);
        double sinHeading = Math.sin(headingRadians);

        newPose.x += xOffset*cosHeading + yOffset*sinHeading;
        newPose.y += -xOffset*sinHeading + yOffset*cosHeading;

        if (debugEnabled)
        {
            dbgTrace.traceInfo(funcName, "xOffset=%.1f, yOffset=%.1f, Pose:%s, newPose:%s",
                    xOffset, yOffset, this, newPose);
        }

        return newPose;
    }   //translatePose

}   //class TrcPose2D
