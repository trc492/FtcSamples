/*
 * Copyright (c) 2018 Titan Robotics Club (http://www.titanrobotics.com)
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

import java.util.Arrays;

/**
 * This class implements a motion profile for tank drive that consists of paths for the left and right wheels.
 */
public class TrcTankMotionProfile
{
    private TrcPath leftPath;
    private TrcPath rightPath;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param leftPoints specifies an array of waypoints for the left wheels.
     * @param rightPoints specifies an array of waypoints for the right wheels.
     */
    public TrcTankMotionProfile(TrcWaypoint[] leftPoints, TrcWaypoint[] rightPoints)
    {
        if (leftPoints.length != rightPoints.length)
        {
            throw new IllegalArgumentException("leftPoints and rightPoints must have the same length!");
        }

        this.leftPath = new TrcPath(true, leftPoints);
        this.rightPath = new TrcPath(true, rightPoints);
    }   //TrcTankMotionProfile

    /**
     * This method loads the left and right path waypoints from CSV files either on external file system or embedded
     * in resources.
     *
     * @param leftPath specifies the left CSV path on the external file system or CSV name embedded in resources.
     * @param rightPath specifies the right CSV path on the external file system or CSV name embedded in resources.
     * @param loadFromResources specifies true if the CSV file is embedded in resources, false if on file system.
     * @return created motion profile.
     */
    public static TrcTankMotionProfile loadProfileFromCsv(String leftPath, String rightPath, boolean loadFromResources)
    {
        return new TrcTankMotionProfile(
                TrcWaypoint.loadPointsFromCsv(leftPath, loadFromResources),
                TrcWaypoint.loadPointsFromCsv(rightPath, loadFromResources));
    }   //loadProfileFromCsv

    /**
     * This method loads the left and right path waypoints from CSV files on external file system.
     *
     * @param leftPath specifies the left CSV path on the external file system.
     * @param rightPath specifies the right CSV path on the external file system.
     * @return created motion profile.
     */
    public static TrcTankMotionProfile loadProfileFromCsv(String leftPath, String rightPath)
    {
        return loadProfileFromCsv(leftPath, rightPath, false);
    }   //loadFromCsv

    /**
     * This method returns the an array of waypoints for the left wheel path.
     *
     * @return array of waypoints.
     */
    public TrcWaypoint[] getLeftPoints()
    {
        return leftPath.getAllWaypoints();
    }   //getLeftPoints

    /**
     * This method returns the an array of waypoints for the right wheel path.
     *
     * @return array of waypoints.
     */
    public TrcWaypoint[] getRightPoints()
    {
        return rightPath.getAllWaypoints();
    }   //getRightPoints

    /**
     * This method determines the number of waypoints in the motion profile.
     *
     * @return number of waypoints in the motion profile.
     */
    public int getNumPoints()
    {
        return leftPath.getSize();
    }   //getNumPoints

    /**
     * This method returns the minimum time step in the motion profile.
     *
     * @return minimum time step.
     */
    public double getMinTimeStep()
    {
        double minTimeStep = Double.POSITIVE_INFINITY;
        //
        // Scan the left path for the minimum time step.
        //
        for (int i = 0; i < leftPath.getSize(); i++)
        {
            if (leftPath.getWaypoint(i).timeStep < minTimeStep)
            {
                minTimeStep = leftPath.getWaypoint(i).timeStep;
            }
        }
        //
        // Scan the right path for the minimum time step.
        //
        for (int i = 0; i < rightPath.getSize(); i++)
        {
            if (rightPath.getWaypoint(i).timeStep < minTimeStep)
            {
                minTimeStep = rightPath.getWaypoint(i).timeStep;
            }
        }

        if (minTimeStep == Double.POSITIVE_INFINITY)
        {
            throw new IllegalStateException("Empty motion profile!");
        }

        return minTimeStep;
    }   //getMinTimeStep

    /**
     * This method scales the motion profile by distance and time.
     * 
     * @param worldUnitsPerEncoderTick specifies the number of world units in a single encoder tick.
     * @param timeStep specifies the speed denomination in seconds.
     */
    public void scale(double worldUnitsPerEncoderTick, double timeStep)
    {
        for (TrcWaypoint point : leftPath.getAllWaypoints())
        {
            point.encoderPosition /= worldUnitsPerEncoderTick;
            point.velocity /= worldUnitsPerEncoderTick / timeStep;
            point.acceleration /= worldUnitsPerEncoderTick / Math.pow(timeStep, 2);
            point.jerk /= worldUnitsPerEncoderTick / Math.pow(timeStep, 3);
        }

        for (TrcWaypoint point : rightPath.getAllWaypoints())
        {
            point.encoderPosition /= worldUnitsPerEncoderTick;
            point.velocity /= worldUnitsPerEncoderTick / timeStep;
            point.acceleration /= worldUnitsPerEncoderTick / Math.pow(timeStep, 2);
            point.jerk /= worldUnitsPerEncoderTick / Math.pow(timeStep, 3);
        }
    }   //scale

    /**
     * This method returns a copy of this motion profile.
     *
     * @return a copy of this motion profile.
     */
    public TrcTankMotionProfile copy()
    {
        TrcWaypoint[] left = Arrays.copyOf(getLeftPoints(), leftPath.getSize());
        TrcWaypoint[] right = Arrays.copyOf(getRightPoints(), rightPath.getSize());
        return new TrcTankMotionProfile(left, right);
    }   //copy

}   //class TrcTankMotionProfile
