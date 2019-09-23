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
 * This class implements a path. A path is consists of an array of waypoints, and can be used for path following,
 * such as motion profiling, pure pursuit, etc. Since heading could be in degrees or radians, each path object specifies
 * the units of its heading value, and can be converted. If the timesteps are not specified, velocity and position data
 * can be used to infer approximate timesteps. Paths can be read from CSV files.
 */
public class TrcPath
{
    private TrcWaypoint[] waypoints;
    private boolean inDegrees;

    /**
     * Constructor: Create a new TrcPath object. Must supply at least 2 entries.
     *
     * @param inDegrees specifies true if the heading values are in degrees, false if they are radians.
     * @param waypoints specifies the array of points that will constitute this path. Cannot be null, and must have
     *                  at least 2 waypoints.
     */
    public TrcPath(boolean inDegrees, TrcWaypoint... waypoints)
    {
        if (waypoints == null || waypoints.length <= 1)
        {
            throw new IllegalArgumentException("Waypoints cannot be null or have less than 2 entries!");
        }

        this.inDegrees = inDegrees;
        this.waypoints = waypoints;
    }   //TrcPath

    /**
     * Constructor: Create a new TrcPath object. Must supply at least 2 entries.
     *
     * @param waypoints specifies the array of points that will constitute this path. Cannot be null, and must have
     *                  at least 2 waypoints.
     */
    public TrcPath(TrcWaypoint... waypoints)
    {
        this(true, waypoints);
    }   //TrcPath

    /**
     * This method loads waypoints from a CSV file and create a path with them.
     *
     * @param path specifies the file path or the resource name where we load the waypoints.
     * @param inDegrees specifies true if the heading values are in degrees, false if they are radians.
     * @param loadFromResources specifies true if waypoints are loaded from resources, false if from file path.
     * @return created path with the loaded waypoints.
     */
    public static TrcPath loadPathFromCsv(String path, boolean inDegrees, boolean loadFromResources)
    {
        return new TrcPath(inDegrees, TrcWaypoint.loadPointsFromCsv(path, loadFromResources));
    }   //loadPathFromCsv

    /**
     * This method returns the waypoint at the given index of the path.
     *
     * @param index specifies the index to the path array.
     * @return the waypoint at the given index.
     */
    public TrcWaypoint getWaypoint(int index)
    {
        return waypoints[index];
    }   //getWaypoint

    /**
     * This method returns the number of waypoints in this path.
     *
     * @return the number of waypoints in this path.
     */
    public int getSize()
    {
        return waypoints.length;
    }   //getSize

    /**
     * This method returns the array of waypoints of the path.
     *
     * @return the waypoint array. This is the same instance, so modifying it will affect other users of this path.
     */
    public TrcWaypoint[] getAllWaypoints()
    {
        return waypoints;
    }   //getAllWaypoints

    /**
     * Create a new path identical to this one, except the heading values are in degrees. If this path's headings are
     * already in degrees, just return deep copy.
     *
     * @return A TrcPath object which the same waypoints except all heading values are now in degrees, if they weren't before.
     */
    public TrcPath toDegrees()
    {
        TrcWaypoint[] waypoints = new TrcWaypoint[this.waypoints.length];
        for (int i = 0; i < this.waypoints.length; i++)
        {
            TrcWaypoint waypoint = new TrcWaypoint(this.waypoints[i]);
            // If already in degree mode, don't convert again.
            waypoint.heading = inDegrees ? waypoint.heading : Math.toDegrees(waypoint.heading);
            waypoints[i] = waypoint;
        }
        return new TrcPath(true, waypoints);
    }

    /**
     * Create a new path identical to this one, except the heading values are in radians. If this path's headings are
     * already in radians, just return deep copy.
     *
     * @return A TrcPath object which the same waypoints except all heading values are now in radians, if they weren't before.
     */
    public TrcPath toRadians()
    {
        TrcWaypoint[] waypoints = new TrcWaypoint[this.waypoints.length];
        for (int i = 0; i < this.waypoints.length; i++)
        {
            TrcWaypoint waypoint = new TrcWaypoint(this.waypoints[i]);
            // If already in degree mode, don't convert again.
            waypoint.heading = inDegrees ? Math.toRadians(waypoint.heading) : waypoint.heading;
            waypoints[i] = waypoint;
        }
        return new TrcPath(true, waypoints);
    }

    /**
     * If not already in degrees, convert this path's heading values to degrees.
     */
    public void mapSelfToDegrees()
    {
        if (!inDegrees)
        {
            inDegrees = true;
            for (TrcWaypoint waypoint : waypoints)
            {
                waypoint.heading = Math.toDegrees(waypoint.heading);
            }
        }
    }

    /**
     * If not already in radians, convert this path's heading values to radians.
     */
    public void mapSelfToRadians()
    {
        if (inDegrees)
        {
            inDegrees = false;
            for (TrcWaypoint waypoint : waypoints)
            {
                waypoint.heading = Math.toRadians(waypoint.heading);
            }
        }
    }

    /**
     * Get the estimated duration of the entire path.
     *
     * @return The estimated time duration, in the same units as <code>timeStep</code>.
     */
    public double getPathDuration()
    {
        double duration = 0;
        for (TrcWaypoint waypoint : waypoints)
        {
            duration += waypoint.timeStep;
        }
        return duration;
    }

    /**
     * Get the curved length of the entire path.
     *
     * @return The curved length of the entire path, in the same units as <code>x</code> and <code>y</code>.
     */
    public double getArcLength()
    {
        double length = 0;
        TrcWaypoint waypoint = waypoints[0];
        for (int i = 1; i < waypoints.length; i++)
        {
            length += waypoint.distanceTo(waypoints[i]);
            waypoint = waypoints[i];
        }
        return length;
    }

    /**
     * Use velocity and position data to infer the timesteps of the waypoint.
     */
    public void inferTimeSteps()
    {
        for (int i = 0; i < waypoints.length - 1; i++)
        {
            TrcWaypoint point = waypoints[i];
            TrcWaypoint next = waypoints[i + 1];
            double displacement = point.distanceTo(next);
            // Area of trapezoid: (v1+v2)/2 * t = d
            // t = d / ((v1+v2)/2)
            point.timeStep = displacement / TrcUtil.average(point.velocity, next.velocity);
        }
        // Assume last waypoint has the same timestep as second to last
        waypoints[waypoints.length - 1].timeStep = waypoints[waypoints.length - 2].timeStep;
    }
}
