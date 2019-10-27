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

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

/**
 * This class implements a platform independent Pure Pursuit drive for holonomic robots.
 * Essentially, a pure pursuit drive navigates the robot to chase a point along the path. The point to chase
 * is chosen by intersecting a circle centered on the robot with a specific radius with the path, and chasing the
 * "furthest" intersection. The smaller the radius is, the more "tightly" the robot will follow a path, but it will be
 * more prone to oscillation and sharp turns. A larger radius will tend to smooth out turns and corners. Note that the
 * error tolerance must be less than the following distance, so choose them accordingly.
 *
 * A path consists of an array of waypoints, specifying position, velocity, and optionally heading. All other properties
 * of the TrcWaypoint object may be ignored.The path may be low resolution, as this automatically interpolates between
 * waypoints. If you want the robot to maintain heading, call setMaintainHeading(true) and it will ignore all the
 * heading values. Otherwise, call setMaintainHeading(false), ensure that the heading tolerance and pid coefficients
 * are set, and it will follow the heading values specified by the path.
 *
 * A somewhat similar idea is here:
 * https://www.chiefdelphi.com/t/paper-implementation-of-the-adaptive-pure-pursuit-controller/166552 or
 * https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf
 *
 * Note that this paper is for non-holonomic robots. This means that all the turning radius stuff isn't very relevant.
 * Technically, we could impose limits on the turning radius as a function of robot velocity and max rot vel, but that's
 * unnecessarily complicated, in my view. Additionally, it does point injection instead of interpolation, and path
 * smoothing, which we don't do, since a nonzero following distance will naturally smooth it anyway.
 */
public class TrcHolonomicPurePursuitDrive
{
    public enum InterpolationType
    {
        LINEAR(1),
        QUADRATIC(2),
        CUBIC(3),
        QUARTIC(4),
        QUADRATIC_INV(2),
        CUBIC_INV(3),
        QUARTIC_INV(4);

        private int value;

        InterpolationType(int value)
        {
            this.value = value;
        }   //InterpolationType

        public int getValue()
        {
            return value;
        }   //getValue

    }   //enum InterpolationType

    private final String instanceName;
    private final TrcDriveBase driveBase;
    private final TrcTaskMgr.TaskObject driveTaskObj;
    private final TrcPidController posPidCtrl, turnPidCtrl, velPidCtrl;
    private volatile double posTolerance; // Volatile so it can be changed at runtime
    private volatile double followingDistance; // Volatile so it can be changed at runtime
    private TrcPath path;
    private int pathIndex = 1;
    private double positionInput;
    private TrcEvent onFinishedEvent;
    private double timedOutTime;
    private TrcWarpSpace warpSpace;
    private InterpolationType interpolationType = InterpolationType.LINEAR;
    private volatile boolean maintainHeading = false;
    private double startHeading;
    private TrcPose2D referencePose;

    public TrcHolonomicPurePursuitDrive(
            String instanceName, TrcDriveBase driveBase, double followingDistance, double posTolerance,
            double turnTolerance, TrcPidController.PidCoefficients posPidCoeff,
            TrcPidController.PidCoefficients turnPidCoeff, TrcPidController.PidCoefficients velPidCoeff)
    {
        if (driveBase.supportsHolonomicDrive())
        {
            this.driveBase = driveBase;
        }
        else
        {
            throw new IllegalArgumentException(
                "Only holonomic drive bases supported for this pure pursuit implementation!");
        }

        this.instanceName = instanceName;
        warpSpace = new TrcWarpSpace(instanceName + ".warpSpace", 0.0, 360.0);
        setPositionToleranceAndFollowingDistance(posTolerance, followingDistance);

        this.posPidCtrl = new TrcPidController(
                instanceName + ".posPid", posPidCoeff, 0.0, this::getPositionInput);
        this.turnPidCtrl = new TrcPidController(
                instanceName + ".turnPid", turnPidCoeff, turnTolerance, driveBase::getHeading);
        this.velPidCtrl = new TrcPidController(
                instanceName + ".velPid", velPidCoeff, 0.0, this::getVelocityInput);

        posPidCtrl.setAbsoluteSetPoint(true);
        turnPidCtrl.setAbsoluteSetPoint(true);
        velPidCtrl.setAbsoluteSetPoint(true);

        turnPidCtrl.setNoOscillation(true);

        this.driveTaskObj = TrcTaskMgr.getInstance().createTask(instanceName + ".driveTask", this::driveTask);
    }   //TrcHolonomicPurePursuitDrive

    public TrcHolonomicPurePursuitDrive(
            String instanceName, TrcDriveBase driveBase, double followingDistance, double posTolerance,
            TrcPidController.PidCoefficients posPidCoeff, TrcPidController.PidCoefficients turnPidCoeff,
            TrcPidController.PidCoefficients velPidCoeff)
    {
        this(instanceName, driveBase, followingDistance, posTolerance, 5.0,
                posPidCoeff, turnPidCoeff, velPidCoeff);
        setMaintainHeading(true);
    }   //TrcHolonomicPurePursuitDrive

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @Override
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * Maintain heading during path following, or follow the heading values in the path. If not maintaining heading,
     * remember to set the heading tolerance!
     *
     * @param maintainHeading If true, maintain heading. If false, use closed loop to control heading.
     */
    public void setMaintainHeading(boolean maintainHeading)
    {
        this.maintainHeading = maintainHeading;
    }   //setMaintainHeading

    /**
     * Set the turn tolerance for the closed loop control on turning. Only applicable if not maintaining heading.
     *
     * @param turnTolerance The turn tolerance, in degrees. Should be positive.
     */
    public void setTurnTolerance(double turnTolerance)
    {
        turnPidCtrl.setTargetTolerance(turnTolerance);
    }   //setTurnTolerance

    /**
     * Configure the method of interpolating between waypoints. Methods ending with INV will favor the ending point.
     *
     * @param interpolationType The type of interpolation to use.
     */
    public void setInterpolationType(InterpolationType interpolationType)
    {
        this.interpolationType = interpolationType == null ? InterpolationType.LINEAR : interpolationType;
    }   //setInterpolationType

    /**
     * Set both the position tolerance and following distance.
     *
     * @param posTolerance         The distance at which the controller will stop itself.
     * @param followingDistance The distance between the robot and following point.
     */
    public void setPositionToleranceAndFollowingDistance(double posTolerance, double followingDistance)
    {
        if (posTolerance >= followingDistance)
        {
            throw new IllegalArgumentException("Position tolerance must be less than followingDistance!");
        }

        this.followingDistance = followingDistance;
        this.posTolerance = posTolerance;
    }   //setPositionToleranceAndFollowingDistance

    /**
     * Set the position tolerance to end the path. Units need to be consistent.
     *
     * @param posTolerance The distance at which the controller will stop itself.
     */
    public void setPositionTolerance(double posTolerance)
    {
        setPositionToleranceAndFollowingDistance(posTolerance, followingDistance);
    }   //setPositionTolerance

    /**
     * Set the following distance for the pure pursuit controller.
     *
     * @param followingDistance The distance between the robot and following point.
     */
    public void setFollowingDistance(double followingDistance)
    {
        setPositionToleranceAndFollowingDistance(posTolerance, followingDistance);
    }   //setFollowingDistance

    /**
     * Sets the pid coefficients for the position controller. This will work in the middle of an operation as well.
     *
     * @param pidCoefficients The new PID coefficients for the position controller.
     */
    public void setPositionPidCoefficients(TrcPidController.PidCoefficients pidCoefficients)
    {
        posPidCtrl.setPidCoefficients(pidCoefficients);
    }   //setPositionPidCoefficients

    /**
     * Sets the pid coefficients for the turn controller. This will work in the middle of an operation as well.
     *
     * @param pidCoefficients The new PID coefficients for the heading controller.
     */
    public void setTurnPidCoefficients(TrcPidController.PidCoefficients pidCoefficients)
    {
        turnPidCtrl.setPidCoefficients(pidCoefficients);
    }   //setTurnPidCoefficients

    /**
     * Sets the pid coefficients for the position controller. This will work in the middle of an operation as well.
     * Note that velocity controllers should have an F term as well.
     *
     * @param pidCoefficients The new PIDF coefficients for the velocity controller.
     */
    public void setVelocityPidCoefficients(TrcPidController.PidCoefficients pidCoefficients)
    {
        velPidCtrl.setPidCoefficients(pidCoefficients);
    }   //setVelocityPidCoefficients

    /**
     * Start following the supplied path using a pure pursuit controller.
     *
     * @param path The path to follow. Must start at (0,0). Velocity is per second.
     */
    public synchronized void start(TrcPath path)
    {
        start(path, null, 0.0);
    }   //start

    /**
     * Start following the supplied path using a pure pursuit controller. The velocity must always be positive, and
     * the path must start at (0,0). Heading will refer to the direction of the velocity vector.
     *
     * @param path            The path to follow. Must start at (0,0).
     * @param onFinishedEvent When finished, signal this event.
     * @param timeout         Number of seconds after which to cancel this operation. 0.0 for no timeout.
     */
    public synchronized void start(TrcPath path, TrcEvent onFinishedEvent, double timeout)
    {
        if (path == null || path.getSize() == 0)
        {
            throw new IllegalArgumentException("Path cannot be null or empty!");
        }

        cancel();

        if (onFinishedEvent != null)
        {
            onFinishedEvent.clear();
        }
        this.onFinishedEvent = onFinishedEvent;

        this.path = path;
        timedOutTime = timeout == 0.0 ? Double.POSITIVE_INFINITY : TrcUtil.getCurrentTime() + timeout;
        pathIndex = 1;
        positionInput = 0;
        startHeading = driveBase.getHeading();

        posPidCtrl.reset();
        turnPidCtrl.reset();
        velPidCtrl.reset();

        posPidCtrl.setTarget(0.0);
        turnPidCtrl.setTarget(startHeading); // Maintain heading to start

        referencePose = driveBase.getAbsolutePose();
        driveTaskObj.registerTask(TrcTaskMgr.TaskType.OUTPUT_TASK);
    }   //start

    /**
     * Checks if the robot is currently following a path.
     *
     * @return True if the pure pursuit controller is active, false otherwise.
     */
    public synchronized boolean isActive()
    {
        return driveTaskObj.isRegistered();
    }   //isActive

    /**
     * If the controller is currently active, cancel the path following operation. Otherwise, do nothing.
     * If there is an event to signal, mark it as cancelled.
     */
    public synchronized void cancel()
    {
        if (isActive())
        {
            if (onFinishedEvent != null)
            {
                onFinishedEvent.cancel();
            }
            stop();
        }
    }   //cancel

    private double getPositionInput()
    {
        return positionInput;
    }   //getPositionInput

    private double getVelocityInput()
    {
        return TrcUtil.magnitude(driveBase.getXVelocity(), driveBase.getYVelocity());
    }   //getVelocityInput

    private synchronized void stop()
    {
        driveTaskObj.unregisterTask();
        driveBase.stop();
    }   //stop

    private synchronized void driveTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        TrcPose2D pose = driveBase.getPoseRelativeTo(referencePose);
        double robotX = pose.x;
        double robotY = pose.y;
        TrcWaypoint point = getFollowingPoint(robotX, robotY);

        double dist = TrcUtil.magnitude(robotX - point.x, robotY - point.y);
        positionInput = -dist; // Make this negative so the control effort is positive.
        velPidCtrl.setTarget(point.velocity);
        // Only follow heading if we're not maintaining heading
        if (!maintainHeading)
        {
            turnPidCtrl.setTarget(point.heading);
        }

        double posPower = posPidCtrl.getOutput();
        double turnPower = turnPidCtrl.getOutput();
        double velPower = velPidCtrl.getOutput();

        double r = posPower + velPower;
        double theta = Math.toDegrees(Math.atan2(point.x - robotX, point.y - robotY));

        double velocity = TrcUtil.magnitude(driveBase.getXVelocity(), driveBase.getYVelocity());

        TrcDbgTrace.getGlobalTracer().traceInfo("TrcHolonomicPurePursuitDrive.driveTask",
            "Robot: (%.2f,%.2f), RobotVel: %.2f, RobotHeading: %.2f, Target: (%.2f,%.2f), TargetVel: %.2f, TargetHeading: %.2f, pathIndex=%d, r,theta=(%.2f,%.1f)\n",
            robotX, robotY, velocity, pose.heading, point.x, point.y, point.velocity, point.heading,
            pathIndex, r, theta);

        // If we have timed out or finished, stop the operation.
        boolean timedOut = TrcUtil.getCurrentTime() >= timedOutTime;
        boolean posOnTarget = dist <= posTolerance;
        boolean headingOnTarget = maintainHeading || (!maintainHeading && turnPidCtrl.isOnTarget());
        if (timedOut || (pathIndex == path.getSize() - 1 && posOnTarget && headingOnTarget))
        {
            if (onFinishedEvent != null)
            {
                onFinishedEvent.set(true);
            }
            stop();
        }
        else
        {
            driveBase.holonomicDrive_Polar(r, theta, turnPower, pose.heading - startHeading);
        }
    }   //driveTask

    private TrcWaypoint interpolate(TrcWaypoint point1, TrcWaypoint point2, double weight)
    {
        double timestep = interpolate(point1.timeStep, point2.timeStep, weight);
        double x = interpolate(point1.x, point2.x, weight);
        double y = interpolate(point1.y, point2.y, weight);
        double position = interpolate(point1.encoderPosition, point2.encoderPosition, weight);
        double velocity = interpolate(point1.velocity, point2.velocity, weight);
        double acceleration = interpolate(point1.acceleration, point2.acceleration, weight);
        double jerk = interpolate(point1.jerk, point2.jerk, weight);
        double heading = interpolate(point1.heading, warpSpace.getOptimizedTarget(point2.heading, point1.heading),
            weight);
        return new TrcWaypoint(timestep, x, y, position, velocity, acceleration, jerk, heading);
    }   //interpolate

    private double interpolate(double start, double end, double weight)
    {
        if (!TrcUtil.inRange(weight, 0.0, 1.0))
        {
            throw new IllegalArgumentException("Weight must be in range [0,1]!");
        }
        switch (interpolationType)
        {
            case LINEAR:
            case QUADRATIC:
            case CUBIC:
            case QUARTIC:
                weight = Math.pow(weight, interpolationType.getValue());
                break;

            case QUADRATIC_INV:
            case CUBIC_INV:
            case QUARTIC_INV:
                weight = Math.pow(weight, 1.0 / interpolationType.getValue());
                break;
        }
        return (1.0 - weight) * start + weight * end;
    }   //interpolate

    private TrcWaypoint interpolatePoints(TrcWaypoint prev, TrcWaypoint point, double robotX, double robotY)
    {
        // Find intersection of path segment with circle with radius followingDistance and center at robot
        RealVector start = new ArrayRealVector(new double[] { prev.x, prev.y });
        RealVector end = new ArrayRealVector(new double[] { point.x, point.y });
        RealVector robot = new ArrayRealVector(new double[] { robotX, robotY });

        RealVector startToEnd = end.subtract(start);
        RealVector robotToStart = start.subtract(robot);
        // Solve quadratic formula
        double a = startToEnd.dotProduct(startToEnd);
        double b = 2 * robotToStart.dotProduct(startToEnd);
        double c = robotToStart.dotProduct(robotToStart) - followingDistance * followingDistance;

        double discriminant = b * b - 4 * a * c;
        if (discriminant < 0)
        {
            // No valid intersection.
            return null;
        }
        else
        {
            // line is a parametric equation, where t=0 is start, t=1 is end.
            discriminant = Math.sqrt(discriminant);
            double t1 = (-b - discriminant) / (2 * a);
            double t2 = (-b + discriminant) / (2 * a);
            double t = Math.max(t1, t2); // We want the furthest intersection
            // If the intersection is not on the line segment, it's invalid.
            if (!TrcUtil.inRange(t, 0.0, 1.0))
            {
                return null;
            }
            return interpolate(prev, point, t);
        }
    }   //interpolatePoints

    private TrcWaypoint getFollowingPoint(double robotX, double robotY)
    {
        if (pathIndex == path.getSize() - 1)
        {
            return path.getWaypoint(pathIndex);
        }

        for (int i = Math.max(pathIndex, 1); i < path.getSize(); i++)
        {
            // If there is a valid intersection, return it.
            TrcWaypoint interpolated = interpolatePoints(path.getWaypoint(i - 1), path.getWaypoint(i), robotX, robotY);
            if (interpolated != null)
            {
                pathIndex = i;
                return interpolated;
            }
        }

        // There are no points where the distance to any point is followingDistance.
        // Choose the one closest to followingDistance.
        TrcWaypoint closestPoint = path.getWaypoint(pathIndex);
        for (int i = pathIndex; i < path.getSize(); i++)
        {
            TrcWaypoint point = path.getWaypoint(i);
            if (Math.abs(TrcUtil.magnitude(robotX - closestPoint.x, robotY - closestPoint.y) - followingDistance)
                >= Math.abs(TrcUtil.magnitude(robotX - point.x, robotY - point.y) - followingDistance))
            {
                closestPoint = point;
                pathIndex = i;
            }
        }
        return closestPoint;
    }   //getFollowingPoint

}   //class TrcPurePursuitDrive
