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

public class TrcHolonomicPurePursuitController
{
    private final String instanceName;
    private final TrcDriveBase driveBase;
    private final TrcTaskMgr.TaskObject driveTaskObj;
    private final TrcPidController positionController, headingController, velocityController;
    private volatile double tolerance; // Volatile so it can be changed at runtime
    private volatile double followingDistance; // Volatile so it can be changed at runtime
    private TrcMotionProfilePoint[] path;
    private int pathIndex = 0;
    private double positionInput;
    private TrcEvent onFinishedEvent;
    private double timedOutTime;

    public TrcHolonomicPurePursuitController(String instanceName, TrcDriveBase driveBase, double followingDistance,
        double tolerance, TrcPidController.PidCoefficients pidCoefficients,
        TrcPidController.PidCoefficients turnPidCoefficients, TrcPidController.PidCoefficients velocityPidCoefficients)
    {
        this.instanceName = instanceName;
        this.followingDistance = followingDistance;
        this.tolerance = tolerance;

        this.positionController = new TrcPidController(instanceName + ".positionController", pidCoefficients, 0.0,
            () -> positionInput);
        this.headingController = new TrcPidController(instanceName + ".headingController", turnPidCoefficients, 0.0,
            driveBase::getHeading);
        this.velocityController = new TrcPidController(instanceName + ".velocityController", velocityPidCoefficients,
            0.0, () -> TrcUtil.magnitude(driveBase.getXVelocity(), driveBase.getYVelocity()));

        positionController.setAbsoluteSetPoint(true);
        headingController.setAbsoluteSetPoint(false); // We'll be maintaining heading.
        velocityController.setAbsoluteSetPoint(true);

        if (driveBase.supportsHolonomicDrive())
        {
            this.driveBase = driveBase;
        }
        else
        {
            throw new IllegalArgumentException(
                "Only holonomic drive bases supported for this pure pursuit implementation!");
        }

        this.driveTaskObj = TrcTaskMgr.getInstance().createTask(instanceName + ".driveTask", this::driveTask);
    }

    /**
     * Set the position tolerance to end the path. Units need to be consistent.
     *
     * @param tolerance The distance at which the controller will stop itself.
     */
    public void setTolerance(double tolerance)
    {
        this.tolerance = tolerance;
    }

    /**
     * Set the following distance for the pure pursuit controller.
     *
     * @param distance The distance between the robot and following point.
     */
    public void setFollowingDistance(double distance)
    {
        this.followingDistance = distance;
    }

    /**
     * Sets the pid coefficients for the position controller. This will work in the middle of an operation as well.
     *
     * @param pidCoefficients The new PID coefficients for the position controller.
     */
    public void setPositionPidCoefficients(TrcPidController.PidCoefficients pidCoefficients)
    {
        positionController.setPidCoefficients(pidCoefficients);
    }

    /**
     * Sets the pid coefficients for the heading controller. This will work in the middle of an operation as well.
     *
     * @param pidCoefficients The new PID coefficients for the heading controller.
     */
    public void setHeadingPidCoefficients(TrcPidController.PidCoefficients pidCoefficients)
    {
        headingController.setPidCoefficients(pidCoefficients);
    }

    /**
     * Sets the pid coefficients for the position controller. This will work in the middle of an operation as well.
     * Note that velocity controllers should have an F term as well.
     *
     * @param pidCoefficients The new PIDF coefficients for the velocity controller.
     */
    public void setVelocityPidCoefficients(TrcPidController.PidCoefficients pidCoefficients)
    {
        velocityController.setPidCoefficients(pidCoefficients);
    }

    /**
     * Start following the supplied path using a pure pursuit controller.
     *
     * @param path The path to follow. Must start at (0,0). Velocity is per second.
     */
    public synchronized void start(TrcMotionProfilePoint[] path)
    {
        start(path, null, 0.0);
    }

    /**
     * Start following the supplied path using a pure pursuit controller. The velocity must always be positive, and
     * the path must start at (0,0). Heading will refer to the direction of the velocity vector.
     *
     * @param path            The path to follow. Must start at (0,0).
     * @param onFinishedEvent When finished, signal this event.
     * @param timeout         Number of seconds after which to cancel this operation. 0.0 for no timeout.
     */
    public synchronized void start(TrcMotionProfilePoint[] path, TrcEvent onFinishedEvent, double timeout)
    {
        if (path == null || path.length == 0)
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
        pathIndex = 0;
        positionInput = 0;

        positionController.reset();
        headingController.reset();
        velocityController.reset();

        positionController.setTarget(0.0);
        headingController.setTarget(0.0); // This is a relative controller

        driveBase.resetOdometry(true, false);
        driveTaskObj.registerTask(TrcTaskMgr.TaskType.OUTPUT_TASK);
    }

    /**
     * Checks if the robot is currently following a path.
     *
     * @return True if the pure pursuit controller is active, false otherwise.
     */
    public synchronized boolean isActive()
    {
        return driveTaskObj.isRegistered();
    }

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
    }

    private synchronized void stop()
    {
        driveTaskObj.unregisterTask();
        driveBase.stop();
    }

    private synchronized void driveTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        double robotX = driveBase.getXPosition();
        double robotY = driveBase.getYPosition();
        TrcMotionProfilePoint point = getFollowingPoint(robotX, robotY);

        double dist = TrcUtil.magnitude(robotX - point.x, robotY - point.y);
        positionInput = -dist; // Make this negative so the control effort is positive.
        velocityController.setTarget(point.velocity);

        double posPower = positionController.getOutput();
        double turnPower = headingController.getOutput();
        double velPower = headingController.getOutput();

        double r = posPower + velPower;
        double theta = Math.toRadians(point.heading);

        // If we have timed out or finished, stop the operation.
        if (TrcUtil.getCurrentTime() >= timedOutTime || (point == path[path.length - 1] && dist <= tolerance))
        {
            onFinishedEvent.set(true);
            stop();
        }
        else
        {
            driveBase.holonomicDrive_Polar(r, theta, turnPower);
        }
    }

    private TrcMotionProfilePoint getFollowingPoint(double robotX, double robotY)
    {
        Double prevDist = null;
        for (int i = pathIndex; i < path.length; i++)
        {
            TrcMotionProfilePoint point = path[i];
            double dist = TrcUtil.magnitude(robotX - point.x, robotY - point.y);
            if (prevDist != null && dist >= followingDistance && prevDist <= followingDistance)
            {
                pathIndex = i;
                return point;
            }
            prevDist = dist;
        }

        // There are no points where the distance to any point is followingDistance.
        // Choose the one closest to followingDistance.
        TrcMotionProfilePoint closestPoint = path[pathIndex];
        for (int i = pathIndex; i < path.length; i++)
        {
            TrcMotionProfilePoint point = path[i];
            if (Math.abs(TrcUtil.magnitude(robotX - closestPoint.x, robotY - closestPoint.y) - followingDistance)
                <= Math.abs(TrcUtil.magnitude(robotX - point.x, robotY - point.y) - followingDistance))
            {
                closestPoint = point;
                pathIndex = i;
            }
        }
        return closestPoint;
    }
}
