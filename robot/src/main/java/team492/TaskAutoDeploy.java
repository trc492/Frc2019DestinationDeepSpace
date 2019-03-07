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

package team492;

import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTaskMgr;
import trclib.TrcWarpSpace;

public class TaskAutoDeploy
{
    private enum State
    {
        START, ORIENT, REFRESH_VISION, ALIGN, ALIGN_X, ALIGN_Y, RAISE_ELEVATOR, DEPLOY, DONE
    }

    public enum DeployType
    {
        CARGO, HATCH, PICKUP_HATCH // Intentionally missing cargo pickup
    }

    public enum DeployLevel
    {
        LOW(0), MEDIUM(1), HIGH(2); // For rocket (cargo and hatch). Low is also cargo ship hatch.

        private int index;

        DeployLevel(int i)
        {
            this.index = i;
        }

        public int getIndex()
        {
            return index;
        }
    }

    private static final boolean USE_VISION_YAW = false;
    private static final boolean REFRESH_VISION = false; // Only applicable if using vision yaw.
    private static final boolean ALIGN_BOTH = true; // if true, align x and y axis at the same time
    private static final double[] HATCH_YAWS = new double[] { 0.0, 90.0 - RobotInfo.ROCKET_SIDE_ANGLE, 90.0,
        90.0 + RobotInfo.ROCKET_SIDE_ANGLE, 270.0 - RobotInfo.ROCKET_SIDE_ANGLE, 270.0,
        270.0 + RobotInfo.ROCKET_SIDE_ANGLE };
    private static final double[] CARGO_YAWS = new double[] { 0.0, 90.0, 270.0 };

    private static final String instanceName = "TaskAutoDeploy";

    private Robot robot;
    private TrcStateMachine<State> sm;
    private TrcEvent event;
    private RaspiVision.RelativePose pose;
    private TrcEvent onFinishedEvent;
    private TrcTaskMgr.TaskObject alignmentTask;
    private double elevatorHeight;
    private double travelHeight;
    private DeployType deployType;
    private TrcWarpSpace warpSpace;
    private boolean alignOnly = false;
    private State afterRefreshVision;

    public TaskAutoDeploy(Robot robot)
    {
        this.robot = robot;
        alignmentTask = TrcTaskMgr.getInstance().createTask(instanceName + ".alignTask", this::alignTask);

        sm = new TrcStateMachine<>(instanceName + ".stateMachine");
        event = new TrcEvent(instanceName + ".event");
        warpSpace = new TrcWarpSpace(instanceName + ".warpSpace", 0.0, 360.0);
    }

    /**
     * Automatically align and optionally deploy the hatch/cargo. If you want to pickup a hatch, you must use the other
     * start method.
     *
     * @param level The level to deploy to.
     */
    public void start(DeployLevel level)
    {
        start(level, robot.pickup.cargoDetected() ? DeployType.CARGO : DeployType.HATCH, null);
    }

    /**
     * Automatically align and optionally deploy the hatch/cargo. If you want to pickup a hatch, you must use the other
     * start method.
     *
     * @param level The level to deploy to.
     * @param event The event to signal when done.
     */
    public void start(DeployLevel level, TrcEvent event)
    {
        start(level, robot.pickup.cargoDetected() ? DeployType.CARGO : DeployType.HATCH, event);
    }

    /**
     * Automatically drive to the nearest hatch/cargo port and raise the elevator to the required height.
     *
     * @param level      The level (low medium high) to deploy to. Irrelevant if deployType is CARGO_PICKUP
     * @param deployType The type of deployment.
     */
    public void start(DeployLevel level, DeployType deployType)
    {
        start(level, deployType, null);
    }

    /**
     * Automatically drive to the nearest hatch/cargo port and raise the elevator to the required height.
     *
     * @param level      The level (low medium high) to deploy to. Irrelevant if deployType is CARGO_PICKUP
     * @param deployType The type of deployment.
     * @param event      The event to signal when done.
     */
    public void start(DeployLevel level, DeployType deployType, TrcEvent event)
    {
        if (deployType == DeployType.PICKUP_HATCH)
        {
            level = DeployLevel.LOW;
        }

        if (event != null)
        {
            event.clear();
        }

        this.elevatorHeight = getElevatorHeight(level, deployType);
        this.deployType = deployType;
        this.onFinishedEvent = event;
        setEnabled(true);
    }

    private double getElevatorHeight(DeployLevel level, DeployType deployType)
    {
        switch (deployType)
        {
            case PICKUP_HATCH:
            case HATCH:
                return RobotInfo.ELEVATOR_HATCH_ROCKET_POSITIONS[level.getIndex()];

            default:
            case CARGO:
                return RobotInfo.ELEVATOR_CARGO_ROCKET_POSITIONS[level.getIndex()];
        }
    }

    /**
     * Is the auto alignment command running?
     *
     * @return True if running, false otherwise.
     */
    public boolean isActive()
    {
        return sm.isEnabled();
    }

    /**
     * Cancel the auto alignment command.
     */
    public void cancel()
    {
        if (isActive())
        {
            if (onFinishedEvent != null)
            {
                onFinishedEvent.set(true);
            }
            stop();
        }
    }

    public void setAlignOnly(boolean alignOnly)
    {
        this.alignOnly = alignOnly;
    }

    private void stop()
    {
        sm.stop();
        robot.stopSubsystems();
        onFinishedEvent = null;
        setEnabled(false);
        robot.setHalfBrakeModeEnabled(true, robot.driveInverted);
        afterRefreshVision = null;
        robot.pickup.retractHatchDeployer();
    }

    private void setEnabled(boolean enabled)
    {
        if (enabled)
        {
            alignmentTask.registerTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
            sm.start(State.START);
        }
        else
        {
            alignmentTask.unregisterTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        }
    }

    private double getTargetRotation(DeployType deployType)
    {
        if (deployType == DeployType.PICKUP_HATCH)
        {
            return 180.0;
        }

        double[] yaws = deployType == DeployType.CARGO ? CARGO_YAWS : HATCH_YAWS;

        double currentRot = robot.driveBase.getHeading();
        double targetYaw = yaws[0];
        for (double yaw : yaws)
        {
            yaw = warpSpace.getOptimizedTarget(yaw, currentRot);
            double error = Math.abs(yaw - currentRot);
            double currError = Math.abs(targetYaw - currentRot);
            if (error < currError)
            {
                targetYaw = yaw;
            }
        }
        return targetYaw;
    }

    private double getXDistance()
    {
        return pose.x + RobotInfo.CAMERA_OFFSET;
    }

    private double getYDistance()
    {
        double y = pose.y - RobotInfo.CAMERA_DEPTH;
        if (deployType == DeployType.PICKUP_HATCH)
        {
            y += 8;
        }

        return y;
    }

    private void alignTask(TrcTaskMgr.TaskType type, TrcRobot.RunMode mode)
    {
        State state = sm.checkReadyAndGetState();

        if (state != null)
        {
            robot.globalTracer.traceInfo("AlignTask", "Curr State: %s", state.toString());
            robot.dashboard.displayPrintf(12, "State: %s", state.toString());

            TrcEvent elevatorEvent;
            double x, y;

            switch (state)
            {
                case START:
                    robot.setHalfBrakeModeEnabled(false, false);
                    robot.enableSmallGains();
                    if (USE_VISION_YAW)
                    {
                        pose = robot.vision.getAveragePose(5, true);
                        if (pose != null)
                        {
                            sm.setState(State.ORIENT);
                        }
                    }
                    else
                    {
                        sm.setState(State.ORIENT);
                    }
                    break;

                case ORIENT:
                    // We don't need an event for this, since it should be done by the time everything else finishes.
                    robot.pickup.setPickupAngle(RobotInfo.PICKUP_PERP_TO_GROUND_POS);

                    if (USE_VISION_YAW)
                    {
                        robot.targetHeading = robot.driveBase.getHeading() + pose.objectYaw;
                    }
                    else
                    {
                        robot.targetHeading = getTargetRotation(deployType);
                    }
                    robot.pidDrive.setTarget(0.0, 0.0, robot.targetHeading, false, event);

                    State nextState = State.ALIGN;
                    if (!USE_VISION_YAW || REFRESH_VISION || !ALIGN_BOTH)
                    {
                        afterRefreshVision = ALIGN_BOTH ? State.ALIGN : State.ALIGN_X;
                        nextState = State.REFRESH_VISION;
                    }
                    sm.waitForSingleEvent(event, nextState);
                    break;

                case REFRESH_VISION:
                    pose = robot.vision.getAveragePose(5, true);
                    if (pose != null)
                    {
                        sm.setState(afterRefreshVision != null ? afterRefreshVision : State.DONE);
                    }
                    break;

                case ALIGN:
                    elevatorEvent = new TrcEvent(instanceName + ".elevatorEvent");
                    travelHeight = Math.min(RobotInfo.ELEVATOR_DRIVE_POS, elevatorHeight);
                    robot.elevator.setPosition(travelHeight, elevatorEvent);

                    if (USE_VISION_YAW && !REFRESH_VISION)
                    {
                        // Vision data was taken BEFORE the ORIENT state, so we need to transform it to match robot coords.
                        double r = pose.r;
                        double theta = pose.theta - pose.objectYaw;
                        x = Math.sin(Math.toRadians(theta)) * r + RobotInfo.CAMERA_OFFSET;
                        y = Math.cos(Math.toRadians(theta)) * r - RobotInfo.CAMERA_DEPTH;
                    }
                    else
                    {
                        // Vision data was taken AFTER the ORIENT state, so no transformation required.
                        x = getXDistance();
                        y = getYDistance();
                    }
                    robot.globalTracer
                        .traceInfo("DeployTask", "State=ALIGN, x=%.1f,y=%.1f,rot=%.1f", x, y, robot.targetHeading);
                    robot.pidDrive.setTarget(x, y, robot.targetHeading, false, event);
                    sm.addEvent(elevatorEvent);
                    sm.addEvent(event);
                    sm.waitForEvents(State.RAISE_ELEVATOR, 0.0, true);
                    break;

                case ALIGN_X:
                    elevatorEvent = new TrcEvent(instanceName + ".elevatorEvent");
                    travelHeight = Math.min(RobotInfo.ELEVATOR_DRIVE_POS, elevatorHeight);
                    robot.elevator.setPosition(travelHeight, elevatorEvent);

                    x = getXDistance();
                    y = 0.0;

                    robot.globalTracer
                        .traceInfo("DeployTask", "State=ALIGN_X, x=%.1f,y=%.1f,rot=%.1f", x, y, robot.targetHeading);
                    robot.pidDrive.setTarget(x, y, robot.targetHeading, false, event);
                    sm.addEvent(elevatorEvent);
                    sm.addEvent(event);
                    sm.waitForEvents(State.REFRESH_VISION, 0.0, true);
                    afterRefreshVision = State.ALIGN_Y;
                    break;

                case ALIGN_Y:
                    robot.elevator.setPosition(travelHeight);

                    x = getXDistance();
                    y = getYDistance();

                    robot.globalTracer
                        .traceInfo("DeployTask", "State=ALIGN_Y, x=%.1f,y=%.1f,rot=%.1f", x, y, robot.targetHeading);
                    robot.pidDrive.setTarget(x, y, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DEPLOY);
                    break;

                case RAISE_ELEVATOR:
                    robot.elevator.setPosition(elevatorHeight, event);
                    sm.waitForSingleEvent(event, State.DEPLOY);
                    break;

                case DEPLOY:
                    robot.elevator.setPosition(elevatorHeight); // Hold it at that height
                    if (alignOnly)
                    {
                        sm.setState(State.DONE);
                    }
                    else
                    {
                        event.clear();
                        switch (deployType)
                        {
                            case CARGO:
                                robot.pickup.deployCargo(event);
                                break;

                            case HATCH:
                                robot.pickup.deployHatch(event);
                                break;

                            case PICKUP_HATCH:
                                robot.pickup.pickupHatch(event);
                                break;
                        }
                        sm.waitForSingleEvent(event, State.DONE);
                    }
                    break;

                default:
                case DONE:
                    if (onFinishedEvent != null)
                    {
                        onFinishedEvent.set(true);
                    }
                    stop();
                    break;
            }
        }
    }
}
