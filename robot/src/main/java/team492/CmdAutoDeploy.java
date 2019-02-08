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

public class CmdAutoDeploy
{
    private enum State
    {
        START, ORIENT, ALIGN, RAISE_ELEVATOR, DEPLOY, DONE
    }

    public enum DeployType
    {
        CARGO, HATCH, PICKUP_CARGO, PICKUP_HATCH
    }

    private static final String instanceName = "CmdAutoDeploy";

    private Robot robot;
    private TrcStateMachine<State> sm;
    private TrcEvent event;
    private RaspiVision.RelativePose pose;
    private TrcEvent onFinishedEvent;
    private TrcTaskMgr.TaskObject alignmentTask;
    private double elevatorHeight;
    private double travelHeight;
    private DeployType deployType;

    public CmdAutoDeploy(Robot robot)
    {
        this.robot = robot;
        alignmentTask = TrcTaskMgr.getInstance().createTask(instanceName + ".alignTask", this::alignTask);

        sm = new TrcStateMachine<>(instanceName + ".stateMachine");
        event = new TrcEvent(instanceName + ".event");
    }

    /**
     * Automatically drive to the nearest hatch/cargo port and raise the elevator to the required height.
     *
     * @param elevatorHeight The height to raise the elevator to.
     * @param deployType     The type of deployment.
     * @param event          The event to signal when done.
     */
    public void start(double elevatorHeight, DeployType deployType, TrcEvent event)
    {
        this.elevatorHeight = elevatorHeight;
        this.deployType = deployType;
        this.onFinishedEvent = event;
        setEnabled(true);
    }

    /**
     * Is the auto alignment command running?
     *
     * @return True if running, false otherwise.
     */
    public boolean isRunning()
    {
        return sm.isEnabled();
    }

    /**
     * Cancel the auto alignment command.
     */
    public void cancel()
    {
        cancel(true);
    }

    /**
     * Cancel the auto alignment command, optionally without stopping the motors. This is so if the drivers want to
     * override it, the robot doesn't stop in the middle. It will allow the robot to switch into manual control faster.
     *
     * @param hardStop True to stop the wheels, false otherwise.
     */
    public void cancel(boolean hardStop)
    {
        if (isRunning())
        {
            if (onFinishedEvent != null)
            {
                onFinishedEvent.set(true);
            }
            stop(hardStop);
        }
    }

    private void stop(boolean hardStop)
    {
        sm.stop();
        robot.pidDrive.cancel(hardStop);
        robot.elevator.setPower(0.0);
        onFinishedEvent = null;
        setEnabled(false);
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
            alignmentTask.unregisterTask(TrcTaskMgr.TaskType.POSTPERIODIC_TASK);
        }
    }

    private void alignTask(TrcTaskMgr.TaskType type, TrcRobot.RunMode mode)
    {
        State state = sm.checkReadyAndGetState();

        if (state != null)
        {
            switch (state)
            {
                case START:
                    if (robot.vision.getConsecutiveTargetFrames() > 5)
                    {
                        pose = robot.vision.getAveragePose(5);
                        sm.setState(State.ORIENT);
                    }
                    break;

                case ORIENT:
                    // We don't need an event for this, since it should be done by the time everything else finishes.
                    robot.pickup.setPickupAngle(RobotInfo.PICKUP_MAX_POS);

                    TrcEvent elevatorEvent = new TrcEvent(instanceName + ".elevatorEvent");
                    travelHeight = Math.min(RobotInfo.ELEVATOR_DRIVE_POS, elevatorHeight);
                    robot.elevator.setPosition(travelHeight, elevatorEvent);

                    robot.targetHeading = robot.driveBase.getHeading() + pose.objectYaw;
                    robot.pidDrive.setTarget(0.0, 0.0, robot.targetHeading, false, event);

                    sm.addEvent(elevatorEvent);
                    sm.addEvent(event);
                    sm.waitForEvents(State.ALIGN);
                    break;

                case ALIGN:
                    robot.elevator.setPosition(travelHeight); // Hold it, since the set with event doesn't hold it.
                    double r = pose.r;
                    double theta = pose.theta - pose.objectYaw;
                    double x = Math.sin(Math.toRadians(theta)) * r + RobotInfo.CAMERA_OFFSET;
                    double y = Math.cos(Math.toRadians(theta)) * r + RobotInfo.CAMERA_DEPTH;
                    robot.pidDrive.setTarget(x, y, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.RAISE_ELEVATOR);
                    break;

                case RAISE_ELEVATOR:
                    robot.elevator.setPosition(elevatorHeight, event);
                    sm.waitForSingleEvent(event, State.DEPLOY);
                    break;

                case DEPLOY:
                    robot.elevator.setPosition(elevatorHeight); // Hold it at that height
                    event.clear();
                    switch(deployType)
                    {
                        case CARGO:
                            robot.pickup.deployCargo(event);
                            break;

                        case HATCH:
                            robot.pickup.deployHatch(event);
                            break;

                        case PICKUP_CARGO:
                            robot.pickup.pickupCargo(event);
                            break;

                        case PICKUP_HATCH:
                            robot.pickup.pickupHatch(event);
                            break;
                    }
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                default:
                case DONE:
                    if (onFinishedEvent != null)
                    {
                        onFinishedEvent.set(true);
                    }
                    stop(true);
                    break;
            }
        }
    }
}
