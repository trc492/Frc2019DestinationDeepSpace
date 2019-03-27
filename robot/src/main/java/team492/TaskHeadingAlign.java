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

import frclib.FrcRemoteVisionProcessor;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTaskMgr;
import trclib.TrcWarpSpace;

public class TaskHeadingAlign
{
    private enum State
    {
        START, TURN, DRIVE, DONE
    }

    private static final boolean ASSISTED_DRIVE = true; // if false, be flush with target. if true, assist driving to target.

    private static final double[] HATCH_YAWS = new double[] { 0.0, 90.0 - RobotInfo.ROCKET_SIDE_ANGLE, 90.0,
        90.0 + RobotInfo.ROCKET_SIDE_ANGLE, 180.0, 270.0 - RobotInfo.ROCKET_SIDE_ANGLE, 270.0,
        270.0 + RobotInfo.ROCKET_SIDE_ANGLE };
    private static final double[] CARGO_YAWS = new double[] { 0.0, 90.0, 270.0 };

    private Robot robot;
    private TrcStateMachine<State> sm;
    private TrcEvent event;
    private TrcTaskMgr.TaskObject turnTaskObj;
    private TrcWarpSpace warpSpace;
    private double targetHeading;
    private TrcEvent onFinishedEvent;
    private TaskAutoDeploy.DeployType deployType;

    public TaskHeadingAlign(Robot robot)
    {
        this.robot = robot;
        sm = new TrcStateMachine<>("TaskHeadingAlign.StateMachine");
        event = new TrcEvent("TaskHeadingAlign.event");
        turnTaskObj = TrcTaskMgr.getInstance().createTask("TurnTask", this::turnTask);
        warpSpace = new TrcWarpSpace("warpSpace", 0.0, 360.0);
    }

    public void start()
    {
        start(null);
    }

    public void start(TrcEvent event)
    {
        start(robot.pickup.cargoDetected() ? TaskAutoDeploy.DeployType.CARGO : TaskAutoDeploy.DeployType.HATCH, event);
    }

    public void start(TaskAutoDeploy.DeployType deployType, TrcEvent event)
    {
        this.deployType = deployType;
        sm.start(State.TURN);
        setEnabled(true);
        this.onFinishedEvent = event;
        if (event != null && ASSISTED_DRIVE)
        {
            throw new IllegalStateException("Assisted driving is on! Turn only is disabled!");
        }
    }

    public void cancel()
    {
        if (isActive())
        {
            setEnabled(false);
            robot.pidDrive.cancel();
            sm.stop();
        }
    }

    public boolean isActive()
    {
        return sm.isEnabled();
    }

    private double getTargetRotation()
    {
        if (deployType == TaskAutoDeploy.DeployType.PICKUP_HATCH)
        {
            return 180.0;
        }

        double[] yaws = deployType == TaskAutoDeploy.DeployType.CARGO ? CARGO_YAWS : HATCH_YAWS;

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

    private void setEnabled(boolean enabled)
    {
        if (enabled)
        {
            turnTaskObj.registerTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        }
        else
        {
            turnTaskObj.unregisterTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        }
    }

    private void turnTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        State state = sm.checkReadyAndGetState();
        if (state != null)
        {
            switch (state)
            {
                case START:
                    if (ASSISTED_DRIVE)
                    {
                        FrcRemoteVisionProcessor.RelativePose pose = robot.vision.getAveragePose(5, false);
                        if (pose != null)
                        {
                            targetHeading = pose.theta + robot.driveBase.getHeading();
                            sm.setState(State.TURN);
                        }
                    }
                    else
                    {
                        targetHeading = getTargetRotation();
                        sm.setState(State.TURN);
                    }
                    break;

                case TURN:
                    robot.enableSmallGains();
                    robot.targetHeading = targetHeading;
                    robot.pidDrive.setTarget(0.0, 0.0, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, ASSISTED_DRIVE ? State.DRIVE : State.DONE);
                    break;

                case DRIVE:
                    FrcRemoteVisionProcessor.RelativePose pose = robot.vision.getAveragePose(5, false);
                    double currHeading = robot.driveBase.getHeading();
                    if (pose != null)
                    {
                        robot.targetHeading = pose.theta + currHeading;
                    }
                    double turnPower =
                        RobotInfo.GYRO_TURN_KP_SMALL * (warpSpace.getOptimizedTarget(robot.targetHeading, currHeading)
                            - currHeading);
                    double drivePower = robot.rightDriveStick.getYWithDeadband(true);
                    robot.globalTracer.tracePrintf(
                        "HeadingAlign aligning: targetHeading=%.1f,currHeading=%.1f,drivePower=%.2f,turnPower=%.2f",
                        robot.targetHeading, currHeading, drivePower, turnPower);
                    robot.driveBase.holonomicDrive(0.0, drivePower, turnPower);
                    // This state does not exit, as it has no exit condition. The driver must release the button.
                    break;

                case DONE:
                    cancel();
                    if (onFinishedEvent != null)
                    {
                        onFinishedEvent.set(true);
                    }
                    break;
            }
        }
    }
}
