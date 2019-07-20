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
import trclib.*;

public class TaskAutoAlign
{
    private enum State
    {
        START, REFRESH_VISION, ALIGN
    }

    private static final boolean USE_VISION_YAW = false;
    private static final boolean USE_DRIVER_Y = true;

    private static final double[] HATCH_YAWS_ANGLED = new double[] { 90.0 - RobotInfo.ROCKET_SIDE_ANGLE,
        90.0 + RobotInfo.ROCKET_SIDE_ANGLE, 270.0 - RobotInfo.ROCKET_SIDE_ANGLE, 270.0 + RobotInfo.ROCKET_SIDE_ANGLE };
    private static final double[] HATCH_YAWS_FLAT = new double[] { 0.0, 90.0, 180.0, 270.0,
        270.0 + RobotInfo.ROCKET_SIDE_ANGLE };
    private static final double[] CARGO_YAWS = new double[] { 0.0, 90.0, 270.0 };

    private static final String instanceName = "TaskAutoAlign";

    private Robot robot;
    private TrcStateMachine<State> sm;
    private FrcRemoteVisionProcessor.RelativePose pose;
    private TrcTaskMgr.TaskObject alignmentTask;
    private TrcWarpSpace warpSpace;
    private TrcPidController xPidController, yPidController, turnPidController;
    private double targetHeading;
    private double lastElevatorPower;
    private boolean deployAtAngle = false;

    public TaskAutoAlign(Robot robot)
    {
        this.robot = robot;
        alignmentTask = TrcTaskMgr.getInstance().createTask(instanceName + ".alignTask", this::alignTask);

        sm = new TrcStateMachine<>(instanceName + ".stateMachine");
        warpSpace = new TrcWarpSpace(instanceName + ".warpSpace", 0.0, 360.0);

        xPidController = new TrcPidController("XPid",
            new TrcPidController.PidCoefficients(RobotInfo.ENCODER_X_KP), 1.0, robot.driveBase::getXPosition);
        yPidController = new TrcPidController("YPid",
            new TrcPidController.PidCoefficients(RobotInfo.ENCODER_Y_KP, RobotInfo.ENCODER_Y_KI,
                RobotInfo.ENCODER_Y_KD), 1.0, robot.driveBase::getYPosition);
        yPidController.setOutputLimit(0.1);
        yPidController.setRampRate(0.2);
        turnPidController = new TrcPidController("TurnPid",
            new TrcPidController.PidCoefficients(RobotInfo.GYRO_TURN_KP_SMALL), 1.0, () -> robot.vision.getLastPose().theta);
        turnPidController.setAbsoluteSetPoint(false);
    }

    /**
     * Automatically drive to the nearest hatch/cargo port and raise the elevator to the required height.
     */
    public void start(boolean deployAtAngle)
    {
        this.deployAtAngle = deployAtAngle;
        setEnabled(true);
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
            stop();
        }
    }

    private void stop()
    {
        sm.stop();
        robot.driveBase.stop();
        robot.pidDrive.cancel();
        setEnabled(false);
        lastElevatorPower = 0.0;
        robot.dashboard.displayPrintf(12, "Curr State: DISABLED");
        robot.globalTracer.traceInfo("AutoAlign.stop", "AutoAlign cancelled!");
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

    private double getTargetRotation()
    {
        double[] yaws;
        if (robot.pickup.cargoDetected())
        {
            yaws = CARGO_YAWS;
        }
        else if (deployAtAngle)
        {
            yaws = HATCH_YAWS_ANGLED;
        }
        else
        {
            yaws = HATCH_YAWS_FLAT;
        }

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

    private void alignTask(TrcTaskMgr.TaskType type, TrcRobot.RunMode mode)
    {
        State state = sm.checkReadyAndGetState();

        if (state != null)
        {
            robot.globalTracer.traceInfo("AlignTask", "Curr State: %s", state.toString());
            robot.dashboard.displayPrintf(12, "State: %s", state.toString());

            switch (state)
            {
                case START:
                    pose = null;
                    lastElevatorPower = 0.0;
                    sm.setState(State.REFRESH_VISION);
                    break;

                case REFRESH_VISION:
                    pose = robot.vision.getMedianPose(5, false);
                    if (pose != null)
                    {
                        targetHeading = getTargetRotation();
                        xPidController.setTarget(pose.x);
                        yPidController.setTarget(pose.y);
                        sm.setState(State.ALIGN);
                        robot.globalTracer
                            .traceInfo("AlignTask", "State=REFRESH_VISION, x=%.1f,y=%.1f,rot=%.1f", pose.x, pose.y,
                                robot.targetHeading);
                        turnPidController.setTarget(0.0);
                    }
                    break;

                case ALIGN:
                    FrcRemoteVisionProcessor.RelativePose newPose = robot.vision.getMedianPose(3, false);
                    if (newPose != null)
                    {
                        pose = newPose;
                        robot.globalTracer.traceInfo("AlignTask", "State=ALIGN, x=%.1f,y=%.1f,rot=%.1f", pose.x, pose.y,
                            robot.targetHeading);
                        xPidController.setTarget(pose.x);
                        yPidController.setTarget(pose.y);
                    }
//                    if (USE_VISION_YAW)
//                    {
//                        turnPidController.setTarget(robot.driveBase.getHeading() + pose.objectYaw);
//                    }
//                    else
//                    {
//                        turnPidController.setTarget(targetHeading);
//                    }
                    double xPower = xPidController.getOutput();
                    double yPower = USE_DRIVER_Y ?
                        robot.rightDriveStick.getYWithDeadband(true) :
                        yPidController.getOutput();
                    double turnPower = turnPidController.getOutput();
                    //robot.driveBase.holonomicDrive(xPower, yPower, turnPower);
                    robot.driveBase.holonomicDrive(0, 0, turnPidController.getOutput());
                    robot.dashboard.displayPrintf(7, "State=ALIGN, xPower=%.2f,yPower=%.2f,rotPower=%.2f", xPower, yPower, turnPower);

                    double elevatorPower = robot.operatorStick.getYWithDeadband(true);
                    if (elevatorPower != lastElevatorPower)
                    {
                        robot.elevator.setPower(elevatorPower);
                        lastElevatorPower = elevatorPower;
                    }
                    break;
            }
        }
    }
}
