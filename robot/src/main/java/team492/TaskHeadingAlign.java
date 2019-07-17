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

import trclib.TrcPidController;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTaskMgr;
import trclib.TrcWarpSpace;

public class TaskHeadingAlign
{
    private enum State
    {
        START, DRIVE
    }

    private static final double[] HATCH_YAWS_ANGLED = new double[] { 90.0 - RobotInfo.ROCKET_SIDE_ANGLE,
        90.0 + RobotInfo.ROCKET_SIDE_ANGLE, 270.0 - RobotInfo.ROCKET_SIDE_ANGLE, 270.0 + RobotInfo.ROCKET_SIDE_ANGLE };
    private static final double[] HATCH_YAWS_FLAT = new double[] { 0.0, 90.0, 180.0, 270.0 };
    private static final double[] CARGO_YAWS = new double[] { 0.0, 90.0, 270.0 };

    private static final String instanceName = "TaskHeadingAlign";

    private Robot robot;
    private TrcStateMachine<State> sm;
    private TrcTaskMgr.TaskObject turnTaskObj;
    private TrcWarpSpace warpSpace;
    private double lastElevatorPower = 0.0;
    private TrcPidController turnPidController;
    private boolean alignToAngle;

    public TaskHeadingAlign(Robot robot)
    {
        this.robot = robot;
        sm = new TrcStateMachine<>(instanceName + ".StateMachine");
        turnTaskObj = TrcTaskMgr.getInstance().createTask(instanceName + ".turnTask", this::turnTask);
        warpSpace = new TrcWarpSpace(instanceName + ".warpSpace", 0.0, 360.0);
        TrcPidController.PidCoefficients pidCoefficients = new TrcPidController.PidCoefficients(
            RobotInfo.GYRO_TURN_KP_SMALL, RobotInfo.GYRO_TURN_KI_SMALL, RobotInfo.GYRO_TURN_KD_SMALL);
        turnPidController = new TrcPidController("TurnPid", pidCoefficients, 1.0, robot.driveBase::getHeading);
        turnPidController.setAbsoluteSetPoint(true);
    }

    public void start(boolean alignToAngle)
    {
        if (isActive())
        {
            cancel();
        }
        sm.start(State.START);
        turnPidController.reset();
        this.alignToAngle = alignToAngle;
        setEnabled(true);
        robot.driveBase.acquireExclusiveAccess(instanceName);
    }

    public void cancel()
    {
        if (isActive())
        {
            setEnabled(false);
            sm.stop();
            lastElevatorPower = 0.0;
            turnPidController.reset();
            robot.driveBase.releaseExclusiveAccess(instanceName);
        }
    }

    public boolean isActive()
    {
        return sm.isEnabled();
    }

    private double getTargetRotation()
    {
        double[] yaws;
        if (robot.pickup.cargoDetected())
        {
            yaws = CARGO_YAWS;
        }
        else if (alignToAngle)
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

    private void updateTarget(double targetHeading)
    {
        double target = warpSpace.getOptimizedTarget(targetHeading, robot.driveBase.getHeading());
        turnPidController.setTarget(target);
    }

    private void turnTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        State state = sm.checkReadyAndGetState();
        if (state != null)
        {
            switch (state)
            {
                case START:
                    updateTarget(getTargetRotation());
                    String log = String
                        .format("RotAssist: cargo=%b,alignToAngle=%b,currHeading=%.1f,targetHeading=%.1f",
                            robot.pickup.cargoDetected(), alignToAngle, robot.driveBase.getHeading(),
                            turnPidController.getTarget());
                    robot.dashboard.displayPrintf(15, log);
                    robot.globalTracer.traceInfo("TaskHeadingAlign.turnTask", log);
                    sm.setState(State.DRIVE);
                    break;

                case DRIVE:
                    // Drive the robot towards the target
                    double turnPower = turnPidController.getOutput();
                    double xPower = robot.leftDriveStick.getXWithDeadband(true);
                    double yPower = robot.rightDriveStick.getYWithDeadband(true);
                    robot.driveBase.holonomicDrive(instanceName, xPower, yPower, turnPower);
                    // Drive the elevator. Operator controls will be run by the button callbacks in teleop
                    double elevatorPower = robot.operatorStick.getYWithDeadband(true);
                    if (elevatorPower != lastElevatorPower)
                    {
                        robot.elevator.setPower(elevatorPower);
                        lastElevatorPower = elevatorPower;
                    }
                    // This state does not exit, as it has no exit condition. The driver must release the button.
                    break;
            }
        }
    }
}
