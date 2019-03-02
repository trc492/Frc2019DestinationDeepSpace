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

import frclib.FrcJoystick;
import hallib.HalDashboard;
import trclib.TrcLoopTimeCounter;
import trclib.TrcRobot;
import trclib.TrcRobot.RunMode;

import java.util.Arrays;

public class FrcTeleOp implements TrcRobot.RobotMode
{
    private enum DriveMode
    {
        MECANUM_MODE, ARCADE_MODE, TANK_MODE
    } // enum DriveMode

    public static final boolean DEBUG_LOOP_TIME = true;

    protected Robot robot;

    private enum DriveSpeed
    {
        SLOW, MEDIUM, FAST
    }

    private DriveSpeed driveSpeed = DriveSpeed.MEDIUM;
    private DriveMode driveMode = DriveMode.MECANUM_MODE;
    private boolean gyroAssist = false;
    private TrcLoopTimeCounter loopTimeCounter;

    public FrcTeleOp(Robot robot)
    {
        this.robot = robot;
    } // FrcTeleOp

    //
    // Implements TrcRobot.RunMode interface.
    //

    @Override
    public void startMode(RunMode prevMode, RunMode nextMode)
    {
        //
        // Configure joysticks.
        //
        robot.leftDriveStick.setButtonHandler(this::leftDriveStickButtonEvent);
        robot.leftDriveStick.setYInverted(true);

        robot.rightDriveStick.setButtonHandler(this::rightDriveStickButtonEvent);
        robot.rightDriveStick.setYInverted(true);

        robot.operatorStick.setButtonHandler(this::operatorStickButtonEvent);
        robot.operatorStick.setYInverted(false);

        robot.buttonPanel.setButtonHandler(this::buttonPanelButtonEvent);

        driveSpeed = DriveSpeed.MEDIUM;

        if (Robot.USE_RASPI_VISION)
        {
            robot.vision.setRingLightEnabled(true);
        }

        if (DEBUG_LOOP_TIME)
        {
            loopTimeCounter = new TrcLoopTimeCounter(1.0);
        }
    } // startMode

    @Override
    public void stopMode(RunMode prevMode, RunMode nextMode)
    {
        if (Robot.USE_RASPI_VISION)
        {
            robot.vision.setRingLightEnabled(false);
        }
    } // stopMode

    @Override
    public void runPeriodic(double elapsedTime)
    {
        double elevatorPower = robot.operatorStick.getYWithDeadband(true);

        double leftDriveX = robot.leftDriveStick.getXWithDeadband(true);
        double leftDriveY = robot.leftDriveStick.getYWithDeadband(true);
        double rightDriveY = robot.rightDriveStick.getYWithDeadband(true);
        double rightTwist = robot.rightDriveStick.getTwistWithDeadband(true);

        robot.updateDashboard(RunMode.TELEOP_MODE);
        robot.announceSafety();

        if (robot.pixy != null && robot.pixy.isEnabled())
        {
            // Force update of LEDs
            robot.pixy.getTargetInfo();
        }

        // Give drivers control only if auto deploy not active, or auto cancelled. AutoDeploy
        // is cancelled only by operator or completion. Other autos can be cancelled by driver moving the joystick.
        if (!robot.autoDeploy.isActive() && (shouldCancelAuto(leftDriveX, leftDriveY, rightDriveY, rightTwist) || !robot
            .isAutoActive()))
        {
            // Cancel any autos running
            robot.cancelAllAuto();
            // TODO: Test if this works
            if (elevatorPower != robot.elevator.getPower())
            {
                robot.elevator.setPower(elevatorPower);
            }
            //
            // DriveBase operation.
            //
            switch (driveMode)
            {
                case TANK_MODE:
                    double leftPower = leftDriveY;
                    double rightPower = rightDriveY;
                    switch (driveSpeed)
                    {
                        case SLOW:
                            leftPower *= RobotInfo.DRIVE_SLOW_YSCALE;
                            rightPower *= RobotInfo.DRIVE_SLOW_YSCALE;
                            break;

                        case MEDIUM:
                            leftPower *= RobotInfo.DRIVE_MEDIUM_YSCALE;
                            rightPower *= RobotInfo.DRIVE_MEDIUM_YSCALE;
                            break;

                        case FAST:
                            leftPower *= RobotInfo.DRIVE_FAST_YSCALE;
                            rightPower *= RobotInfo.DRIVE_FAST_YSCALE;
                            break;
                    }
                    robot.driveBase.tankDrive(leftPower, rightPower, robot.driveInverted);
                    break;

                case ARCADE_MODE:
                    double drivePower = rightDriveY;
                    double turnPower = rightTwist;
                    switch (driveSpeed)
                    {
                        case SLOW:
                            drivePower *= RobotInfo.DRIVE_SLOW_YSCALE;
                            turnPower *= RobotInfo.DRIVE_SLOW_TURNSCALE;
                            break;

                        case MEDIUM:
                            drivePower *= RobotInfo.DRIVE_MEDIUM_YSCALE;
                            turnPower *= RobotInfo.DRIVE_MEDIUM_TURNSCALE;
                            break;

                        case FAST:
                            drivePower *= RobotInfo.DRIVE_FAST_YSCALE;
                            turnPower *= RobotInfo.DRIVE_FAST_TURNSCALE;
                            break;
                    }
                    robot.driveBase.arcadeDrive(drivePower, turnPower, robot.driveInverted);
                    break;

                case MECANUM_MODE:
                    double x = leftDriveX;
                    double y = rightDriveY;
                    double rot = rightTwist;
                    switch (driveSpeed)
                    {
                        case SLOW:
                            x *= RobotInfo.DRIVE_SLOW_XSCALE;
                            y *= RobotInfo.DRIVE_SLOW_YSCALE;
                            rot *= RobotInfo.DRIVE_SLOW_TURNSCALE;
                            break;

                        case MEDIUM:
                            x *= RobotInfo.DRIVE_MEDIUM_XSCALE;
                            y *= RobotInfo.DRIVE_MEDIUM_YSCALE;
                            rot *= RobotInfo.DRIVE_MEDIUM_TURNSCALE;
                            break;

                        case FAST:
                            x *= RobotInfo.DRIVE_FAST_XSCALE;
                            y *= RobotInfo.DRIVE_FAST_YSCALE;
                            rot *= RobotInfo.DRIVE_FAST_TURNSCALE;
                            break;
                    }
                    robot.driveBase.holonomicDrive(x, y, rot, robot.driveInverted);
                    break;
            }
        }
    } // runPeriodic

    private boolean shouldCancelAuto(double... joystickValues)
    {
        return Arrays.stream(joystickValues).anyMatch(d -> d != 0.0);
    }

    private void setAllManualOverrideEnabled(boolean enabled)
    {
        robot.elevator.setManualOverrideEnabled(enabled);
        robot.pickup.setManualOverrideEnabled(enabled);
    }

    @Override
    public void runContinuous(double elapsedTime)
    {
        if (Robot.USE_RASPI_VISION)
        {
            HalDashboard.putBoolean("Status/TapeDetected", robot.vision.getAveragePose(5, true) != null);
        }
        HalDashboard.putBoolean("Status/CargoDetected", robot.pickup.cargoDetected());
        HalDashboard.putString("Status/DriveSpeed", driveSpeed.toString());

        if (DEBUG_LOOP_TIME)
        {
            loopTimeCounter.update();
            robot.dashboard
                .displayPrintf(1, "Period: %.3f/%.3f/%3f, Frequency: %.2f/%.2f/%.2f", loopTimeCounter.getMinPeriod(),
                    loopTimeCounter.getPeriod(), loopTimeCounter.getMaxPeriod(), loopTimeCounter.getMinFrequency(),
                    loopTimeCounter.getFrequency(), loopTimeCounter.getMaxFrequency());
        }
    } // runContinuous

    //
    // Implements TrcJoystick.ButtonHandler.
    //
    public void leftDriveStickButtonEvent(int button, boolean pressed)
    {
        boolean isAutoActive = robot.isAutoActive();
        robot.dashboard
            .displayPrintf(8, " LeftDriveStick: button=0x%04x %s, auto=%b", button, pressed ? "pressed" : "released",
                isAutoActive);
        if (isAutoActive)
        {
            return; // Auto can ony be cancelled by operator
        }

        switch (button)
        {
            case FrcJoystick.LOGITECH_TRIGGER:
                robot.driveInverted = pressed;
                robot.setHalfBrakeModeEnabled(true, robot.driveInverted);
                break;

            case FrcJoystick.LOGITECH_BUTTON2:
                break;

            case FrcJoystick.LOGITECH_BUTTON3:
                driveSpeed = pressed ? DriveSpeed.FAST : DriveSpeed.MEDIUM;
                break;

            case FrcJoystick.LOGITECH_BUTTON4:
                break;

            case FrcJoystick.LOGITECH_BUTTON5:
                break;

            case FrcJoystick.LOGITECH_BUTTON6:
                break;

            case FrcJoystick.LOGITECH_BUTTON7:
                break;

            case FrcJoystick.LOGITECH_BUTTON8:
                break;

            case FrcJoystick.LOGITECH_BUTTON9:
                break;

            case FrcJoystick.LOGITECH_BUTTON10:
                break;

            case FrcJoystick.LOGITECH_BUTTON11:
                break;

            case FrcJoystick.LOGITECH_BUTTON12:
                break;
        }
    } // leftDriveStickButtonEvent

    public void rightDriveStickButtonEvent(int button, boolean pressed)
    {
        boolean isAutoActive = robot.isAutoActive();
        robot.dashboard
            .displayPrintf(8, "RightDriveStick: button=0x%04x %s, auto=%b", button, pressed ? "pressed" : "released",
                isAutoActive);
        if (isAutoActive)
        {
            return; // Auto can only be cancelled by operator
        }

        switch (button)
        {
            case FrcJoystick.SIDEWINDER_TRIGGER:
                driveSpeed = pressed ? DriveSpeed.SLOW : DriveSpeed.MEDIUM;
                break;

            case FrcJoystick.SIDEWINDER_BUTTON2:
                break;

            case FrcJoystick.SIDEWINDER_BUTTON3:
                break;

            case FrcJoystick.SIDEWINDER_BUTTON4:
                break;

            case FrcJoystick.SIDEWINDER_BUTTON5:
                if (pressed)
                {
                    gyroAssist = !gyroAssist;
                    if (gyroAssist)
                    {
                        robot.driveBase
                            .enableGyroAssist(RobotInfo.DRIVE_MAX_ROTATION_RATE, RobotInfo.DRIVE_GYRO_ASSIST_KP);
                    }
                    else
                    {
                        robot.driveBase.disableGyroAssist();
                    }
                }
                break;

            case FrcJoystick.SIDEWINDER_BUTTON6:
                break;

            case FrcJoystick.SIDEWINDER_BUTTON7:
                break;

            case FrcJoystick.SIDEWINDER_BUTTON8:
                break;

            case FrcJoystick.SIDEWINDER_BUTTON9:
                break;
        }
    } // rightDriveStickButtonEvent

    public void operatorStickButtonEvent(int button, boolean pressed)
    {
        boolean isAutoActive = robot.isAutoActive();
        robot.dashboard
            .displayPrintf(8, "  OperatorStick: button=0x%04x %s, auto=%b", button, pressed ? "pressed" : "released",
                isAutoActive);
        if (isAutoActive)
        {
            return;
        }

        switch (button)
        {
            case FrcJoystick.LOGITECH_TRIGGER:
                if (pressed)
                {
                    robot.pickup.pickupCargo(null);
                }
                else
                {
                    robot.pickup.cancel();
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON2:
                if (pressed)
                {
                    robot.pickup.extendHatchDeployer();
                }
                else
                {
                    robot.pickup.retractHatchDeployer();
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON3:
                if (pressed)
                {
                    robot.pickup.deployCargo(null);
                }
                else
                {
                    robot.pickup.cancel();
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON4:
                robot.elevator.setManualOverrideEnabled(pressed);
                break;

            case FrcJoystick.LOGITECH_BUTTON5:
                robot.pickup.setManualOverrideEnabled(pressed);
                break;

            case FrcJoystick.LOGITECH_BUTTON6:
                robot.pickup.setPitchPower(pressed ? -0.75 : 0.0);
                break;

            case FrcJoystick.LOGITECH_BUTTON7:
                robot.pickup.setPitchPower(pressed ? 0.6 : 0.0);
                break;

            case FrcJoystick.LOGITECH_BUTTON8:
                break;

            case FrcJoystick.LOGITECH_BUTTON9:
                setAllManualOverrideEnabled(pressed);
                break;

            case FrcJoystick.LOGITECH_BUTTON10:
                if (pressed)
                {
                    robot.pickup.setPickupAngle(RobotInfo.PICKUP_GROUND_CARGO_POS);
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON11:
                if (pressed)
                {
                    robot.pickup.setPickupAngle(RobotInfo.PICKUP_HATCH_PICKUP_POS);
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON12:
                break;
        }
    } // operatorStickButtonEvent

    private void setElevatorHeight(TaskAutoDeploy.DeployLevel level)
    {
        boolean cargo = robot.pickup.cargoDetected();
        if (cargo)
        {
            robot.elevator.setPosition(RobotInfo.ELEVATOR_CARGO_ROCKET_POSITIONS[level.getIndex()]);
        }
        else
        {
            robot.elevator.setPosition(RobotInfo.ELEVATOR_HATCH_ROCKET_POSITIONS[level.getIndex()]);
        }
    }

    public void buttonPanelButtonEvent(int button, boolean pressed)
    {
        boolean isAutoActive = robot.isAutoActive();
        robot.dashboard
            .displayPrintf(8, "  ButtonPanel: button=0x%04x %s, auto=%b", button, pressed ? "pressed" : "released",
                isAutoActive);

        if (isAutoActive && (pressed || (button != FrcJoystick.PANEL_BUTTON1 && button != FrcJoystick.PANEL_BUTTON2
            && button != FrcJoystick.PANEL_BUTTON3 && button != FrcJoystick.PANEL_BUTTON4)))
        {
            return;
        }

        switch (button)
        {
            case FrcJoystick.PANEL_BUTTON1:
                if (Robot.USE_RASPI_VISION)
                {
                    if (pressed)
                    {
                        robot.autoDeploy.start(TaskAutoDeploy.DeployLevel.HIGH);
                    }
                    else
                    {
                        robot.autoDeploy.cancel();
                    }
                }
                break;

            case FrcJoystick.PANEL_BUTTON2:
                if (Robot.USE_RASPI_VISION)
                {
                    if (pressed)
                    {
                        robot.autoDeploy.start(TaskAutoDeploy.DeployLevel.MEDIUM);
                    }
                    else
                    {
                        robot.autoDeploy.cancel();
                    }
                }
                break;

            case FrcJoystick.PANEL_BUTTON3:
                if (Robot.USE_RASPI_VISION)
                {
                    if (pressed)
                    {
                        robot.autoDeploy.start(TaskAutoDeploy.DeployLevel.LOW);
                    }
                    else
                    {
                        robot.autoDeploy.cancel();
                    }
                }
                break;

            case FrcJoystick.PANEL_BUTTON4:
                if (Robot.USE_RASPI_VISION)
                {
                    if (pressed)
                    {
                        robot.autoDeploy.start(TaskAutoDeploy.DeployLevel.LOW, TaskAutoDeploy.DeployType.PICKUP_HATCH);
                    }
                    else
                    {
                        robot.autoDeploy.cancel();
                    }
                }
                break;

            case FrcJoystick.PANEL_BUTTON5:
                if (pressed)
                {
                    //setElevatorHeight(TaskAutoDeploy.DeployLevel.LOW);
                }
                break;

            case FrcJoystick.PANEL_BUTTON6:
                if (pressed)
                {
                    //setElevatorHeight(TaskAutoDeploy.DeployLevel.MEDIUM);
                }
                break;

            case FrcJoystick.PANEL_BUTTON7:
                if (pressed)
                {
                    //setElevatorHeight(TaskAutoDeploy.DeployLevel.HIGH);
                }
                break;

            case FrcJoystick.PANEL_BUTTON8:
                break;

            case FrcJoystick.PANEL_BUTTON9:
                break;

            case FrcJoystick.PANEL_BUTTON10:
                break;
        }
    } // operatorStickButtonEvent
} // class FrcTeleOp
