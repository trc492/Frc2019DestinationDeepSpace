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
import trclib.TrcRobot;
import trclib.TrcRobot.RunMode;

public class FrcTeleOp implements TrcRobot.RobotMode
{
    private enum DriveMode
    {
        MECANUM_MODE, ARCADE_MODE, TANK_MODE
    } // enum DriveMode

    protected Robot robot;

    private boolean slowDriveOverride = true;
    private DriveMode driveMode = DriveMode.MECANUM_MODE;
    private boolean driveInverted = false;
    private boolean gyroAssist = false;

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

        slowDriveOverride = true;
    } // startMode

    @Override
    public void stopMode(RunMode prevMode, RunMode nextMode)
    {
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

        // Give drivers control only if no auto active. Auto is cancelled only by operator or completion.
        if (!robot.isAutoActive())
        {
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
                    if (slowDriveOverride)
                    {
                        leftPower *= RobotInfo.DRIVE_SLOW_YSCALE;
                        rightPower *= RobotInfo.DRIVE_SLOW_YSCALE;
                    }
                    robot.driveBase.tankDrive(leftPower, rightPower, driveInverted);
                    break;

                case ARCADE_MODE:
                    double drivePower = rightDriveY;
                    double turnPower = rightTwist;
                    if (slowDriveOverride)
                    {
                        drivePower *= RobotInfo.DRIVE_SLOW_YSCALE;
                        turnPower *= RobotInfo.DRIVE_SLOW_TURNSCALE;
                    }
                    robot.driveBase.arcadeDrive(drivePower, turnPower, driveInverted);
                    break;

                case MECANUM_MODE:
                    double x = leftDriveX;
                    double y = rightDriveY;
                    double rot = rightTwist;
                    if (slowDriveOverride)
                    {
                        x *= RobotInfo.DRIVE_SLOW_XSCALE;
                        y *= RobotInfo.DRIVE_SLOW_YSCALE;
                        rot *= RobotInfo.DRIVE_SLOW_TURNSCALE;
                    }
                    robot.driveBase.holonomicDrive(x, y, rot, driveInverted);
                    break;
            }
        }
    } // runPeriodic

    private void setAllManualOverrideEnabled(boolean enabled)
    {
        robot.elevator.setManualOverrideEnabled(enabled);
        robot.pickup.setManualOverrideEnabled(enabled);
    }

    @Override
    public void runContinuous(double elapsedTime)
    {
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
                break;

            case FrcJoystick.LOGITECH_BUTTON2:
                break;

            case FrcJoystick.LOGITECH_BUTTON3:
                break;

            case FrcJoystick.LOGITECH_BUTTON4:
                break;

            case FrcJoystick.LOGITECH_BUTTON5:
                break;

            case FrcJoystick.LOGITECH_BUTTON6:
                break;

            case FrcJoystick.LOGITECH_BUTTON7:
                slowDriveOverride = !slowDriveOverride;
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
                robot.pickup.setPitchPower(pressed ? 0.6 : 0.0);
                break;

            case FrcJoystick.LOGITECH_BUTTON7:
                robot.pickup.setPitchPower(pressed ? -0.75 : 0.0);
                break;

            case FrcJoystick.LOGITECH_BUTTON8:
                break;

            case FrcJoystick.LOGITECH_BUTTON9:
                setAllManualOverrideEnabled(pressed);
                break;

            case FrcJoystick.LOGITECH_BUTTON10:
                if (pressed)
                {
                    robot.pickup.setPickupAngle(0.0);
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON11:
                if (pressed)
                {
                    robot.pickup.setPickupAngle(90.0);
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON12:
                break;
        }
    } // operatorStickButtonEvent

    public void buttonPanelButtonEvent(int button, boolean pressed)
    {
        boolean isAutoActive = robot.isAutoActive();
        robot.dashboard
            .displayPrintf(8, "  OperatorStick: button=0x%04x %s, auto=%b", button, pressed ? "pressed" : "released",
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
                break;

            case FrcJoystick.PANEL_BUTTON6:
                break;

            case FrcJoystick.PANEL_BUTTON7:
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
