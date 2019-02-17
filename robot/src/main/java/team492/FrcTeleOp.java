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

import java.util.Arrays;

public class FrcTeleOp implements TrcRobot.RobotMode
{
    private enum DriveMode
    {
        MECANUM_MODE, ARCADE_MODE, TANK_MODE
    } // enum DriveMode

    protected Robot robot;

    private boolean slowDriveOverride = false;
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

        robot.updateDashboard(RunMode.TELEOP_MODE);
        robot.announceSafety();

        if (robot.pixy != null && robot.pixy.isEnabled())
        {
            // Force update of LEDs
            robot.pixy.getTargetInfo();
        }

        if (shouldCancelAuto(leftDriveX, leftDriveY, rightDriveY) || !robot.isAutoActive())
        {
            robot.cancelAllAuto();
            robot.elevator.setPower(elevatorPower, false); // For debugging purposes, leave it false.
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
                        leftPower /= RobotInfo.DRIVE_SLOW_YSCALE;
                        rightPower /= RobotInfo.DRIVE_SLOW_YSCALE;
                    }
                    robot.driveBase.tankDrive(leftPower, rightPower, driveInverted);
                    break;

                case ARCADE_MODE:
                    double drivePower = rightDriveY;
                    double turnPower = robot.rightDriveStick.getTwistWithDeadband(true);
                    if (slowDriveOverride)
                    {
                        drivePower /= RobotInfo.DRIVE_SLOW_YSCALE;
                        turnPower /= RobotInfo.DRIVE_SLOW_TURNSCALE;
                    }
                    robot.driveBase.arcadeDrive(drivePower, turnPower, driveInverted);
                    break;

                case MECANUM_MODE:
                    double x = leftDriveX;
                    double y = rightDriveY;
                    double rot = robot.rightDriveStick.getTwistWithDeadband(true);
                    if (slowDriveOverride)
                    {
                        x /= RobotInfo.DRIVE_SLOW_XSCALE;
                        y /= RobotInfo.DRIVE_SLOW_YSCALE;
                        rot /= RobotInfo.DRIVE_SLOW_TURNSCALE;
                    }
                    robot.driveBase.holonomicDrive(x, y, rot, driveInverted);
                    break;
            }
        }
    } // runPeriodic

    private boolean shouldCancelAuto(double... joystickValues)
    {
        return Arrays.stream(joystickValues).anyMatch(d -> d != 0.0);
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
        robot.dashboard.displayPrintf(8, " LeftDriveStick: button=0x%04x %s", button, pressed ? "pressed" : "released");
        if (robot.isAutoActive())
        {
            return;
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
        robot.dashboard.displayPrintf(8, "RightDriveStick: button=0x%04x %s", button, pressed ? "pressed" : "released");
        if (robot.isAutoActive())
        {
            return;
        }

        switch (button)
        {
            case FrcJoystick.SIDEWINDER_TRIGGER:
                slowDriveOverride = pressed;
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
        robot.dashboard.displayPrintf(8, "  OperatorStick: button=0x%04x %s", button, pressed ? "pressed" : "released");
        if (robot.isAutoActive())
        {
            return;
        }

        switch (button)
        {
            case FrcJoystick.LOGITECH_TRIGGER:
//                if (pressed)
//                {
//                    robot.pickup.pickupCargo(null);
//                }
//                else
//                {
//                    robot.pickup.cancel();
//                }
                // TODO: Test if pickupCargo works, when the DIO gets connected
                robot.pickup.setPickupPower(pressed ? RobotInfo.PICKUP_CARGO_PICKUP_POWER : 0.0);
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
                break;

            case FrcJoystick.LOGITECH_BUTTON5:
                break;

            case FrcJoystick.LOGITECH_BUTTON6:
                robot.pickup.setPitchPower(pressed ? 0.6 : 0.0, false);
                break;

            case FrcJoystick.LOGITECH_BUTTON7:
                robot.pickup.setPitchPower(pressed ? -0.75 : 0.0, false);
                break;

            case FrcJoystick.LOGITECH_BUTTON8:
                if (pressed)
                {
                    robot.pickup.setPickupAngle(0.0);
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON9:
                if (pressed)
                {
                    robot.pickup.setPickupAngle(90.0);
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON10:
                if (pressed)
                {
                    // Lower right button on joystick base.
                    robot.autoAlignTarget.start(null);
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON11:
                break;

            case FrcJoystick.LOGITECH_BUTTON12:
                break;
        }
    } // operatorStickButtonEvent

    public void buttonPanelButtonEvent(int button, boolean pressed)
    {
        robot.dashboard.displayPrintf(8, "  OperatorStick: button=0x%04x %s", button, pressed ? "pressed" : "released");
        if (robot.isAutoActive())
        {
            return;
        }

        /*
        Button mappings:
        1 = rocket hatch high
        2 = rocket hatch med
        3 = rocket hatch low
        4 = rocket cargo high
        5 = rocket cargo med
        6 = rocket cargo low
        7 = ship cargo
        8 = ship hatch
        9 = pickup cargo
        10 = pickup hatch
         */
        // TODO: this could maybe be cut down by automatically detecting which gamepiece is being held
        switch (button)
        {
            case FrcJoystick.PANEL_BUTTON1:
                if (pressed)
                {
                    robot.autoDeploy
                        .start(RobotInfo.ELEVATOR_POS_HATCH_ROCKET_HIGH, TaskAutoDeploy.DeployType.HATCH, null);
                }
                break;

            case FrcJoystick.PANEL_BUTTON2:
                if (pressed)
                {
                    robot.autoDeploy
                        .start(RobotInfo.ELEVATOR_POS_HATCH_ROCKET_MED, TaskAutoDeploy.DeployType.HATCH, null);
                }
                break;

            case FrcJoystick.PANEL_BUTTON3:
                if (pressed)
                {
                    robot.autoDeploy
                        .start(RobotInfo.ELEVATOR_POS_HATCH_ROCKET_LOW, TaskAutoDeploy.DeployType.HATCH, null);
                }
                break;

            case FrcJoystick.PANEL_BUTTON4:
                if (pressed)
                {
                    robot.autoDeploy
                        .start(RobotInfo.ELEVATOR_POS_CARGO_ROCKET_HIGH, TaskAutoDeploy.DeployType.CARGO, null);
                }
                break;

            case FrcJoystick.PANEL_BUTTON5:
                if (pressed)
                {
                    robot.autoDeploy
                        .start(RobotInfo.ELEVATOR_POS_CARGO_ROCKET_MED, TaskAutoDeploy.DeployType.CARGO, null);
                }
                break;

            case FrcJoystick.PANEL_BUTTON6:
                if (pressed)
                {
                    robot.autoDeploy
                        .start(RobotInfo.ELEVATOR_POS_CARGO_ROCKET_LOW, TaskAutoDeploy.DeployType.CARGO, null);
                }
                break;

            case FrcJoystick.PANEL_BUTTON7:
                if (pressed)
                {
                    robot.autoDeploy.start(RobotInfo.ELEVATOR_POS_CARGO_SHIP, TaskAutoDeploy.DeployType.CARGO, null);
                }
                break;

            case FrcJoystick.PANEL_BUTTON8:
                if (pressed)
                {
                    robot.autoDeploy.start(RobotInfo.ELEVATOR_POS_HATCH_SHIP, TaskAutoDeploy.DeployType.HATCH, null);
                }
                break;

            case FrcJoystick.PANEL_BUTTON9:
                if (pressed)
                {
                    robot.autoDeploy
                        .start(RobotInfo.ELEVATOR_POS_CARGO_PICKUP, TaskAutoDeploy.DeployType.PICKUP_CARGO, null);
                }
                break;

            case FrcJoystick.PANEL_BUTTON10:
                if (pressed)
                {
                    robot.autoDeploy
                        .start(RobotInfo.ELEVATOR_POS_HATCH_PICKUP, TaskAutoDeploy.DeployType.PICKUP_HATCH, null);
                }
                break;
        }
    } // operatorStickButtonEvent
} // class FrcTeleOp
