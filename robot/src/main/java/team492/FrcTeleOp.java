/*
 * Copyright (c) 2018 Titan Robotics Club (http://www.titanrobotics.com)
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
    private static final String moduleName = "FrcTeleOp";

    private enum DriveMode
    {
        MECANUM_MODE, ARCADE_MODE, TANK_MODE
    } // enum DriveMode

    protected Robot robot;

    private boolean slowDriveOverride = false;
    private DriveMode driveMode = DriveMode.MECANUM_MODE;
    private boolean driveInverted = false;
    private boolean gyroAssist = false;
    private int winchDirection = 0;

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
        robot.switchPanel.setButtonHandler(this::switchPanelButtonEvent);
    } // startMode

    @Override
    public void stopMode(RunMode prevMode, RunMode nextMode)
    {
    } // stopMode

    @Override
    public void runPeriodic(double elapsedTime)
    {
        double leftDriveX = robot.leftDriveStick.getXWithDeadband(true);
        double leftDriveY = robot.leftDriveStick.getYWithDeadband(true);
        double rightDriveY = robot.rightDriveStick.getYWithDeadband(true);

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
//                    double xForceOz = x * RobotInfo.MAX_WHEEL_FORCE_OZ;
//                    double yForceOz = y * RobotInfo.MAX_WHEEL_FORCE_OZ;
                robot.driveBase.holonomicDrive(x, y, rot, driveInverted);
//                    HalDashboard.putNumber("xForceOz", xForceOz);
//                    HalDashboard.putNumber("yForceOz", yForceOz);
                break;
        }

        double elevatorPower = robot.operatorStick.getYWithDeadband(true);
        robot.elevator.setPower(elevatorPower); // Pull joystick back -> move elevator up

        double winchPower = winchDirection*(1.0 - robot.operatorStick.getZ())/2.0;
        robot.winch.setPower(winchPower);

        robot.updateDashboard(RunMode.TELEOP_MODE);
        robot.announceSafety();
    } // runPeriodic

    @Override
    public void runContinuous(double elapsedTime)
    {
        final String funcName = moduleName + ".runContinuous";

        if (robot.elevator.elevator.isActive())
        {
            robot.elevator.elevatorPidCtrl.printPidInfo(robot.globalTracer, elapsedTime, robot.battery);
            robot.globalTracer.traceInfo(
                funcName, "elevatorLimitSwitches=%b/%b, talonCurrent=%.3f, pdpElevatorCurrent=%.3f",
                robot.elevator.elevatorMotor.isLowerLimitSwitchActive(),
                robot.elevator.elevatorMotor.isUpperLimitSwitchActive(),
                robot.elevator.elevatorMotor.motor.getOutputCurrent(),
                robot.pdp.getCurrent(RobotInfo.PDP_CHANNEL_ELEVATOR));
        }
    } // runContinuous

    //
    // Implements TrcJoystick.ButtonHandler.
    //

    public void leftDriveStickButtonEvent(int button, boolean pressed)
    {
        robot.dashboard.displayPrintf(8, " LeftDriveStick: button=0x%04x %s", button, pressed? "pressed": "released");

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
        robot.dashboard.displayPrintf(8, "RightDriveStick: button=0x%04x %s", button, pressed? "pressed": "released");

        switch (button)
        {
            case FrcJoystick.SIDEWINDER_TRIGGER:
                slowDriveOverride = pressed;
                break;

            case FrcJoystick.SIDEWINDER_BUTTON2:
                break;

            case FrcJoystick.SIDEWINDER_BUTTON3:
                if(pressed)
                {
                    robot.leftFlipper.timedExtend(RobotInfo.FLIPPER_EXTEND_PERIOD);
                }
                break;

            case FrcJoystick.SIDEWINDER_BUTTON4:
                if(pressed)
                {
                    robot.rightFlipper.timedExtend(RobotInfo.FLIPPER_EXTEND_PERIOD);
                }
                break;

            case FrcJoystick.SIDEWINDER_BUTTON5:
                if (pressed)
                {
                    gyroAssist = !gyroAssist;
                    if (gyroAssist)
                    {
                        robot.driveBase.enableGyroAssist(
                            RobotInfo.DRIVE_MAX_ROTATION_RATE, RobotInfo.DRIVE_GYRO_ASSIST_KP);
                        robot.ledIndicator.indicateGyroAssistOn();
                    }
                    else
                    {
                        robot.driveBase.disableGyroAssist();
                        robot.ledIndicator.indicateGyroAssistOff();
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
        robot.dashboard.displayPrintf(8, "  OperatorStick: button=0x%04x %s", button, pressed? "pressed": "released");

        switch (button)
        {
            case FrcJoystick.LOGITECH_TRIGGER:
                if (pressed)
                {
                    robot.cubePickup.grabCube(RobotInfo.PICKUP_TELEOP_POWER, null);
                }
                else
                {
                    robot.cubePickup.stopPickup();
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON2:
                robot.elevator.setManualOverride(pressed);
//                if (pressed)
//                {
//                    robot.cubePickup.grabCube(0.5);
//                }
//                else
//                {
//                    robot.cubePickup.grabCube(0.0);
//                }
                break;

            case FrcJoystick.LOGITECH_BUTTON3:
                if (pressed)
                {
                    robot.cubePickup.dropCube(RobotInfo.PICKUP_TELEOP_POWER);
                }
                else
                {
                    robot.cubePickup.stopPickup();
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON4:
                //
                // Cube Claw open.
                //
                if (pressed)
                {
                    robot.cubePickup.openClaw();
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON5:
                //
                // Cube Claw close.
                //
                if (pressed)
                {
                    robot.cubePickup.closeClaw();
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON6:
                //
                // Cube Deployer down.
                //
                if (pressed)
                {
                    robot.cubePickup.raisePickup();
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON7:
                //
                // Cube Deployer up.
                //
                if (pressed)
                {
                    robot.cubePickup.deployPickup();
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON8:
                if(pressed)
                {
                    robot.cubePickup.turtle();
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON9:
                if(pressed)
                {
                    robot.cubePickup.prepareForPickup();
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON10:
                winchDirection = pressed? -1: 0;
                if(pressed)
                {
                    robot.cubePickup.turtle();
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON11:
                winchDirection = pressed? 1: 0;
                if(pressed)
                {
                    robot.cubePickup.turtle();
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON12:
                break;
        }
    } // operatorStickButtonEvent

    public void buttonPanelButtonEvent(int button, boolean pressed)
    {
        robot.dashboard.displayPrintf(8, "  ButtonPanel: button=0x%04x %s", button, pressed? "pressed": "released");

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
                if (pressed && robot.visionPidDrive != null)
                {
                    robot.visionPidDrive.setTarget(0.0, 0.0, 0.0, false, null);
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON11:
                break;

            case FrcJoystick.LOGITECH_BUTTON12:
                break;
        }
    }

    public void switchPanelButtonEvent(int button, boolean pressed)
    {
        robot.dashboard.displayPrintf(8, "  SwitchPanel: button=0x%04x %s", button, pressed? "pressed": "released");

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
                driveInverted = pressed;
                break;

            case FrcJoystick.LOGITECH_BUTTON11:
                break;

            case FrcJoystick.LOGITECH_BUTTON12:
                break;
        }
    } // switchPanelButtonEvent

} // class FrcTeleOp
