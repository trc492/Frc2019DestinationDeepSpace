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
import frclib.FrcRemoteVisionProcessor;
import hallib.HalDashboard;
import trclib.TrcLoopPerformanceMonitor;
import trclib.TrcRobot;
import trclib.TrcRobot.RunMode;

public class FrcTeleOp implements TrcRobot.RobotMode
{
    public static final boolean DEBUG_LOOP_TIME = true;

    protected Robot robot;

    private enum DriveSpeed
    {
        SLOW, MEDIUM, FAST
    }

    private DriveSpeed driveSpeed = DriveSpeed.MEDIUM;
    private boolean gyroAssist = false;
    private double lastElevatorPower;
    private double lastActuatorPower;
    private TrcLoopPerformanceMonitor loopPerformanceMonitor;

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

        // CodeReview: obsolete code???
        robot.driveClimberWheels = false;
        robot.actuatorEnabled = false;

        driveSpeed = DriveSpeed.MEDIUM;

        if (Robot.USE_VISION_TARGETING)
        {
            robot.vision.setRingLightEnabled(true);
        }

        if (DEBUG_LOOP_TIME)
        {
            loopPerformanceMonitor = new TrcLoopPerformanceMonitor("TeleOpLoop", 1.0);
        }

        lastElevatorPower = 0.0;
        lastActuatorPower = 0.0;
    } // startMode

    @Override
    public void stopMode(RunMode prevMode, RunMode nextMode)
    {
    } // stopMode

    private void showStatus()
    {
        if (Robot.USE_VISION_TARGETING)
        {
            FrcRemoteVisionProcessor.RelativePose pose = robot.vision.getLastPose();
            HalDashboard.putBoolean("Status/TapeDetected", pose != null);
            if (pose == null)
            {
                robot.indicator.signalNoVisionDetected();
            }
            else if (pose.x > RobotInfo.CAMERA_CENTERED_THRESHOLD)
            {
                robot.indicator.signalVisionRight();
            }
            else if (pose.x < -RobotInfo.CAMERA_CENTERED_THRESHOLD)
            {
                robot.indicator.signalVisionLeft();
            }
            else
            {
                robot.indicator.signalVisionCentered();
            }
        }
        boolean cargoDetected = robot.pickup.cargoDetected();
        HalDashboard.putBoolean("Status/CargoDetected", cargoDetected);
        robot.indicator.signalCargoDetected(cargoDetected);

        HalDashboard.putString("Status/DriveSpeed", driveSpeed.toString());
    }

    @Override
    public void runPeriodic(double elapsedTime)
    {
        showStatus();

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

        if (elevatorPower != lastElevatorPower)
        {
            robot.elevator.setPower(elevatorPower);
            lastElevatorPower = elevatorPower;
        }

        // CodeReview: obsolete code???
        double actuatorPower = robot.actuatorEnabled ? robot.operatorStick.getTwistWithDeadband(true) : 0.0;
        if (actuatorPower != lastActuatorPower)
        {
            robot.climber.setActuatorPower(actuatorPower);
            lastActuatorPower = actuatorPower;
        }

        //
        // DriveBase operation.
        //
        // CodeReview: obsolete code???
        if (robot.driveClimberWheels)
        {
            // CodeReview: should multiply with a scale factor in order to sync the mecanum speed.
            robot.climber.setWheelPower(rightDriveY);
        }
        else
        {
            robot.climber.setWheelPower(0.0);
            switch (robot.driveMode)
            {
                case HOLONOMIC_MODE:
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

                    if (robot.visionPidDrive == null || !robot.visionPidDrive.isActive())
                    {
                        robot.driveBase.holonomicDrive(x, y, rot, robot.driveInverted);
                    }
                    break;

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

                    if (robot.visionPidDrive == null || !robot.visionPidDrive.isActive())
                    {
                        robot.driveBase.tankDrive(leftPower, rightPower, robot.driveInverted);
                    }
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

                    if (robot.visionPidDrive == null || !robot.visionPidDrive.isActive())
                    {
                        robot.driveBase.arcadeDrive(drivePower, turnPower, robot.driveInverted);
                    }
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
        if (DEBUG_LOOP_TIME)
        {
            loopPerformanceMonitor.update();
            robot.dashboard.displayPrintf(1, "Period: %.3f/%.3f/%3f, Frequency: %.2f/%.2f/%.2f",
                loopPerformanceMonitor.getMinPeriod(), loopPerformanceMonitor.getAveragePeriod(),
                loopPerformanceMonitor.getMaxPeriod(), loopPerformanceMonitor.getMinFrequency(),
                loopPerformanceMonitor.getAverageFrequency(), loopPerformanceMonitor.getMaxFrequency());
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

        switch (button)
        {
            case FrcJoystick.LOGITECH_TRIGGER:
                robot.driveInverted = pressed;
                robot.setHalfBrakeModeEnabled(true);
                if (pressed)
                {
                    robot.autoHeadingAlign.cancel();
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON2:
                break;

            case FrcJoystick.LOGITECH_BUTTON3:
                driveSpeed = pressed ? DriveSpeed.FAST : DriveSpeed.MEDIUM;
                break;

            case FrcJoystick.LOGITECH_BUTTON4:
                if (pressed)
                {
                    robot.autoHeadingAlign.start(true);
                }
                else
                {
                    robot.autoHeadingAlign.cancel();
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON5:
                if (pressed)
                {
                    robot.autoHeadingAlign.start(false);
                }
                else
                {
                    robot.autoHeadingAlign.cancel();
                }
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
        robot.dashboard
            .displayPrintf(8, "RightDriveStick: button=0x%04x %s, auto=%b", button, pressed ? "pressed" : "released",
                robot.isAutoActive());

        switch (button)
        {
            case FrcJoystick.SIDEWINDER_TRIGGER:
                driveSpeed = pressed ? DriveSpeed.SLOW : DriveSpeed.MEDIUM;
                break;

            case FrcJoystick.SIDEWINDER_BUTTON2:
                robot.vision.setRingLightEnabled(!pressed);
                break;

            case FrcJoystick.SIDEWINDER_BUTTON3:
                /*
                if (pressed)
                {
                    robot.autoAlign.start(false);
                }
                else
                {
                    robot.autoAlign.cancel();
                }
                */
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
                // CodeReview: obsolete code???
                robot.driveClimberWheels = pressed;
                if (!pressed)
                {
                    robot.climber.setWheelPower(0.0);
                }
                break;

            case FrcJoystick.SIDEWINDER_BUTTON9:
                break;
        }
    } // rightDriveStickButtonEvent

    public void operatorStickButtonEvent(int button, boolean pressed)
    {
        robot.dashboard.displayPrintf(8, "  OperatorStick: button=0x%04x %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            case FrcJoystick.LOGITECH_TRIGGER:
                if (pressed)
                {
                    robot.pickup.pickupCargo(null);
                    robot.indicator.enablePickupPriorities();
                }
                else
                {
                    robot.pickup.cancel();
                    robot.indicator.enableNormalPriorities();
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON2:
                if (pressed)
                {
                    robot.pickup.retractHatchGrabber();
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
                if (pressed)
                {
                    robot.pickup.extendHatchGrabber();
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON5:
                if (pressed)
                {
                    robot.pickup.retractHatchGrabber();
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON6:
                robot.pickup.setPitchPower(pressed ? -0.75 : 0.0);
                break;

            case FrcJoystick.LOGITECH_BUTTON7:
                robot.pickup.setPitchPower(pressed ? 0.6 : 0.0);
                break;

            case FrcJoystick.LOGITECH_BUTTON8:
                if (pressed)
                {
                    robot.elevator.setPosition(RobotInfo.ELEVATOR_POS_CARGO_SHIP);
                    robot.pickup.setPickupAngle(RobotInfo.PICKUP_CARGO_SHIP_POS);
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON9:
                // Ignore this button if the switch is flipped.
                // NOTE: If the switch binding changes, this WILL break!
                if (!robot.switchPanel.getRawButton(FrcJoystick.PANEL_BUTTON_GREEN1))
                {
                    setAllManualOverrideEnabled(pressed);
                }
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

    public void buttonPanelButtonEvent(int button, boolean pressed)
    {
        robot.dashboard.displayPrintf(8, "  ButtonPanel: button=0x%04x %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            case FrcJoystick.PANEL_BUTTON_RED1:
                break;

            case FrcJoystick.PANEL_BUTTON_GREEN1:
                break;

            case FrcJoystick.PANEL_BUTTON_BLUE1:
                break;

            case FrcJoystick.PANEL_BUTTON_YELLOW1:
                break;

            case FrcJoystick.PANEL_BUTTON_WHITE1:
                if (pressed)
                {
                    setElevatorHeight(RobotInfo.DeployLevel.LOW);
                }
                break;

            case FrcJoystick.PANEL_BUTTON_RED2:
                if (pressed)
                {
                    setElevatorHeight(RobotInfo.DeployLevel.MEDIUM);
                }
                break;

            case FrcJoystick.PANEL_BUTTON_GREEN2:
                if (pressed)
                {
                    setElevatorHeight(RobotInfo.DeployLevel.HIGH);
                }
                break;

            case FrcJoystick.PANEL_BUTTON_BLUE2:
                break;

            case FrcJoystick.PANEL_BUTTON_YELLOW2:
                break;

            case FrcJoystick.PANEL_BUTTON_WHITE2:
                if (pressed)
                {
                    robot.climber.zeroCalibrateActuator();
                    if (!robot.climber.isActive())
                    {
                        robot.elevator.zeroCalibrate();
                        robot.pickup.zeroCalibrate();
                        robot.pickup.retractHatchGrabber();
                        robot.pickup.retractHatchDeployer();
                    }
                }
                break;
        }
    } // operatorStickButtonEvent

    public void switchPanelButtonEvent(int button, boolean pressed)
    {
        robot.dashboard.displayPrintf(8, "  SwitchPanel: button=0x%04x %s", button, pressed ? "pressed" : "released");
        switch (button)
        {
            case FrcJoystick.PANEL_SWITCH_WHITE1:
                if (pressed)
                {
                    robot.elevator.setPosition(RobotInfo.ELEVATOR_POS_HATCH_PICKUP_GROUND);
                    robot.pickup.setPickupAngle(RobotInfo.PICKUP_MAX_POS);
                }
                break;

            case FrcJoystick.PANEL_SWITCH_RED1:
                setAllManualOverrideEnabled(pressed);
                break;

            case FrcJoystick.PANEL_SWITCH_GREEN1:
                break;

            case FrcJoystick.PANEL_SWITCH_BLUE1:
                break;

            case FrcJoystick.PANEL_SWITCH_YELLOW1:
                break;

            case FrcJoystick.PANEL_SWITCH_WHITE2:
                break;

            case FrcJoystick.PANEL_SWITCH_RED2:
                break;

            case FrcJoystick.PANEL_SWITCH_GREEN2:
                break;

            case FrcJoystick.PANEL_SWITCH_BLUE2:
                if (pressed)
                {
                    robot.climber.climb();
                }
                else
                {
                    robot.climber.cancel();
                }
                break;

            case FrcJoystick.PANEL_SWITCH_YELLOW2:
                //CodeReview: obsolete code???
                robot.climber.cancel();
                robot.actuatorEnabled = pressed;
                if (!pressed)
                {
                    robot.climber.setActuatorPower(0.0);
                }
                break;
        }
    }

    private void setElevatorHeight(RobotInfo.DeployLevel level)
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
} // class FrcTeleOp
