/*
 * Copyright (c) 2017 Titan Robotics Club (http://www.titanrobotics.com)
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

import edu.wpi.first.wpilibj.Relay.Value;
import frclib.FrcJoystick;
import hallib.HalDashboard;
import trclib.TrcRobot;

public class FrcTeleOp implements TrcRobot.RobotMode, FrcJoystick.ButtonHandler
{
    private enum DriveMode
    {
        MECANUM_MODE,
        ARCADE_MODE,
        TANK_MODE
    }   // enum DriveMode

    protected Robot robot;

    //
    // Input subsystem.
    //
    private FrcJoystick leftDriveStick;
    private FrcJoystick rightDriveStick;
    private FrcJoystick operatorStick;
    private CmdVisionGearDeploy cmdVisionDeploy;
    private CmdWaltzTurn cmdWaltzTurn;
    private String message = null;

    private boolean slowDriveOverride = false;
    private DriveMode driveMode = DriveMode.MECANUM_MODE;

    private boolean driveInverted = false;
    private boolean flashLightsOn = false;
    private boolean visionAssistOn = false;
    private boolean waltzTurnOn = false;

    public FrcTeleOp(Robot robot)
    {
        this.robot = robot;
        //
        // Input subsystem.
        //
        leftDriveStick = new FrcJoystick("leftDriveStick", RobotInfo.JSPORT_LEFT_DRIVESTICK, this);
        leftDriveStick.setYInverted(true);

        rightDriveStick = new FrcJoystick("rightDriveStick", RobotInfo.JSPORT_RIGHT_DRIVESTICK, this);
        rightDriveStick.setYInverted(true);

        operatorStick = new FrcJoystick("operatorStick", RobotInfo.JSPORT_OPERATORSTICK, this);
        operatorStick.setYInverted(true);

        cmdVisionDeploy = new CmdVisionGearDeploy(robot);
        cmdWaltzTurn = new CmdWaltzTurn(robot);
        message = HalDashboard.getString("Message", "Hello, Titans!");
    }   // FrcTeleOp

    //
    // Implements TrcRobot.RunMode interface.
    //

    @Override
    public void startMode()
    {
        HalDashboard.getInstance().clearDisplay();
        robot.setVisionEnabled(true);

        robot.driveBase.resetPosition();
        robot.targetHeading = 0.0;
        robot.winch.clearState();

        robot.encoderXPidCtrl.setOutputRange(-1.0, 1.0);
        robot.encoderYPidCtrl.setOutputRange(-1.0, 1.0);
        robot.gyroTurnPidCtrl.setOutputRange(-1.0, 1.0);
        robot.sonarDrivePidCtrl.setOutputRange(-1.0, 1.0);
        robot.visionTurnPidCtrl.setOutputRange(-1.0, 1.0);
    }   // startMode

    @Override
    public void stopMode()
    {
        robot.setVisionEnabled(false);
    }   // stopMode

    @Override
    public void runPeriodic(double elapsedTime)
    {
        if (!visionAssistOn && !waltzTurnOn)
        {
            //
            // DriveBase operation.
            //
            switch (driveMode)
            {
                case TANK_MODE:
                    double leftPower = leftDriveStick.getYWithDeadband(true);
                    double rightPower = rightDriveStick.getYWithDeadband(true);
                    if (slowDriveOverride)
                    {
                        leftPower /= RobotInfo.DRIVE_SLOW_YSCALE;
                        rightPower /= RobotInfo.DRIVE_SLOW_YSCALE;
                    }
                    robot.driveBase.tankDrive(leftPower, rightPower, driveInverted);
                    break;

                case ARCADE_MODE:
                    double drivePower = rightDriveStick.getYWithDeadband(true);
                    double turnPower = rightDriveStick.getTwistWithDeadband(true);
                    if (slowDriveOverride)
                    {
                        drivePower /= RobotInfo.DRIVE_SLOW_YSCALE;
                        turnPower /= RobotInfo.DRIVE_SLOW_TURNSCALE;
                    }
                    robot.driveBase.arcadeDrive(drivePower, turnPower, driveInverted);
                    break;

                default:
                case MECANUM_MODE:
                    double x = leftDriveStick.getXWithDeadband(true);
                    double y = rightDriveStick.getYWithDeadband(true);
                    double rot = rightDriveStick.getTwistWithDeadband(true);
                    if (slowDriveOverride)
                    {
                        x /= RobotInfo.DRIVE_SLOW_XSCALE;
                        y /= RobotInfo.DRIVE_SLOW_YSCALE;
                        rot /= RobotInfo.DRIVE_SLOW_TURNSCALE;
                    }
                    robot.driveBase.mecanumDrive_Cartesian(x, y, rot, driveInverted);
                    break;
            }

            double winchPower = operatorStick.getYWithDeadband(true);
            robot.winch.setPower(winchPower);
        }

        robot.updateDashboard();
    }   // runPeriodic

    @Override
    public void runContinuous(double elapsedTime)
    {
        if (waltzTurnOn)
        {
            if (cmdWaltzTurn.cmdPeriodic(elapsedTime))
            {
                waltzTurnOn = false;
            }
        }
        else if (visionAssistOn)
        {
            if (cmdVisionDeploy.cmdPeriodic(elapsedTime))
            {
                visionAssistOn = false;
            }
        }
    }   // runContinuous

    //
    // Implements TrcJoystick.ButtonHandler.
    //

    @Override
    public void joystickButtonEvent(FrcJoystick joystick, int button, boolean pressed)
    {
        if (joystick == leftDriveStick)
        {
            switch (button)
            {
                case FrcJoystick.LOGITECH_TRIGGER:
                    slowDriveOverride = pressed;
                    break;

                case FrcJoystick.LOGITECH_BUTTON2:
                    if (pressed && robot.tts != null)
                    {
                        robot.tts.speak("Kevin, drop the rope!");
                    }
                    break;

                case FrcJoystick.LOGITECH_BUTTON3:
                    if (pressed && robot.tts != null)
                    {
                        robot.tts.speak("Kevin, spin the rotor!");
                    }
                    break;

                case FrcJoystick.LOGITECH_BUTTON4:
                    if (pressed && robot.tts != null)
                    {
                        robot.tts.speak("Kevin, pick up the gear!");
                    }
                    break;

                case FrcJoystick.LOGITECH_BUTTON5:
                    if (pressed && robot.tts != null)
                    {
                        robot.tts.speak("Kevin, pick up the free gear!");
                    }
                    break;

                case FrcJoystick.LOGITECH_BUTTON6:
                    if (pressed && robot.tts != null && message != null)
                    {
                        robot.tts.speak(message);
                    }
                    break;

                case FrcJoystick.LOGITECH_BUTTON7:
                    break;

                case FrcJoystick.LOGITECH_BUTTON8:
                    break;

                case FrcJoystick.LOGITECH_BUTTON9:
                    break;

                case FrcJoystick.LOGITECH_BUTTON10:
                    if (pressed)
                    {
                        cmdVisionDeploy.start();
                    }
                    else
                    {
                        cmdVisionDeploy.stop();
                    }
                    visionAssistOn = pressed;
                    break;

                case FrcJoystick.LOGITECH_BUTTON11:
                    break;

                case FrcJoystick.LOGITECH_BUTTON12:
                    break;
            }
        }
        else if (joystick == rightDriveStick)
        {
            switch (button)
            {
                case FrcJoystick.SIDEWINDER_TRIGGER:
                    if (pressed)
                    {
                        driveInverted = !driveInverted;
                    }
                    break;

                case FrcJoystick.SIDEWINDER_BUTTON2:
                    if (pressed)
                    {
                        cmdWaltzTurn.stop();
                        waltzTurnOn = false;
                    }
                    break;

                case FrcJoystick.SIDEWINDER_BUTTON3:
                    if (pressed)
                    {
                        cmdWaltzTurn.setClockwiseTurn(true, driveInverted);
                        cmdWaltzTurn.start();
                        driveInverted = !driveInverted;
                        waltzTurnOn = true;
                    }
                    break;

                case FrcJoystick.SIDEWINDER_BUTTON4:
                    if (pressed)
                    {
                        cmdWaltzTurn.setClockwiseTurn(false, driveInverted);
                        cmdWaltzTurn.start();
                        driveInverted = !driveInverted;
                        waltzTurnOn = true;
                    }
                    break;

                case FrcJoystick.SIDEWINDER_BUTTON5:
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
        }
        else if (joystick == operatorStick)
        {
            switch (button)
            {
                case FrcJoystick.LOGITECH_TRIGGER:
                    break;

                case FrcJoystick.LOGITECH_BUTTON2:
                    //
                    // Arm down.
                    //
                    if (pressed && !visionAssistOn)
                    {
                        robot.gearPickup.lowerArm();
                    }
                    break;

                case FrcJoystick.LOGITECH_BUTTON3:
                    //
                    // Arm up.
                    //
                    if (pressed && !visionAssistOn)
                    {
                        robot.gearPickup.liftArm();
                    }
                    break;

                case FrcJoystick.LOGITECH_BUTTON4:
                    //
                    // Claw open.
                    //
                    if (pressed && !visionAssistOn)
                    {
                        robot.gearPickup.openClaw();
                    }
                    break;

                case FrcJoystick.LOGITECH_BUTTON5:
                    //
                    // Claw close.
                    //
                    if (pressed && !visionAssistOn)
                    {
                        robot.gearPickup.closeClaw();
                    }
                    break;

                case FrcJoystick.LOGITECH_BUTTON6:
                    if (pressed && !visionAssistOn)
                    {
                        robot.mailbox.extend();
                    }
                    break;

                case FrcJoystick.LOGITECH_BUTTON7:
                    if (pressed && !visionAssistOn)
                    {
                        robot.mailbox.retract();
                    }
                    break;

                case FrcJoystick.LOGITECH_BUTTON8:
                    break;

                case FrcJoystick.LOGITECH_BUTTON9:
                    if (pressed)
                    {
                        flashLightsOn = !flashLightsOn;
                        robot.flashLightsPower.set(flashLightsOn? Value.kOn: Value.kOff);
                    }
                    break;

                case FrcJoystick.LOGITECH_BUTTON10:
                    robot.winch.setManualOverride(pressed);
                    break;

                case FrcJoystick.LOGITECH_BUTTON11:
                    break;

                case FrcJoystick.LOGITECH_BUTTON12:
                    break;
            }
        }
    }   // joystickButtonEvent

}   // class FrcTeleOp
