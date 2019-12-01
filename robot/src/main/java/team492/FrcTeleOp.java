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

import edu.wpi.first.wpilibj.GenericHID;
import frclib.FrcJoystick;
import trclib.TrcLoopPerformanceMonitor;
import trclib.TrcRobot;
import trclib.TrcRobot.RunMode;

public class FrcTeleOp implements TrcRobot.RobotMode
{
    public static final boolean DEBUG_LOOP_TIME = true;
    public static final boolean USE_CONTROLLER = false;

    protected Robot robot;

    private enum DriveSpeed
    {
        SLOW, MEDIUM, FAST
    }

    private DriveSpeed driveSpeed = DriveSpeed.MEDIUM;
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

        driveSpeed = DriveSpeed.MEDIUM;

        if (DEBUG_LOOP_TIME)
        {
            loopPerformanceMonitor = new TrcLoopPerformanceMonitor("TeleOpLoop", 1.0);
        }
    } // startMode

    @Override
    public void stopMode(RunMode prevMode, RunMode nextMode)
    {
    } // stopMode

    @Override
    public void runPeriodic(double elapsedTime)
    {
        double driveScale = 0.8;

        double x, y, rot;
        boolean fieldOriented;

        if (USE_CONTROLLER)
        {
            x = deadband(robot.xboxController.getX(GenericHID.Hand.kLeft));
            y = deadband(-robot.xboxController.getY(GenericHID.Hand.kLeft));
            double rightTrigger = deadband(robot.xboxController.getTriggerAxis(GenericHID.Hand.kRight));
            double leftTrigger = deadband(robot.xboxController.getTriggerAxis(GenericHID.Hand.kLeft));
            rot = rightTrigger > 0 ? rightTrigger : -leftTrigger;
            fieldOriented = robot.xboxController.getXButton();
        }
        else
        {
            double leftDriveX = robot.leftDriveStick.getXWithDeadband(true);
            double leftDriveY = robot.leftDriveStick.getYWithDeadband(true);
            double leftTwist = robot.leftDriveStick.getTwistWithDeadband(true);
            double rightDriveX = robot.rightDriveStick.getXWithDeadband(true);
            double rightDriveY = robot.rightDriveStick.getYWithDeadband(true);
            double rightTwist = robot.rightDriveStick.getTwistWithDeadband(true);
            x = rightDriveX;
            y = rightDriveY;
            rot = rightTwist;
            fieldOriented = robot.rightDriveStick.getRawButton(FrcJoystick.SIDEWINDER_TRIGGER);
        }

        robot.updateDashboard(RunMode.TELEOP_MODE);

        robot.driveBase.holonomicDrive(x * driveScale, y * driveScale, rot * driveScale,
            fieldOriented ? robot.driveBase.getHeading() : 0.0);
    } // runPeriodic

    private double deadband(double d)
    {
        return Math.abs(d) >= 0.15 ? d : 0;
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
        robot.dashboard
            .displayPrintf(8, "RightDriveStick: button=0x%04x %s, auto=%b", button, pressed ? "pressed" : "released",
                robot.isAutoActive());

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
                break;

            case FrcJoystick.PANEL_BUTTON_RED2:
                break;

            case FrcJoystick.PANEL_BUTTON_GREEN2:
                break;

            case FrcJoystick.PANEL_BUTTON_BLUE2:
                break;

            case FrcJoystick.PANEL_BUTTON_YELLOW2:
                break;

            case FrcJoystick.PANEL_BUTTON_WHITE2:
                break;
        }
    } // operatorStickButtonEvent

    public void switchPanelButtonEvent(int button, boolean pressed)
    {
        robot.dashboard.displayPrintf(8, "  SwitchPanel: button=0x%04x %s", button, pressed ? "pressed" : "released");
        switch (button)
        {
            case FrcJoystick.PANEL_SWITCH_WHITE1:
                break;

            case FrcJoystick.PANEL_SWITCH_RED1:
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
                break;

            case FrcJoystick.PANEL_SWITCH_YELLOW2:
                break;
        }
    }
} // class FrcTeleOp
