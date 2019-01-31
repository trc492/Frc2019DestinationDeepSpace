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

import java.util.Date;

import frclib.FrcChoiceMenu;
import hallib.HalDashboard;
import trclib.TrcRobot;

public class FrcAuto implements TrcRobot.RobotMode
{
    public static enum AutoStrategy
    {
        LEFT_GEAR_LIFT,
        RIGHT_GEAR_LIFT,
        MIDDLE_GEAR_LIFT,
        X_TIMED_DRIVE,
        Y_TIMED_DRIVE,
        X_DISTANCE_DRIVE,
        Y_DISTANCE_DRIVE,
        TURN_DEGREES,
        DO_NOTHING
    }   //enum AutoStrategy

    private Robot robot;
    private boolean useVision = false;

    //
    // Menus.
    //
    private FrcChoiceMenu<FrcAuto.AutoStrategy> autoStrategyMenu;

    private AutoStrategy autoStrategy;
    private double delay;

    private TrcRobot.RobotCommand autoCommand;

    public FrcAuto(Robot robot)
    {
        this.robot = robot;
        //
        // Create Autonomous Mode specific menus.
        //
        autoStrategyMenu = new FrcChoiceMenu<>("Autonomous Strategies");
        //
        // Populate Autonomous Mode menus.
        //
        autoStrategyMenu.addChoice("Middle Gear Lift", FrcAuto.AutoStrategy.MIDDLE_GEAR_LIFT, true);
        autoStrategyMenu.addChoice("Left Gear Lift", FrcAuto.AutoStrategy.LEFT_GEAR_LIFT, false);
        autoStrategyMenu.addChoice("Right Gear Lift", FrcAuto.AutoStrategy.RIGHT_GEAR_LIFT, false);
        autoStrategyMenu.addChoice("X Timed Drive", FrcAuto.AutoStrategy.X_TIMED_DRIVE, false);
        autoStrategyMenu.addChoice("Y Timed Drive", FrcAuto.AutoStrategy.Y_TIMED_DRIVE, false);
        autoStrategyMenu.addChoice("X Distance Drive", FrcAuto.AutoStrategy.X_DISTANCE_DRIVE, false);
        autoStrategyMenu.addChoice("Y Distance Drive", FrcAuto.AutoStrategy.Y_DISTANCE_DRIVE, false);
        autoStrategyMenu.addChoice("Turn Degrees", FrcAuto.AutoStrategy.TURN_DEGREES, false);
        autoStrategyMenu.addChoice("Do Nothing", FrcAuto.AutoStrategy.DO_NOTHING, false);
    }   //FrcAuto

    //
    // Implements TrcRobot.RunMode.
    //

    @Override
    public void startMode()
    {
        HalDashboard.getInstance().clearDisplay();

        if (Robot.USE_TRACELOG) robot.startTraceLog(null);

        Date now = new Date();
        robot.tracer.traceInfo(Robot.programName, "%s: ***** Starting autonomous *****", now.toString());

        //
        // Retrieve menu choice values.
        //
        autoStrategy = autoStrategyMenu.getCurrentChoiceObject();
        delay = HalDashboard.getNumber("Delay", 0.0);

        switch (autoStrategy)
        {
            case LEFT_GEAR_LIFT:
                useVision = true;
                autoCommand = new CmdSideGearLift(robot, delay, false);
                break;

            case RIGHT_GEAR_LIFT:
                useVision = true;
                autoCommand = new CmdSideGearLift(robot, delay, true);
                break;

            case MIDDLE_GEAR_LIFT:
                useVision = true;
                autoCommand = new CmdMidGearLift(robot, delay);
                break;

            case X_TIMED_DRIVE:
                autoCommand = new CmdTimedDrive(robot, delay, robot.driveTime, robot.drivePower, 0.0, 0.0);
                break;

            case Y_TIMED_DRIVE:
                autoCommand = new CmdTimedDrive(robot, delay, robot.driveTime, 0.0, robot.drivePower, 0.0);
                break;

            case X_DISTANCE_DRIVE:
                autoCommand = new CmdPidDrive(
                    robot, delay, robot.driveDistance, 0.0, 0.0, robot.drivePowerLimit, false);
                break;

            case Y_DISTANCE_DRIVE:
                autoCommand = new CmdPidDrive(
                    robot, delay, 0.0, robot.driveDistance, 0.0, robot.drivePowerLimit, false);
                break;

            case TURN_DEGREES:
                autoCommand = new CmdPidDrive(
                    robot, delay, 0.0, 0.0, robot.turnDegrees, robot.drivePowerLimit, false);
                break;

            default:
            case DO_NOTHING:
                autoCommand = null;
                break;
        }
        //
        // Start vision thread if necessary.
        //
        if (useVision)
        {
            robot.setVisionEnabled(true);
        }

        robot.driveBase.resetPosition();
        robot.targetHeading = 0.0;

        robot.encoderXPidCtrl.setOutputRange(-0.5, 0.5);
        robot.encoderYPidCtrl.setOutputRange(-0.5, 0.5);
        robot.gyroTurnPidCtrl.setOutputRange(-0.5, 0.5);
        robot.sonarDrivePidCtrl.setOutputRange(-0.5, 0.5);
        robot.visionTurnPidCtrl.setOutputRange(-0.5, 0.5);
    }   //startMode

    @Override
    public void stopMode()
    {
        if (useVision)
        {
            robot.setVisionEnabled(false);
        }

        if (Robot.USE_TRACELOG) robot.stopTraceLog();
    }   //stopMode

    @Override
    public void runPeriodic(double elapsedTime)
    {
    }   //runPeriodic

    @Override
    public void runContinuous(double elapsedTime)
    {
        if (autoCommand != null)
        {
            autoCommand.cmdPeriodic(elapsedTime);

            if (robot.pidDrive.isActive())
            {
                robot.encoderXPidCtrl.printPidInfo(robot.tracer, robot.battery);
                robot.encoderYPidCtrl.printPidInfo(robot.tracer, robot.battery);
                robot.gyroTurnPidCtrl.printPidInfo(robot.tracer, robot.battery);
            }
            else if (robot.visionPidDrive.isActive())
            {
                robot.sonarDrivePidCtrl.printPidInfo(robot.tracer, robot.battery);
                robot.visionTurnPidCtrl.printPidInfo(robot.tracer, robot.battery);
            }

            robot.updateDashboard();
        }
    }   //runContinuous

}   //class FrcAuto
