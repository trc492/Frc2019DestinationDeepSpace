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

import frclib.FrcCANTalon;
import frclib.FrcCANTalonLimitSwitch;
import frclib.FrcJoystick;
import frclib.FrcPdp;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTaskMgr;

public class Climber
{
    private enum State
    {
        PREP_ELEVATOR, CALIB_ELEVATOR, INIT_SUBSYSTEMS, MANUAL_CLIMB
    }

    private FrcCANTalon actuator;
    private FrcCANTalon climberWheels;
    private Robot robot;
    private TrcTaskMgr.TaskObject climbTaskObj;
    private TrcStateMachine<State> sm;
    private TrcEvent event;
    private boolean wheelContacted = false;

    public Climber(Robot robot)
    {
        this.robot = robot;

        actuator = new FrcCANTalon("ClimberActuator", RobotInfo.CANID_CLIMB_ACTUATOR); // this should be 7, eventually
        actuator.motor.overrideLimitSwitchesEnable(true);
        actuator.configFwdLimitSwitchNormallyOpen(false);
        actuator.configRevLimitSwitchNormallyOpen(false);
        actuator.setBrakeModeEnabled(true);
        actuator.motor.configVoltageCompSaturation(RobotInfo.BATTERY_NOMINAL_VOLTAGE);
        actuator.motor.enableVoltageCompensation(true);

        FrcCANTalonLimitSwitch actuatorLowerLimitSwitch = new FrcCANTalonLimitSwitch("ActuatorLowerLimit", actuator,
            false);
        actuator.resetPositionOnDigitalInput(actuatorLowerLimitSwitch);

        climberWheels = new FrcCANTalon("ClimberWheels", RobotInfo.CANID_CLIMB_WHEELS);
        climberWheels.setInverted(true);
        climberWheels.setBrakeModeEnabled(true);

        robot.pdp.registerEnergyUsed(new FrcPdp.Channel(RobotInfo.PDP_CHANNEL_CLIMBER, "Climber"),
            new FrcPdp.Channel(RobotInfo.PDP_CHANNEL_LEFT_FRONT_WHEEL, "ClimberWheels"));

        climbTaskObj = TrcTaskMgr.getInstance().createTask("ClimberTask", this::climbTask);

        sm = new TrcStateMachine<>("ClimbStateMachine");
        event = new TrcEvent("TrcEvent");
    }

    public void zeroCalibrateActuator()
    {
        actuator.zeroCalibrate(RobotInfo.CLIMBER_CALIBRATE_POWER);
    }

    public void setWheelPower(double power)
    {
        climberWheels.set(power);
    }

    public double getWheelPower()
    {
        return climberWheels.getPower();
    }

    public double getWheelRawPos()
    {
        return climberWheels.getPosition();
    }

    public boolean getLowerLimitSwitch()
    {
        return actuator.isLowerLimitSwitchActive();
    }

    public boolean getUpperLimitSwitch()
    {
        return actuator.isUpperLimitSwitchActive();
    }

    public void setActuatorPower(double power)
    {
        actuator.set(power);
    }

    public void climb()
    {
        sm.start(State.PREP_ELEVATOR);
        setEnabled(true);
    }

    private void setEnabled(boolean enabled)
    {
        if (enabled)
        {
            climbTaskObj.registerTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        }
        else
        {
            climbTaskObj.unregisterTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        }
    }

    public double getActuatorPower()
    {
        return actuator.getPower();
    }

    public double getActuatorRawPos()
    {
        return actuator.getPosition();
    }

    public void cancel()
    {
        if (isActive())
        {
            actuator.set(0.0);
            robot.elevator.setPower(0.0);
            robot.elevator.setManualOverrideEnabled(false);
            robot.pickup.setManualOverrideEnabled(false);
            climberWheels.set(0.0);
            robot.pickup.setPitchPower(0.0);
            sm.stop();
            setEnabled(false);
            wheelContacted = false;
            robot.climbingButDriving = false;
            robot.dashboard.displayPrintf(9, "");
        }
    }

    public boolean isActive()
    {
        return sm.isEnabled();
    }

    private void climbTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        State state = sm.checkReadyAndGetState();
        if (state != null)
        {
            robot.dashboard.displayPrintf(9, "State: %s", state.toString());
            switch (state)
            {
                case PREP_ELEVATOR:
                    boolean levelThree;
                    robot.climbingButDriving = true;
                    if (robot.switchPanel.getRawButton(RobotInfo.PANEL_BUTTON_CLIMB))
                    {
                        levelThree = false;
                        robot.elevator.setPosition(RobotInfo.CLIMBER_ELEVATOR_POS_LVL_2_CLEARANCE, event);
                    }
                    else
                    {
                        levelThree = true;
                        robot.elevator.setPosition(RobotInfo.CLIMBER_ELEVATOR_POS_LVL_3_CLEARANCE, event);
                    }
                    robot.globalTracer.traceInfo("climbTask", "Climb state=PREP_ELEVATOR,level3=%b", levelThree);
                    robot.pickup.setManualOverrideEnabled(true);
                    robot.pickup.setPitchPower(RobotInfo.CLIMBER_PICKUP_HOLD_POWER);
                    sm.waitForSingleEvent(event, State.CALIB_ELEVATOR, 2.0);
                    break;

                case CALIB_ELEVATOR:
                    boolean levelTwo = robot.switchPanel.getRawButton(RobotInfo.PANEL_BUTTON_CLIMB);
                    if (levelTwo)
                    {
                        robot.elevator.setPosition(RobotInfo.CLIMBER_ELEVATOR_POS_LVL_2_CLEARANCE);
                    }
                    else
                    {
                        robot.elevator.setPosition(RobotInfo.CLIMBER_ELEVATOR_POS_LVL_3_CLEARANCE);
                    }
                    if (robot.buttonPanel.getRawButton(RobotInfo.PANEL_BUTTON_CLIMB))
                    {
                        robot.globalTracer.traceInfo("climbTask", "Starting climb! level3=%b", !levelTwo);
                        robot.climbingButDriving = false;
                        if (levelTwo)
                        {
                            robot.elevator.setPosition(RobotInfo.CLIMBER_ELEVATOR_POS_LVL_2, event);
                        }
                        else
                        {
                            robot.elevator.setPosition(RobotInfo.CLIMBER_ELEVATOR_POS_LVL_3, event);
                        }
                        sm.waitForSingleEvent(event, State.INIT_SUBSYSTEMS, 2.0);
                    }
                    break;

                case INIT_SUBSYSTEMS:
                    robot.globalTracer.traceInfo("climbTask", "Initializing climb!");
                    robot.setHalfBrakeModeEnabled(false);
                    climberWheels.set(0.0);
                    climberWheels.setBrakeModeEnabled(true);
                    wheelContacted = false;

                    robot.elevator.setManualOverrideEnabled(true);
                    robot.pickup.setManualOverrideEnabled(true);

                    sm.setState(State.MANUAL_CLIMB);
                    break;

                case MANUAL_CLIMB:
                    // pickup pitch is at maximum power.
                    if (!wheelContacted)
                    {
                        robot.pickup.setPitchPower(RobotInfo.CLIMBER_PICKUP_HOLD_POWER);
                    }

                    // If panel button 8 is pressed and held, we are climbing. Set climber power to maximum. As long as
                    // the elevator height is not at the lowest DONE position, set elevator power to a percentage of the
                    // climber power so the robot is level. If necessary, allow the operator stick to fine adjust the
                    // elevator power to level the robot. If the elevator has reached the lowest DONE position, robot
                    // front wheels must be over the HAB platform???, so turn elevator power off and let the front end
                    // drop onto the HAB.
                    if (robot.buttonPanel.getRawButton(RobotInfo.PANEL_BUTTON_CLIMB))
                    {
                        setActuatorPower(RobotInfo.CLIMBER_ACTUATOR_CLIMB_POWER);
                        if (robot.elevator.getPosition() > RobotInfo.CLIMBER_ELEVATOR_DONE_POS)
                        {
                            robot.elevator.setPower(
                                RobotInfo.CLIMBER_ELEVATOR_CLIMB_POWER + robot.operatorStick.getYWithDeadband(true));
                        }
                        else
                        {
                            // CodeReview: while the elevator is not at DONE position, it will keep going down.
                            // But once it reaches DONE position, power is set to zero and the robot front will
                            // start to drop. Since we keep repeating this state, the elevator power will be
                            // re-applied. The end result is then the robot front will twitch up and down at around
                            // the DONE position while the BLUE button is held. Is this correct? If so, why don't
                            // we just apply GRAVITY_COMP instead of 0.0 here so the robot front will be held at
                            // DONE position steadily.
                            robot.elevator.setPower(0.0);
                        }
                    }
                    else
                    {
                        // If panel button 8 is released, set climber power to GravityComp so it will hold its height
                        // while setting elevator power to zero to let the front end drop. It looks like we are
                        // expecting not releasing the BLUE button until the front wheel passes above the HAB platform.
                        // Because if we release the BLUE button too early, we are setting elevator power zero here
                        // meaning the robot front will start to drop and we don't want it drop unless the front wheels
                        // pass above the HAB platform.
                        if (robot.buttonPanel.getRawButton(FrcJoystick.PANEL_BUTTON_YELLOW2))
                        {
                            setActuatorPower(-0.1);
                        }
                        else
                        {
                            setActuatorPower(RobotInfo.CLIMBER_ACTUATOR_GRAVITY_COMP);
                        }
                        robot.elevator.setPower(0.0);
                    }

                    // Driver presses left stick button 2 when the front wheel is completely above the HAB platform.
                    // So retract the pickup to upright position so the front end of the robot will drop onto the
                    // HAB platform.
                    if (robot.leftDriveStick.getRawButton(FrcJoystick.LOGITECH_BUTTON2))
                    {
                        wheelContacted = true;
                        robot.pickup.setManualOverrideEnabled(true);
                        robot.pickup.setPitchPower(-0.6);
                    }

                    robot.globalTracer
                        .traceInfo("ClimbTask", "elevPower=%.2f,climbPower=%.2f,elevSpeed=%.2f,climbSpeed=%.2f",
                            robot.elevator.getPower(), actuator.getPower(),
                            robot.elevator.getMotor().getVelocity() * RobotInfo.ELEVATOR_INCHES_PER_COUNT,
                            actuator.getVelocity() * RobotInfo.CLIMBER_INCHES_PER_COUNT);

                    // Use the left drive stick to control the climber wheel power while the right drive stick is
                    // controlling the drive base wheel power.
                    setWheelPower(robot.leftDriveStick.getYWithDeadband(true));
                    robot.driveBase.arcadeDrive(robot.rightDriveStick.getYWithDeadband(true), 0.0);
                    break;
            }
        }
        else
        {
            robot.dashboard.displayPrintf(9, "State: DISABLED");
        }
    }
}
