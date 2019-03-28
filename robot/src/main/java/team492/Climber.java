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
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTaskMgr;
import trclib.TrcUtil;

public class Climber
{
    private enum State
    {
        INIT_SUBSYSTEMS, MANUAL_CLIMB
    }

    private FrcCANTalon actuator;
    private FrcCANTalon climberWheels;
    private Robot robot;
    private TrcTaskMgr.TaskObject climbTaskObj;
    private TrcStateMachine<State> sm;
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
        sm.start(State.INIT_SUBSYSTEMS);
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
                case INIT_SUBSYSTEMS:
                    robot.setHalfBrakeModeEnabled(true);
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

                    // climber power is at high power if panel button 8 is pressed, otherwise set at 0.0.
                    // If panel button 8 is not pressed, climber is power at 0.0 and so is the elevator which may fight
                    // with the line above setting elevator power by operator stick.
                    if (robot.buttonPanel
                        .getRawButton(TrcUtil.mostSignificantSetBitPosition(FrcJoystick.PANEL_BUTTON8) + 1))
                    {
                        setActuatorPower(RobotInfo.CLIMBER_ACTUATOR_CLIMB_POWER);
                        if (robot.elevator.getPosition() > RobotInfo.CLIMBER_ELEVATOR_DONE_POS)
                        {
                            robot.elevator.setPower(
                                RobotInfo.CLIMBER_ELEVATOR_CLIMB_POWER + robot.operatorStick.getYWithDeadband(true));
                        }
                        else
                        {
                            robot.elevator.setPower(0.0);
                        }
                    }
                    else
                    {
                        setActuatorPower(RobotInfo.CLIMBER_ACTUATOR_GRAVITY_COMP);
                        robot.elevator.setPower(0.0);
                    }

                    if (robot.leftDriveStick.getRawButton(TrcUtil.mostSignificantSetBitPosition(FrcJoystick.LOGITECH_BUTTON2) + 1))
                    {
                        wheelContacted = true;
                        robot.pickup.setPickupAngle(RobotInfo.PICKUP_MIN_POS);
                    }

                    robot.globalTracer
                        .traceInfo("ClimbTask", "elevPower=%.2f,climbPower=%.2f,elevSpeed=%.2f,climbSpeed=%.2f",
                            robot.elevator.getPower(), actuator.getPower(),
                            robot.elevator.getMotor().getVelocity() * RobotInfo.ELEVATOR_INCHES_PER_COUNT,
                            actuator.getVelocity() * RobotInfo.CLIMBER_INCHES_PER_COUNT);

                    setWheelPower(robot.leftDriveStick.getYWithDeadband(true));
                    robot.driveBase.arcadeDrive(robot.rightDriveStick.getYWithDeadband(true), 0.0);
                    break;
            }
        }
    }
}
