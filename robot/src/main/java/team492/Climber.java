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
import trclib.TrcDigitalTrigger;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTaskMgr;

public class Climber
{
    private enum State
    {
        INIT_SUBSYSTEMS, MANUAL_CLIMB
    }

    public enum HabLevel
    {
        LEVEL_2(RobotInfo.CLIMBER_ELEVATOR_POS_LVL_2), LEVEL_3(RobotInfo.CLIMBER_ELEVATOR_POS_LVL_3);

        private double height;

        HabLevel(double height)
        {
            this.height = height;
        }

        public double getHeight()
        {
            return height;
        }
    }

    private FrcCANTalon actuator;
    private FrcCANTalon climberWheels;
    private Robot robot;
    private TrcTaskMgr.TaskObject climbTaskObj;
    private TrcStateMachine<State> sm;
    private HabLevel level;
    private TrcDigitalTrigger actuatorLowerLimitSwitchTrigger;
    private boolean calibrating = false;
    private boolean driveWheels = false;
    private boolean driveActuator = false;
    private FrcJoystick.ButtonHandler rightDriveHandler, panelHandler;

    public Climber(Robot robot)
    {
        this.robot = robot;

        actuator = new FrcCANTalon("ClimberActuator", RobotInfo.CANID_CLIMB_ACTUATOR); // this should be 7, eventually
        actuator.motor.overrideLimitSwitchesEnable(true);
        actuator.configFwdLimitSwitchNormallyOpen(false);
        actuator.configRevLimitSwitchNormallyOpen(false);
        actuator.setBrakeModeEnabled(true);

        FrcCANTalonLimitSwitch actuatorLowerLimitSwitch = new FrcCANTalonLimitSwitch("ActuatorLowerLimit", actuator,
            false);
        actuatorLowerLimitSwitchTrigger = new TrcDigitalTrigger("TrcDigitalTrigger", actuatorLowerLimitSwitch,
            this::lowerLimitSwitchEvent);
        actuatorLowerLimitSwitchTrigger.setEnabled(true);

        climberWheels = new FrcCANTalon("ClimberWheels", RobotInfo.CANID_CLIMB_WHEELS);
        climberWheels.setInverted(true);
        climberWheels.setBrakeModeEnabled(true);

        climbTaskObj = TrcTaskMgr.getInstance().createTask("ClimberTask", this::climbTask);

        sm = new TrcStateMachine<>("ClimbStateMachine");
    }

    private void lowerLimitSwitchEvent(boolean triggered)
    {
        actuator.resetPosition(true);
        if (calibrating)
        {
            actuator.set(0.0);
            calibrating = false;
        }
    }

    public void zeroCalibrateActuator()
    {
        actuatorLowerLimitSwitchTrigger.setEnabled(true);
        actuator.set(RobotInfo.CLIMBER_ACTUATOR_CAL_POWER);
        calibrating = true;
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
        calibrating = false;
    }

    public void climb(HabLevel level)
    {
        this.level = level;
        sm.start(State.INIT_SUBSYSTEMS);
        setEnabled(true);
        calibrating = false;
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
            driveWheels = false;
            driveActuator = false;
            restoreButtonHandlers();
            setEnabled(false);
            robot.dashboard.displayPrintf(9, "");
        }
    }

    private void overrideButtonHandlers()
    {
        rightDriveHandler = robot.rightDriveStick.getButtonHandler();
        panelHandler = robot.buttonPanel.getButtonHandler();
        robot.rightDriveStick.setButtonHandler(this::rightDriveStickButtonEvent);
        robot.buttonPanel.setButtonHandler(this::buttonPanelButtonEvent);
    }

    private void restoreButtonHandlers()
    {
        robot.rightDriveStick.setButtonHandler(rightDriveHandler);
        robot.buttonPanel.setButtonHandler(panelHandler);
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
                    overrideButtonHandlers();

                    robot.setHalfBrakeModeEnabled(true);
                    climberWheels.set(0.0);
                    climberWheels.setBrakeModeEnabled(true);

                    robot.elevator.setManualOverrideEnabled(true);
                    robot.pickup.setManualOverrideEnabled(true);

                    sm.setState(State.MANUAL_CLIMB);
                    break;

                case MANUAL_CLIMB:
                    robot.elevator.getPidController().restoreOutputLimit();
                    robot.pickup.getPitchPidController().restoreOutputLimit();

                    robot.elevator.setPower(robot.operatorStick.getYWithDeadband(true));
                    robot.pickup.setPitchPower(RobotInfo.CLIMBER_PICKUP_HOLD_POWER);

                    setActuatorPower(driveActuator ? RobotInfo.CLIMBER_ACTUATOR_CLIMB_POWER : 0.0);

                    if (driveWheels)
                    {
                        robot.driveBase.arcadeDrive(0.0, 0.0);
                        climberWheels.setBrakeModeEnabled(true);
                        setWheelPower(robot.rightDriveStick.getYWithDeadband(true));
                    }
                    else
                    {
                        setWheelPower(0.0);
                        climberWheels.setBrakeModeEnabled(false);
                        robot.driveBase.arcadeDrive(robot.rightDriveStick.getYWithDeadband(true), 0.0);
                    }
                    break;
            }
        }
    }

    private void rightDriveStickButtonEvent(int button, boolean pressed)
    {
        switch (button)
        {
            case FrcJoystick.SIDEWINDER_BUTTON8:
                driveWheels = pressed;
                break;
        }
    }

    private void buttonPanelButtonEvent(int button, boolean pressed)
    {
        switch (button)
        {
            case FrcJoystick.PANEL_BUTTON8:
                driveActuator = pressed;
                break;
        }
    }
}
