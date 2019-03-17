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
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTaskMgr;
import trclib.TrcUtil;

public class Climber
{
    private enum State
    {
        ZERO_SUBSYSTEMS, MONITOR_ACTUATOR_CALIBRATION, START_CLIMB, CLIMB, DRIVE_FORWARD, DONE
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

    public Climber(Robot robot)
    {
        this.robot = robot;

        actuator = new FrcCANTalon("ClimberActuator", RobotInfo.CANID_CLIMB_ACTUATOR); // this should be 7, eventually
        actuator.motor.overrideLimitSwitchesEnable(true);
        actuator.configFwdLimitSwitchNormallyOpen(false);
        actuator.configRevLimitSwitchNormallyOpen(false);
        actuator.setBrakeModeEnabled(true);

        climberWheels = new FrcCANTalon("ClimberWheels", RobotInfo.CANID_CLIMB_WHEELS);
        climberWheels.setInverted(true);

        climbTaskObj = TrcTaskMgr.getInstance().createTask("ClimberTask", this::climbTask);

        sm = new TrcStateMachine<>("ClimbStateMachine");
    }

    public void setWheelPower(double power)
    {
        climberWheels.set(power);
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

    public void climb(HabLevel level)
    {
        this.level = level;
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

    public double getActuatorPosition()
    {
        return actuator.getPosition() * RobotInfo.CLIMBER_ACTUATOR_SCALE;
    }

    private double getClimbPosition()
    {
        return TrcUtil.average(getActuatorPosition(), level.getHeight() - robot.elevator.getPosition());
    }

    public void cancel()
    {
        actuator.set(0.0);
        robot.elevator.setPower(0.0);
        robot.elevator.setManualOverrideEnabled(false);
        climberWheels.set(0.0);
        climberWheels.setBrakeModeEnabled(false);
        robot.pickup.setPitchPower(0.0);
        setEnabled(false);
    }

    private void climbTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        State state = sm.checkReadyAndGetState();
        if (state != null)
        {
            switch (state)
            {
                case ZERO_SUBSYSTEMS:
                    robot.setHalfBrakeModeEnabled(false);
                    robot.elevator.setPosition(level.getHeight());
                    robot.pickup.setPickupAngle(RobotInfo.CLIMBER_PICKUP_ANGLE);
                    climberWheels.setBrakeModeEnabled(true);
                    climberWheels.set(0.0);

                    sm.setState(State.MONITOR_ACTUATOR_CALIBRATION);
                    break;

                case MONITOR_ACTUATOR_CALIBRATION:
                    if (getActuatorPosition() >= RobotInfo.CLIMBER_ACTUATOR_GROUND_POS)
                    {
                        actuator.set(0.0);
                        sm.setState(State.START_CLIMB);
                    }
                    else
                    {
                        actuator.set(RobotInfo.CLIMBER_ACTUATOR_CAL_POWER);
                    }
                    break;

                case START_CLIMB:
                    robot.elevator.setManualOverrideEnabled(true); // Disable the elevator pid
                    actuator.resetPosition(true);

                    sm.setState(State.CLIMB);
                    break;

                case CLIMB:
                    if (getClimbPosition() >= level.getHeight() - RobotInfo.CLIMBER_CLIMB_TOLERANCE)
                    {
                        robot.elevator.setPower(RobotInfo.CLIMBER_ELEVATOR_HOLD_POWER);
                        actuator.set(RobotInfo.CLIMBER_ACTUATOR_HOLD_POWER);
                        sm.setState(State.DRIVE_FORWARD);
                    }
                    else
                    {
                        actuator.set(RobotInfo.CLIMBER_ACTUATOR_CLIMB_POWER);
                        robot.elevator.setPower(RobotInfo.CLIMBER_ELEVATOR_CLIMB_POWER);
                        robot.pickup.setPitchPower(RobotInfo.CLIMBER_PICKUP_HOLD_POWER);
                    }
                    break;

                case DRIVE_FORWARD:
                    climberWheels.set(RobotInfo.CLIMBER_WHEELS_DRIVE_POWER);
                    sm.setState(State.DONE);
                    break;

                case DONE:
                    setEnabled(false);
                    break;
            }
        }
    }
}
