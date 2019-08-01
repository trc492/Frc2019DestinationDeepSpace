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
import trclib.TrcEvent;
import trclib.TrcExclusiveSubsystem;
import trclib.TrcPidActuator;
import trclib.TrcPidController;
import trclib.TrcUtil;

public class Elevator implements TrcExclusiveSubsystem
{
    private TrcPidActuator elevator;
    private FrcCANTalon motor;
    private TrcPidController pidController;

    public Elevator(Robot robot)
    {
        motor = new FrcCANTalon("ElevatorMotor", RobotInfo.CANID_ELEVATOR);
        motor.setInverted(true);
        motor.setPositionSensorInverted(true);
        motor.motor.overrideLimitSwitchesEnable(true);
        motor.configFwdLimitSwitchNormallyOpen(false);
        motor.configRevLimitSwitchNormallyOpen(false);
        motor.setBrakeModeEnabled(true);
        motor.motor.configVoltageCompSaturation(RobotInfo.BATTERY_NOMINAL_VOLTAGE);
        motor.motor.enableVoltageCompensation(true);

        robot.pdp.registerEnergyUsed(RobotInfo.PDP_CHANNEL_ELEVATOR, "Elevator");

        // TODO: Tune ALL of these constants
        TrcPidController.PidCoefficients pidCoefficients = new TrcPidController.PidCoefficients(RobotInfo.ELEVATOR_KP,
            RobotInfo.ELEVATOR_KI, RobotInfo.ELEVATOR_KD);
        pidController = new TrcPidController("ElevatorPidController", pidCoefficients, RobotInfo.ELEVATOR_TOLERANCE,
            this::getPosition);
        FrcCANTalonLimitSwitch lowerLimitSwitch = new FrcCANTalonLimitSwitch("ElevatorLowerLimitSwitch", motor, false);
        // TODO: Need to determine the proper gravity compensation value.
        elevator = new TrcPidActuator("ElevatorActuator", motor, lowerLimitSwitch, pidController,
            RobotInfo.ELEVATOR_CALIBRATE_POWER, RobotInfo.ELEVATOR_PID_FLOOR, RobotInfo.ELEVATOR_PID_CEILING,
            () -> RobotInfo.ELEVATOR_GRAVITY_COMP);
        elevator.setPositionScale(RobotInfo.ELEVATOR_INCHES_PER_COUNT, RobotInfo.ELEVATOR_MIN_POS);
        // elevator.setStallProtection(RobotInfo.ELEVATOR_STALL_MIN_POWER, RobotInfo.ELEVATOR_STALL_TIMEOUT,
        //     RobotInfo.ELEVATOR_STALL_RESET_TIMEOUT);
    }

    public TrcPidController getPidController()
    {
        return pidController;
    }

    public TrcPidActuator getElevator()
    {
        return elevator;
    }

    public FrcCANTalon getMotor()
    {
        return motor;
    }

    public double getPower()
    {
        return motor.getPower();
    }

    public void setManualOverrideEnabled(String owner, boolean enabled)
    {
        if (validateOwnership(owner))
        {
            elevator.setManualOverride(enabled);
        }
    }

    public void setManualOverrideEnabled(boolean enabled)
    {
        setManualOverrideEnabled(null, enabled);
    }

    public void zeroCalibrate(String owner)
    {
        if (validateOwnership(owner))
        {
            elevator.zeroCalibrate();
        }
    }

    public void zeroCalibrate()
    {
        zeroCalibrate(null);
    }

    public void setPosition(String owner, double position, TrcEvent event)
    {
        if (validateOwnership(owner))
        {
            position = TrcUtil.clipRange(position, RobotInfo.ELEVATOR_MIN_POS, RobotInfo.ELEVATOR_MAX_POS);
            elevator.setTarget(position, event, 0.0);
        }
    }

    public void setPosition(double position, TrcEvent event)
    {
        setPosition(null, position, event);
    }

    public void setPosition(String owner, double position)
    {
        if (validateOwnership(owner))
        {
            position = TrcUtil.clipRange(position, RobotInfo.ELEVATOR_MIN_POS, RobotInfo.ELEVATOR_MAX_POS);
            elevator.setTarget(position, position != RobotInfo.ELEVATOR_MIN_POS);
        }
    }

    public void setPosition(double position)
    {
        setPosition(null, position);
    }

    public boolean isUpperLimitSwitchActive()
    {
        return motor.isUpperLimitSwitchActive();
    }

    public boolean isLowerLimitSwitchActive()
    {
        return motor.isLowerLimitSwitchActive();
    }

    public double getPosition()
    {
        return elevator.getPosition();
    }

    public double getRawPosition()
    {
        return motor.getPosition();
    }

    public void setPower(String owner, double power)
    {
        if (validateOwnership(owner))
        {
            // elevator.cancel();
            power = TrcUtil.clipRange(power, -1.0, 1.0);
            elevator.setPower(power, true);
            // TODO: figure this out
            // motor.set(power);
        }
    }

    public void setPower(double power)
    {
        setPower(null, power);
    }
}
