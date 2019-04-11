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
import frclib.FrcPdp;
import trclib.TrcPidActuator;
import trclib.TrcPidController;
import trclib.TrcUtil;

public class Climber2
{
    private final boolean USE_PID_ACTUATOR = true;
    private final Elevator elevator;
    private final Pickup pickup;
    private final FrcCANTalon climberMotor;
    private TrcPidActuator climber = null;
    private final FrcCANTalon climberWheels;

    public Climber2(Robot robot)
    {
        this.elevator = robot.elevator;
        this.pickup = robot.pickup;

        climberMotor = new FrcCANTalon("ClimberMotor", RobotInfo.CANID_CLIMB_ACTUATOR);
        climberMotor.motor.overrideLimitSwitchesEnable(true);
        climberMotor.configFwdLimitSwitchNormallyOpen(false);
        climberMotor.configRevLimitSwitchNormallyOpen(false);
        climberMotor.setBrakeModeEnabled(true);
        climberMotor.motor.configVoltageCompSaturation(RobotInfo.BATTERY_NOMINAL_VOLTAGE);
        climberMotor.motor.enableVoltageCompensation(true);

        FrcCANTalonLimitSwitch climberLowerLimitSwitch = new FrcCANTalonLimitSwitch(
            "ClimberLowerLimitSwitch", climberMotor, false);
        climberMotor.resetPositionOnDigitalInput(climberLowerLimitSwitch);

        if (USE_PID_ACTUATOR)
        {
            TrcPidController.PidCoefficients pidCoefficients = new TrcPidController.PidCoefficients(
                RobotInfo.CLIMBER_KP, RobotInfo.CLIMBER_KI, RobotInfo.CLIMBER_KD);
            TrcPidController climberPidController = new TrcPidController(
                "ClimberPidController", pidCoefficients, RobotInfo.CLIMBER_TOLERANCE, this::getClimberPosition);
            // TODO: Need to determine the proper gravity compensation value.
            climber = new TrcPidActuator(
                "Climber", elevator.getMotor(), climberMotor, RobotInfo.CLIMBER_SYNC_GAIN, climberLowerLimitSwitch,
                climberPidController, RobotInfo.CLIMBER_CALIBRATE_POWER, RobotInfo.CLIMBER_PID_FLOOR,
                RobotInfo.CLIMBER_PID_CEILING, () -> RobotInfo.CLIMBER_GRAVITY_COMP);
            climber.setPositionScale(RobotInfo.CLIMBER_INCHES_PER_COUNT, RobotInfo.CLIMBER_MIN_POS);
        }

        climberWheels = new FrcCANTalon("ClimberWheels", RobotInfo.CANID_CLIMB_WHEELS);
        climberWheels.setInverted(true);
        climberWheels.setBrakeModeEnabled(true);

        robot.pdp.registerEnergyUsed(
            new FrcPdp.Channel(RobotInfo.PDP_CHANNEL_CLIMBER, "Climber"),
            new FrcPdp.Channel(RobotInfo.PDP_CHANNEL_LEFT_FRONT_WHEEL, "ClimberWheels"));
    }   //Climber2

    public void zeroCalibrateClimber()
    {
        if (climber != null)
        {
            climber.zeroCalibrate();
        }
        else
        {
            climberMotor.zeroCalibrate(RobotInfo.CLIMBER_CALIBRATE_POWER);
        }
    }

    public boolean getClimberLowerLimitSwitch()
    {
        return climberMotor.isLowerLimitSwitchActive();
    }

    public boolean getClimberUpperLimitSwitch()
    {
        return climberMotor.isUpperLimitSwitchActive();
    }

    public void setupClimb(double elevatorStartPos)
    {
        elevator.setPosition(elevatorStartPos);
        pickup.setPickupAngle(RobotInfo.CLIMBER_PICKUP_ANGLE);
    }

    public void setClimbPower(double power, double adjustment)
    {
        double elevatorPower = -power;
        double climberPower = power/RobotInfo.CLIMBER_POWER_RATIO + adjustment;
        double normalizedFactor = Math.max(1.0, TrcUtil.maxMagnitude(elevatorPower, climberPower));
        elevator.setPower(elevatorPower/normalizedFactor, true);
        if (climber != null)
        {
            climber.setPower(climberPower/normalizedFactor, true);
        }
        else
        {
            climberMotor.set(climberPower == 0.0? RobotInfo.CLIMBER_GRAVITY_COMP: climberPower);
        }
    }   //setClimbPower
    
    public void setClimbPower(double power)
    {
        setClimbPower(power, 0.0);
    }   //setClimbPower

    public double getClimbPower()
    {
        return climberMotor.getPower();
    }

    public double getClimberPosition()
    {
        // This should be the same as elevator position if climber is a PidActuator.
        return climber != null? climber.getPosition(): climberMotor.getPosition();
    }   //getClimberPosition

    public double getClimberRawPos()
    {
        return climberMotor.getPosition();
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

}   //class Climber2
