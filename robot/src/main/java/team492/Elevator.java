/*
 * Copyright (c) 2018 Titan Robotics Club (http://www.titanrobotics.com)
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

import trclib.TrcEvent;
import trclib.TrcPidActuator;
import trclib.TrcPidController;
import trclib.TrcUtil;
import frclib.FrcCANTalon;
import frclib.FrcCANTalonLimitSwitch;
import team492.Robot;
import team492.RobotInfo;

public class Elevator
{
    public FrcCANTalon elevatorMotor;
    public TrcPidController elevatorPidCtrl;
    public TrcPidActuator elevator;

    private double elevatorPower = 0.0;

    public Elevator(Robot robot)
    {
        elevatorMotor = new FrcCANTalon("elevatorMotor", RobotInfo.CANID_ELEVATOR);
        robot.pdp.registerEnergyUsed(RobotInfo.PDP_CHANNEL_ELEVATOR, "Elevator");
        elevatorMotor.configFwdLimitSwitchNormallyOpen(false);
        elevatorMotor.configRevLimitSwitchNormallyOpen(false);
        elevatorMotor.motor.overrideLimitSwitchesEnable(true);
        elevatorMotor.setInverted(true);
        elevatorMotor.setBrakeModeEnabled(true);
        elevatorPidCtrl = new TrcPidController("elevatorPidController",
            new TrcPidController.PidCoefficients(RobotInfo.ELEVATOR_KP, RobotInfo.ELEVATOR_KI, RobotInfo.ELEVATOR_KD),
            RobotInfo.ELEVATOR_TOLERANCE, this::getPosition);
        elevator = new TrcPidActuator(
            "elevator", elevatorMotor, new FrcCANTalonLimitSwitch("elevatorLowerLimit", elevatorMotor, false),
            elevatorPidCtrl, RobotInfo.ELEVATOR_CAL_POWER, RobotInfo.ELEVATOR_PID_FLOOR, RobotInfo.ELEVATOR_PID_CEILING,
            this::getGravityCompensation);
        elevator.setPositionScale(RobotInfo.ELEVATOR_INCHES_PER_COUNT, RobotInfo.ELEVATOR_POSITION_OFFSET);
        // elevator.setStallProtection(
        //     RobotInfo.ELEVATOR_STALL_MIN_POWER, RobotInfo.ELEVATOR_STALL_TIMEOUT,
        //     RobotInfo.ELEVATOR_STALL_RESET_TIMEOUT);
    }

    public void setManualOverride(boolean manualOverride)
    {
        elevator.setManualOverride(manualOverride);
    } // setManualOverride

    public void zeroCalibrate()
    {
        elevator.zeroCalibrate();
    } // zeroCalibrate

    /**
     * @param pos
     *            (Altitude in inches)
     * @param event
     *            (TrcEvent event)
     * @param timeout
     *            Set the position for Elevator to move to in inches using PID
     *            control.
     */
    public void setPosition(double pos)
    {
        pos = TrcUtil.clipRange(pos, RobotInfo.ELEVATOR_MIN_HEIGHT, RobotInfo.ELEVATOR_MAX_HEIGHT);
        elevator.setTarget(pos, pos != RobotInfo.ELEVATOR_MIN_HEIGHT);
    } // setPosition

    public void setPosition(double pos, TrcEvent event, double timeout)
    {
        pos = TrcUtil.clipRange(pos, RobotInfo.ELEVATOR_MIN_HEIGHT, RobotInfo.ELEVATOR_MAX_HEIGHT);
        elevator.setTarget(pos, event, timeout);
    }

    public void setPower(double power)
    {
        elevator.setPower(power, true);
        elevatorPower = power;
    } // setPower

    // get the current power the elevator actuator is running at.
    public double getPower()
    {
        return elevatorPower;
    }

    // get the current altitude of the elevator relative to encoder zero. (in
    // inches)
    public double getPosition()
    {
        return elevator.getPosition();
    }

    public double getGravityCompensation()
    {
        // % of power needed to keep the elevator from sliding down,
        // disregarding friction
        return RobotInfo.ELEVATOR_GRAVITY_COMPENSATION;
    }
}
