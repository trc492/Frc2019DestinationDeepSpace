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
import trclib.TrcPidActuator;
import trclib.TrcPidController;

public class Climber2
{
    private class ClimberMotor extends FrcCANTalon
    {
        public ClimberMotor(String instanceName, int canId)
        {
            super(instanceName, canId);
        }   //ClimberMotor
    
        @Override
        public double getPosition()
        {
            return habHeight - super.getPosition() * RobotInfo.CLIMBER_INCHES_PER_COUNT;
        }   //getPosition

    }   //class ClimberMotor
    
    private final Elevator elevator;
    private final ClimberMotor climberMotor;
    private final TrcPidActuator climber;
    private double habHeight;

    public Climber2(Elevator elevator)
    {
        this.elevator = elevator;

        climberMotor = new ClimberMotor("ClimberMotor", RobotInfo.CANID_CLIMB_ACTUATOR);
        climberMotor.setInverted(true);
        climberMotor.setPositionSensorInverted(true);
        climberMotor.motor.overrideLimitSwitchesEnable(true);
        climberMotor.configFwdLimitSwitchNormallyOpen(false);
        climberMotor.configRevLimitSwitchNormallyOpen(false);
        climberMotor.setBrakeModeEnabled(true);

        TrcPidController.PidCoefficients pidCoefficients = new TrcPidController.PidCoefficients(
            RobotInfo.CLIMBER_KP, RobotInfo.CLIMBER_KI, RobotInfo.CLIMBER_KD);
        TrcPidController pidController = new TrcPidController(
            "ClimberPidController", pidCoefficients, RobotInfo.CLIMBER_TOLERANCE, this::getPosition);
        FrcCANTalonLimitSwitch lowerLimitSwitch = new FrcCANTalonLimitSwitch(
            "ClimberLowerLimitSwitch", climberMotor, false);
        // TODO: Need to determine the proper gravity compensation value.
        climber = new TrcPidActuator(
            "Climber", elevator.getMotor(), climberMotor, RobotInfo.CLIMBER_SYNC_GAIN, lowerLimitSwitch, pidController,
            RobotInfo.CLIMBER_CALIBRATE_POWER, RobotInfo.CLIMBER_PID_FLOOR, RobotInfo.CLIMBER_PID_CEILING,
            () -> RobotInfo.CLIMBER_GRAVITY_COMP);
        climber.setPositionScale(RobotInfo.CLIMBER_INCHES_PER_COUNT, RobotInfo.CLIMBER_MIN_POS);
    }   //Climber

    public double getPosition()
    {
        // This should be the same as elevator position.
        return climber.getPosition();
    }   //getPosition

    public void zeroCalibrate(double habHeight)
    {
        this.habHeight = habHeight;
        elevator.setPosition(habHeight);
        climberMotor.set(-Math.abs(RobotInfo.CLIMBER_CALIBRATE_POWER));
    }

    public void climb()
    {
        climber.setTarget(RobotInfo.ELEVATOR_MIN_POS, true);
    }

}   //class Climber
