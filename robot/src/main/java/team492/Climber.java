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
import trclib.TrcBangBangController;
import trclib.TrcMotor;
import trclib.TrcPidMotor;
import trclib.TrcPidMotorToMotorAdapter;
import trclib.TrcPidController;
import trclib.TrcUtil;

public class Climber
{
    private TrcPidMotor climber;
    private TrcMotor actuator;
    private TrcBangBangController actuatorController;
    private Robot robot;

    public Climber(Robot robot)
    {
        this.robot = robot;

        TrcMotor elevator = new TrcPidMotorToMotorAdapter("ElevatorAdapter", robot.elevator.getElevator());
        actuator = new FrcCANTalon("ClimberActuator", RobotInfo.CANID_LEFT_DRIVE_MASTER);

        actuatorController = new TrcBangBangController("ActuatorController", RobotInfo.CLIMBER_ACTUATOR_CAL_POWER, 0.0,
            actuator);

        TrcPidController.PidCoefficients pidCoefficients = new TrcPidController.PidCoefficients(RobotInfo.CLIMBER_KP,
            RobotInfo.CLIMBER_KI, RobotInfo.CLIMBER_KD);
        TrcPidController climberController = new TrcPidController("Climber", pidCoefficients,
            RobotInfo.CLIMBER_TOLERANCE, this::getClimberPosition);
        climber = new TrcPidMotor("Climber", elevator, actuator, climberController, 0.0);

        /**
         * The climb task is as follows:
         * 1. Set elevator to the climb position, set pickup angle
         * 2. Set actuator to ground position with bang bang controller
         * 3. Raise them up using a preset power (with sync power) and a bang bang controller
         */
    }

    public double getClimberPosition()
    {
        return TrcUtil.average(actuator.getMotorPosition() * RobotInfo.CLIMBER_ACTUATOR_SCALE,
            robot.elevator.getPosition() - RobotInfo.ELEVATOR_POS_CLIMB);
    }
}
