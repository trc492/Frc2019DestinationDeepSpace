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

import com.ctre.phoenix.motorcontrol.ControlMode;

import frclib.FrcCANTalon;

public class Winch
{
    private FrcCANTalon mainMotor;
    private FrcCANTalon slaveMotor;
    private double motorPower = 0.0;

    public Winch(Robot robot)
    {
        mainMotor = new FrcCANTalon("WinchMaster", RobotInfo.CANID_WINCH_MASTER);
        slaveMotor = new FrcCANTalon("WinchSlave", RobotInfo.CANID_WINCH_SLAVE);
        robot.pdp.registerEnergyUsed(RobotInfo.PDP_CHANNEL_WINCH_MASTER, "WinchMaster");
        robot.pdp.registerEnergyUsed(RobotInfo.PDP_CHANNEL_WINCH_SLAVE, "WinchSlave");
        mainMotor.setInverted(true);
        slaveMotor.setInverted(true);
        slaveMotor.motor.set(ControlMode.Follower, RobotInfo.CANID_WINCH_MASTER);
        mainMotor.setPositionSensorInverted(false);
    }

    public double getPower()
    {
        return motorPower;
    }

    public double getCurrent()
    {
        return mainMotor.motor.getOutputCurrent() + slaveMotor.motor.getOutputCurrent();
    }

    public void setPower(double power)
    {
        motorPower = power;
        mainMotor.set(motorPower);
    }

}   //class Winch
