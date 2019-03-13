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

package trclib;

public class TrcPidMotorToMotorAdapter extends TrcMotor
{
    private TrcPidMotor pidMotor;
    public TrcPidMotorToMotorAdapter(String instanceName, TrcPidMotor pidMotor)
    {
        super(instanceName);
        this.pidMotor = pidMotor;
    }

    @Override
    public double getMotorPosition()
    {
        return pidMotor.getPosition();
    }

    @Override
    public void setMotorPower(double power)
    {
        pidMotor.setPower(power);
    }

    @Override
    public boolean getInverted()
    {
        return pidMotor.getMotor().getInverted();
    }

    @Override
    public double getPower()
    {
        return pidMotor.getMotor().getPower();
    }

    @Override
    public boolean isLowerLimitSwitchActive()
    {
        return pidMotor.getMotor().isLowerLimitSwitchActive();
    }

    @Override
    public boolean isUpperLimitSwitchActive()
    {
        return pidMotor.getMotor().isUpperLimitSwitchActive();
    }

    @Override
    public void resetPosition(boolean hardware)
    {
        pidMotor.getMotor().resetPosition(hardware);
    }

    @Override
    public void setBrakeModeEnabled(boolean enabled)
    {
        pidMotor.getMotor().setBrakeModeEnabled(enabled);
    }

    @Override
    public void setInverted(boolean inverted)
    {
        pidMotor.getMotor().setInverted(inverted);
    }

    @Override
    public void setPositionSensorInverted(boolean inverted)
    {
        pidMotor.getMotor().setPositionSensorInverted(inverted);
    }

    @Override
    public void setSoftLimitEnabled(boolean lowerLimitEnabled, boolean upperLimitEnabled)
    {
        pidMotor.getMotor().setSoftLimitEnabled(lowerLimitEnabled, upperLimitEnabled);
    }

    @Override
    public void setSoftLowerLimit(double position)
    {
        pidMotor.getMotor().setSoftLowerLimit(position);
    }

    @Override
    public void setSoftUpperLimit(double position)
    {
        pidMotor.getMotor().setSoftUpperLimit(position);
    }
}
