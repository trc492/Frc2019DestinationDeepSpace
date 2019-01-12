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

package frclib;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import trclib.TrcMotor;
import trclib.TrcUtil;

public class FrcCANSparkMax extends TrcMotor
{

    /**
     * This class implements a SparkMAX motor controller by REV robotics. Access the user manual here:
     * http://www.revrobotics.com/sparkmax-users-manual/?mc_cid=a60a44dc08&mc_eid=1935741b98#section-2-3
     */

    private CANSparkMax motor;
    private boolean isBrushless;
    private CANEncoder encoder;
    private CANDigitalInput lowerLimitSwitch, upperLimitSwitch;

    private double zeroPosition;

    private boolean encoderInverted;

    private double softLowerLimit, softUpperLimit;
    private boolean softLowerLimitEnabled, softUpperLimitEnabled;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FrcCANSparkMax(String instanceName, int deviceId, boolean isBrushless)
    {
        super(instanceName);
        this.isBrushless = isBrushless;
        motor = new CANSparkMax(deviceId,
            isBrushless ? CANSparkMaxLowLevel.MotorType.kBrushless : CANSparkMaxLowLevel.MotorType.kBrushed);
        encoder = motor.getEncoder();
        lowerLimitSwitch = motor.getReverseLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyOpen);
        upperLimitSwitch = motor.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyOpen);
    }

    @Override
    public void setPositionSensorInverted(boolean inverted)
    {
        this.encoderInverted = inverted;
    }

    @Override
    public double getMotorPosition()
    {
        double pos = encoder.getPosition() - zeroPosition;
        return encoderInverted ? -pos : pos;
    }

    @Override
    public double getPosition()
    {
        return getMotorPosition();
    }

    @Override
    public void resetPosition(boolean hardware)
    {
        zeroPosition = getPosition();
    }

    @Override
    public double getVelocity()
    {
        double vel = encoder.getVelocity();
        return encoderInverted ? -vel : vel;
    }

    @Override
    public void setMotorPower(double power)
    {
        if (softLowerLimitEnabled || softUpperLimitEnabled)
        {
            double pos = getMotorPosition();
            if ((softLowerLimitEnabled && pos < softLowerLimit) || (softUpperLimitEnabled && pos > softUpperLimit))
            {
                power = 0.0;
            }
        }
        motor.set(power);
    }

    @Override
    public double getPower()
    {
        return motor.get();
    }

    @Override
    public boolean getInverted()
    {
        return motor.getInverted();
    }

    @Override
    public void setInverted(boolean inverted)
    {
        motor.setInverted(inverted);
    }

    @Override
    public void setBrakeModeEnabled(boolean enabled)
    {
        motor.setIdleMode(enabled ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    }

    public void setLowerLimitSwitchNormallyOpen(boolean normalOpen)
    {
        lowerLimitSwitch = motor.getReverseLimitSwitch(normalOpen ?
            CANDigitalInput.LimitSwitchPolarity.kNormallyOpen :
            CANDigitalInput.LimitSwitchPolarity.kNormallyClosed);
    }

    public void setUpperLimitSwitchNormallyOpen(boolean normalOpen)
    {
        upperLimitSwitch = motor.getForwardLimitSwitch(normalOpen ?
            CANDigitalInput.LimitSwitchPolarity.kNormallyOpen :
            CANDigitalInput.LimitSwitchPolarity.kNormallyClosed);
    }

    @Override
    public boolean isLowerLimitSwitchActive()
    {
        return lowerLimitSwitch.isLimitSwitchEnabled();
    }

    @Override
    public boolean isUpperLimitSwitchActive()
    {
        return upperLimitSwitch.isLimitSwitchEnabled();
    }

    @Override
    public void setSoftLimitEnabled(boolean lowerLimitEnabled, boolean upperLimitEnabled)
    {
        this.softLowerLimitEnabled = lowerLimitEnabled;
        this.softUpperLimitEnabled = upperLimitEnabled;
    }

    @Override
    public void setSoftLowerLimit(double position)
    {
        this.softLowerLimit = position;
    }

    @Override
    public void setSoftUpperLimit(double position)
    {
        this.softUpperLimit = position;
    }
}
