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
import com.revrobotics.CANError;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.IdleMode;

import trclib.TrcDbgTrace;
import trclib.TrcMotor;
import trclib.TrcPidController;
import trclib.TrcUtil;

/**
 * This class implements a SparkMAX motor controller by REV robototics. It extends the TrcMotor class and
 * implements the standard TrcMotorController interface to be compatible with the TRC library.
 * Reference manual of the motor controller can be found here:
 * http://www.revrobotics.com/sparkmax-users-manual/?mc_cid=a60a44dc08&mc_eid=1935741b98#section-2-3
 */
public class FrcCANSparkMax extends TrcMotor
{
    public CANSparkMax motor;
    private boolean brushless;
    private CANEncoder encoder;
    private CANDigitalInput fwdLimitSwitch, revLimitSwitch;
    private double maxVelocity = 0.0;
    // private boolean feedbackDeviceIsPot = false;
    private boolean limitSwitchesSwapped = false;
    private double zeroPosition = 0.0;
    private double encoderSign = 1.0;
    private double currPower = 0.0;
    private boolean softLowerLimitEnabled = false;
    private boolean softUpperLimitEnabled = false;
    private double softLowerLimit = 0.0;
    private double softUpperLimit = 0.0;
    // private FeedbackDevice feedbackDeviceType;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param deviceId     specifies the CAN ID of the device.
     * @param brushless    specifies true if the motor is brushless, false otherwise.
     */
    public FrcCANSparkMax(String instanceName, int deviceId, boolean brushless)
    {
        super(instanceName);
        this.brushless = brushless;
        motor = new CANSparkMax(deviceId,
            brushless ? CANSparkMaxLowLevel.MotorType.kBrushless : CANSparkMaxLowLevel.MotorType.kBrushed);
        encoder = motor.getEncoder();
        fwdLimitSwitch = motor.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyOpen);
        revLimitSwitch = motor.getReverseLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyOpen);
        resetPosition(true);
    }   //FrcCANSparkMax

    @Override
    public void follow(TrcMotor motor)
    {
        if (motor instanceof FrcCANSparkMax)
        {
            FrcCANSparkMax sparkMax = (FrcCANSparkMax) motor;
            this.motor.follow(sparkMax.motor);
        }
        else if (motor instanceof FrcCANTalon)
        {
            FrcCANTalon talon = (FrcCANTalon) motor;
            this.motor.follow(CANSparkMax.ExternalFollower.kFollowerPhoenix, talon.motor.getDeviceID());
        }
        else
        {
            super.follow(motor);
        }
    }

    /**
     * This method returns the motor type.
     *
     * @return true if the motor is brushless, false otherwise.
     */
    public boolean isBrushless()
    {
        final String funcName = "isBrushless";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", brushless);
        }

        return brushless;
    }   //isBrushless

    /**
     * This method sets the motor controller to velocity mode with the specified maximum velocity.
     *
     * @param maxVelocity     specifies the maximum velocity the motor can run, in sensor units per second.
     * @param pidCoefficients specifies the PIDF coefficients to send to the Talon to use for velocity control.
     */
    @Override
    public void enableVelocityMode(double maxVelocity, TrcPidController.PidCoefficients pidCoefficients)
    {
        final String funcName = "enableVelocityMode";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "maxVel=%f,pidCoefficients=%s", maxVelocity,
                pidCoefficients == null ? "N/A" : pidCoefficients.toString());
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.maxVelocity = maxVelocity;

        if (pidCoefficients != null)
        {
            CANPIDController pidController = motor.getPIDController();
            pidController.setP(pidCoefficients.kP);
            pidController.setI(pidCoefficients.kI);
            pidController.setD(pidCoefficients.kD);
            pidController.setFF(pidCoefficients.kF);
        }
    }   //enableVelocityMode

    /**
     * This method disables velocity mode returning it to power mode.
     */
    public void disableVelocityMode()
    {
        final String funcName = "disableVelocityMode";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.maxVelocity = 0.0;
    }   //disableVelocityMode

    /**
     * This method swaps the forward and reverse limit switches. By default, the lower limit switch is associated
     * with the reverse limit switch and the upper limit switch is associated with the forward limit switch. This
     * method will swap the association.
     *
     * @param swapped specifies true to swap the limit switches, false otherwise.
     */
    public void setLimitSwitchesSwapped(boolean swapped)
    {
        final String funcName = "setLimitSwitchesSwapped";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "swapped=%s", swapped);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        limitSwitchesSwapped = swapped;
    }   //setLimitSwitchesSwapped

    //
    // Overriding TrcMotor specific methods.
    //

    /**
     * This method configures the forward limit switch to be normally open (i.e. active when close).
     *
     * @param normalOpen specifies true for normal open, false for normal close.
     */
    public void configFwdLimitSwitchNormallyOpen(boolean normalOpen)
    {
        final String funcName = "configFwdLimitSwitchNormallyOpen";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "normalOpen=%s", normalOpen);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        fwdLimitSwitch = motor.getForwardLimitSwitch(
            normalOpen ? LimitSwitchPolarity.kNormallyOpen : LimitSwitchPolarity.kNormallyClosed);
    }   //configFwdLimitSwitchNormallyOpen

    /**
     * This method configures the reverse limit switch to be normally open (i.e. active when close).
     *
     * @param normalOpen specifies true for normal open, false for normal close.
     */
    public void configRevLimitSwitchNormallyOpen(boolean normalOpen)
    {
        final String funcName = "configRevLimitSwitchNormallyOpen";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "normalOpen=%s", normalOpen);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        revLimitSwitch = motor.getReverseLimitSwitch(
            normalOpen ? LimitSwitchPolarity.kNormallyOpen : LimitSwitchPolarity.kNormallyClosed);
    }   //configRevLimitSwitchNormallyOpen

    // /**
    //  * This method sets the feedback device type.
    //  *
    //  * @param devType specifies the feedback device type.
    //  */
    // public void setFeedbackDevice(FeedbackDevice devType)
    // {
    //     final String funcName = "setFeedbackDevice";

    //     if (debugEnabled)
    //     {
    //         dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "devType=%s", devType.toString());
    //         dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
    //     }

    //     this.feedbackDeviceType = devType;
    //     recordResponseCode(motor.configSelectedFeedbackSensor(devType, 0, 0));
    //     feedbackDeviceIsPot = devType == FeedbackDevice.Analog;
    // }   //setFeedbackDevice

    //
    // Implements TrcMotor abstract methods.
    //

    /**
     * This method returns the motor position by reading the position sensor. The position sensor can be an encoder
     * or a potentiometer.
     *
     * @return current motor position.
     */
    @Override
    public double getMotorPosition()
    {
        final String funcName = "getMotorPosition";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        double currPos = encoder.getPosition() * encoderSign;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%d", currPos);
        }

        return currPos;
    }   //getMotorPosition

    /**
     * This method sets the raw motor power.
     *
     * @param power specifies the percentage power (range -1.0 to 1.0) to be set.
     */
    @Override
    public void setMotorPower(double power)
    {
        final String funcName = "setMotorPower";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "value=%f", power);
        }

        if (power != currPower)
        {
            motor.set(power);
            currPower = power;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "! (value=%f)", power);
        }
    }   //setMotorPower

    //
    // Implements TrcMotorController interface.
    //

    /**
     * This method returns the state of the motor controller direction.
     *
     * @return true if the motor direction is inverted, false otherwise.
     */
    @Override
    public boolean getInverted()
    {
        final String funcName = "getInverted";
        boolean inverted = motor.getInverted();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", inverted);
        }

        return inverted;
    }   //getInverted

    /**
     * This method returns the motor position by reading the position sensor. The position sensor can be an encoder
     * or a potentiometer.
     *
     * @return current motor position.
     */
    @Override
    public double getPosition()
    {
        final String funcName = "getPosition";
        double pos = getMotorPosition() - zeroPosition;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", pos);
        }

        return pos;
    }   //getPosition

    /**
     * This method gets the last set power.
     *
     * @return the last setPower value.
     */
    @Override
    public double getPower()
    {
        final String funcName = "getPower";
        double power = motor.get();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", power);
        }

        return power;
    }   //getPower

    /**
     * This method returns the velocity of the motor rotation in sensor unit per second.
     *
     * @return motor rotation velocity in sensor unit per second.
     */
    @Override
    public double getVelocity()
    {
        final String funcName = "getVelocity";
        double velocity = encoder.getVelocity() * encoderSign / 60.0;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", velocity);
        }

        return velocity;
    }   //getVelocity

    /**
     * This method returns the state of the lower limit switch.
     *
     * @return true if lower limit switch is active, false otherwise.
     */
    @Override
    public boolean isLowerLimitSwitchActive()
    {
        final String funcName = "isLowerLimitSwitchActive";
        boolean isActive = limitSwitchesSwapped ? fwdLimitSwitch.get() : revLimitSwitch.get();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", isActive);
        }

        return isActive;
    }   //isLowerLimitSwitchClosed

    /**
     * This method returns the state of the upper limit switch.
     *
     * @return true if upper limit switch is active, false otherwise.
     */
    @Override
    public boolean isUpperLimitSwitchActive()
    {
        final String funcName = "isUpperLimitSwitchActive";
        boolean isActive = limitSwitchesSwapped ? revLimitSwitch.get() : fwdLimitSwitch.get();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", isActive);
        }

        return isActive;
    }   //isUpperLimitSwitchActive

    /**
     * This method resets the motor position sensor, typically an encoder. This method emulates a reset for a
     * potentiometer.
     *
     * @param hardware specifies true for resetting hardware position, false for resetting software position.
     */
    @Override
    public void resetPosition(boolean hardware)
    {
        final String funcName = "resetPosition";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "hardware=%s", hardware);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (hardware)
        {
            while (encoder.setPosition(0.0) != CANError.kOK)
            {
                Thread.yield();
            }
            zeroPosition = 0.0;
        }
        else
        {
            zeroPosition = getMotorPosition();
        }
    }   //resetPosition

    /**
     * This method resets the motor position sensor, typically an encoder. This method emulates a reset for a
     * potentiometer.
     */
    public void resetPosition()
    {
        resetPosition(false);
    }   //resetPosition

    /**
     * This method sets the motor output value. The value can be power or velocity percentage depending on whether
     * the motor controller is in power mode or velocity mode.
     *
     * @param value specifies the percentage power or velocity (range -1.0 to 1.0) to be set.
     */
    @Override
    public void set(double value)
    {
        final String funcName = "set";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "value=%f", value);
        }

        if (!TrcUtil.inRange(value, -1.0, 1.0))
        {
            throw new IllegalArgumentException("Value must be in the range of -1.0 to 1.0.");
        }

        if (softLowerLimitEnabled && value < 0.0 && getPosition() <= softLowerLimit
            || softUpperLimitEnabled && value > 0.0 && getPosition() >= softUpperLimit)
        {
            value = 0.0;
        }

        if (maxVelocity == 0.0)
        {
            currPower = value;
        }
        // else
        // {
        //     controlMode = ControlMode.Velocity;
        //     value *= maxVelocity;
        //     value = TrcUtil.round(value); // Velocity mode is in sensor units/100ms, and sensor units are in integers.
        // }
        motor.set(value);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "! (value=%f)", value);
        }
    }   //set

    /**
     * This method enables/disables motor brake mode. In motor brake mode, set power to 0 would stop the motor very
     * abruptly by shorting the motor wires together using the generated back EMF to stop the motor. When brakMode
     * is false (i.e. float/coast mode), the motor wires are just disconnected from the motor controller so the motor
     * will stop gradually.
     *
     * @param enabled specifies true to enable brake mode, false otherwise.
     */
    @Override
    public void setBrakeModeEnabled(boolean enabled)
    {
        final String funcName = "setBrakeModeEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enabled=%s", enabled);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        motor.setIdleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
    }   //setBrakeModeEnabled

    /**
     * This method inverts the motor direction.
     *
     * @param inverted specifies true to invert motor direction, false otherwise.
     */
    @Override
    public void setInverted(boolean inverted)
    {
        final String funcName = "setInverted";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "inverted=%s", inverted);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        motor.setInverted(inverted);
    }   //setInverted

    /**
     * This method inverts the position sensor direction. This may be rare but there are scenarios where the motor
     * encoder may be mounted somewhere in the power train that it rotates opposite to the motor rotation. This will
     * cause the encoder reading to go down when the motor is receiving positive power. This method can correct this
     * situation.
     *
     * @param inverted specifies true to invert position sensor direction, false otherwise.
     */
    @Override
    public void setPositionSensorInverted(boolean inverted)
    {
        final String funcName = "setPositionSensorInverted";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "inverted=%s", inverted);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        encoderSign = inverted ? -1.0 : 1.0;
    }   //setPositionSensorInverted

    /**
     * This method enables/disables soft limit switches.
     *
     * @param lowerLimitEnabled specifies true to enable lower soft limit switch, false otherwise.
     * @param upperLimitEnabled specifies true to enable upper soft limit switch, false otherwise.
     */
    @Override
    public void setSoftLimitEnabled(boolean lowerLimitEnabled, boolean upperLimitEnabled)
    {
        final String funcName = "setSoftLimitEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "lowerEnabled=%s,upperEnabled=%s",
                Boolean.toString(lowerLimitEnabled), Boolean.toString(upperLimitEnabled));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        softLowerLimitEnabled = lowerLimitEnabled;
        softUpperLimitEnabled = upperLimitEnabled;
    }   //setSoftLimitEnabled

    /**
     * This method sets the lower soft limit.
     *
     * @param position specifies the position of the lower limit.
     */
    @Override
    public void setSoftLowerLimit(double position)
    {
        final String funcName = "setSoftLowerLimit";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "position=%f", position);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        softLowerLimit = position;
    }   //setSoftLowerLimit

    /**
     * This method sets the upper soft limit.
     *
     * @param position specifies the position of the upper limit.
     */
    @Override
    public void setSoftUpperLimit(double position)
    {
        final String funcName = "setSoftUpperLimit";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "position=%f", position);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        softUpperLimit = position;
    }   //setSoftUpperLimit

}   //class FrcCANSparkMax
