/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
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

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import trclib.TrcDbgTrace;
import trclib.TrcMotor;
import trclib.TrcPidController;
import trclib.TrcUtil;

/**
 * This class implements a CANTalon motor controller. It extends the TrcMotor class and
 * implements the standard TrcMotorController interface to be compatible with the TRC library.
 */
public class FrcCANTalon extends TrcMotor
{
    private class EncoderInfo implements Sendable
    {
        private String name, subsystem;

        public EncoderInfo(String name)
        {
            this.name = name;
        }   //EncoderInfo

        @Override
        public String getName()
        {
            return name;
        }   //getName

        @Override
        public void setName(String name)
        {
            this.name = name;
        }   //setName

        @Override
        public String getSubsystem()
        {
            return subsystem;
        }   //getSubsystem

        @Override
        public void setSubsystem(String subsystem)
        {
            this.subsystem = subsystem;
        }   //setSubsystem

        @Override
        public void initSendable(SendableBuilder builder)
        {
            if (feedbackDeviceType != FeedbackDevice.QuadEncoder)
            {
                throw new IllegalStateException("Only QuadEncoder supported for Shuffleboard!");
            }

            builder.setSmartDashboardType("Quadrature Encoder");
            builder.addDoubleProperty("Speed", FrcCANTalon.this::getVelocity, null);
            builder.addDoubleProperty("Distance", FrcCANTalon.this::getPosition, null);
            builder.addDoubleProperty("DistancePerCount", () -> 1, null);
        }   //initSendable

    }   //class EncoderInfo

    public TalonSRX motor;
    private double maxVelocity = 0.0;
    private boolean feedbackDeviceIsPot = false;
    private boolean limitSwitchesSwapped = false;
    private boolean revLimitSwitchNormalOpen = false;
    private boolean fwdLimitSwitchNormalOpen = false;
    private double zeroPosition = 0.0;
    private double prevPower = 0.0;
    private boolean softLowerLimitEnabled = false;
    private boolean softUpperLimitEnabled = false;
    private double softLowerLimit = 0.0;
    private double softUpperLimit = 0.0;
    private FeedbackDevice feedbackDeviceType;

    /**
     * The number of non-success error codes reported by the device after sending a command.
     */
    private int errorCount = 0;
    private ErrorCode lastError = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param deviceNumber specifies the CAN ID of the device.
     */
    public FrcCANTalon(final String instanceName, int deviceNumber)
    {
        super(instanceName);
        motor = new TalonSRX(deviceNumber);
        resetPosition(true);
    }   //FrcCANTalon

    /**
     * This method creates an EncoderInfo object and returns it.
     *
     * @return created GyroInfo object.
     */
    public Sendable getEncoderSendable()
    {
        return new EncoderInfo(toString());
    }   //getEncoderSendable

    /**
     * This method returns the number of error responses seen from the motor after sending a command.
     *
     * @return The number of non-OK error code responses seen from the motor
     * after sending a command.
     */
    public int getErrorCount()
    {
        return errorCount;
    } //getErrorCount

    /**
     * The method returns the last CANTalon error code. If there is none, null is returned.
     *
     * @return last CAN Talon error code.
     */
    public ErrorCode getLastError()
    {
        return lastError;
    }   //getLastError

    /**
     * This method checks for error code returned by the motor controller executing the last command. If there was
     * an error, the error count is incremented.
     *
     * @param errorCode specifies the error code returned by the motor controller.
     */
    private void recordResponseCode(ErrorCode errorCode)
    {
        lastError = errorCode;
        if (errorCode != null && !errorCode.equals(ErrorCode.OK))
        {
            errorCount++;
            if (debugEnabled)
            {
                dbgTrace.traceErr("CANTalonError", "ErrorCode=%s", errorCode);
            }
        }
    } //recordResponseCode

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
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "maxVel=%f,pidCoefficients=%s",
                maxVelocity, pidCoefficients == null ? "N/A" : pidCoefficients.toString());
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.maxVelocity = maxVelocity;

        if (pidCoefficients != null)
        {
            this.motor.config_kP(0, pidCoefficients.kP);
            this.motor.config_kI(0, pidCoefficients.kI);
            this.motor.config_kD(0, pidCoefficients.kD);
            this.motor.config_kF(0, pidCoefficients.kF);
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
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "swapped=%s", Boolean.toString(swapped));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        limitSwitchesSwapped = swapped;
    }   //setLimitSwitchesSwapped

    //
    // Overriding CANTalon specific methods.
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
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "normalOpen=%s", Boolean.toString(normalOpen));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        recordResponseCode(motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
            normalOpen ? LimitSwitchNormal.NormallyOpen : LimitSwitchNormal.NormallyClosed, 0));
        fwdLimitSwitchNormalOpen = normalOpen;
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
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "normalOpen=%s", Boolean.toString(normalOpen));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        recordResponseCode(motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
            normalOpen ? LimitSwitchNormal.NormallyOpen : LimitSwitchNormal.NormallyClosed, 0));
        revLimitSwitchNormalOpen = normalOpen;
    }   //configRevLimitSwitchNormallyOpen

    /**
     * This method sets the feedback device type.
     *
     * @param devType specifies the feedback device type.
     */
    public void setFeedbackDevice(FeedbackDevice devType)
    {
        final String funcName = "setFeedbackDevice";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "devType=%s", devType.toString());
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.feedbackDeviceType = devType;
        recordResponseCode(motor.configSelectedFeedbackSensor(devType, 0, 0));
        feedbackDeviceIsPot = devType == FeedbackDevice.Analog;
    }   //setFeedbackDevice

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

        double currPos;
        if (feedbackDeviceType == FeedbackDevice.QuadEncoder)
        {
            currPos = motor.getSensorCollection().getQuadraturePosition();
        }
        else
        {
            currPos = motor.getSelectedSensorPosition(0);
        }

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

        if (power != prevPower)
        {
            motor.set(ControlMode.PercentOutput, power);
            prevPower = power;
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
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Boolean.toString(inverted));
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
        double pos;
        if (feedbackDeviceType == FeedbackDevice.QuadEncoder)
        {
            pos = motor.getSensorCollection().getQuadraturePosition();
        }
        else
        {
            pos = motor.getSelectedSensorPosition(0);
        }
        recordResponseCode(motor.getLastError());

        pos -= zeroPosition;

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
        double power = motor.getMotorOutputPercent();
        recordResponseCode(motor.getLastError());

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
        // double speed = motor.getSelectedSensorVelocity(0)/
        //     (motor.getStatusFramePeriod(feedbackDeviceIsPot? StatusFrameEnhanced.Status_2_Feedback0:
        //         StatusFrameEnhanced.Status_3_Quadrature, 0)/1000.0);
        //
        // The sensor velocity is in the raw sensor unit per 100 msec, adjust it to sensor unit per second.
        //
        double velocity;
        if (feedbackDeviceType == FeedbackDevice.QuadEncoder)
        {
            velocity = motor.getSensorCollection().getQuadratureVelocity() / 0.1;
        }
        else
        {
            velocity = motor.getSelectedSensorVelocity(0) / 0.1;
        }
        recordResponseCode(motor.getLastError());

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
        boolean isActive = limitSwitchesSwapped ?
            fwdLimitSwitchNormalOpen == motor.getSensorCollection().isFwdLimitSwitchClosed() :
            revLimitSwitchNormalOpen == motor.getSensorCollection().isRevLimitSwitchClosed();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Boolean.toString(isActive));
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
        boolean isActive = limitSwitchesSwapped ?
            revLimitSwitchNormalOpen == motor.getSensorCollection().isRevLimitSwitchClosed() :
            fwdLimitSwitchNormalOpen == motor.getSensorCollection().isFwdLimitSwitchClosed();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Boolean.toString(isActive));
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
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "hardware=%s", Boolean.toString(hardware));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (feedbackDeviceIsPot || !hardware)
        {
            //
            // Potentiometer has no hardware position to reset. So clear the software one.
            //
            zeroPosition = motor.getSelectedSensorPosition(0);
            recordResponseCode(motor.getLastError());
        }
        else if (hardware)
        {
            if (feedbackDeviceType == FeedbackDevice.QuadEncoder)
            {
                recordResponseCode(motor.getSensorCollection().setQuadraturePosition(0, 0));
                while (motor.getSensorCollection().getQuadraturePosition() != 0)
                {
                    Thread.yield();
                }
            }
            else
            {
                recordResponseCode(motor.setSelectedSensorPosition(0, 0, 0));
                while (motor.getSelectedSensorPosition(0) != 0)
                {
                    Thread.yield();
                }
            }
            zeroPosition = 0.0;
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

        if (value < -1.0 || value > 1.0)
        {
            throw new IllegalArgumentException("Value must be in the range of -1.0 to 1.0.");
        }

        if (softLowerLimitEnabled && value < 0.0 && getPosition() <= softLowerLimit
            || softUpperLimitEnabled && value > 0.0 && getPosition() >= softUpperLimit)
        {
            value = 0.0;
        }

        ControlMode controlMode;
        if (maxVelocity == 0.0)
        {
            controlMode = ControlMode.PercentOutput;
            prevPower = value;
        }
        else
        {
            controlMode = ControlMode.Velocity;
            value *= maxVelocity;
            value = TrcUtil.round(value); // Velocity mode is in sensor units/100ms, and sensor units are in integers.
        }
        motor.set(controlMode, value);
        recordResponseCode(motor.getLastError());

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
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enabled=%s", Boolean.toString(enabled));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        motor.setNeutralMode(enabled ? NeutralMode.Brake : NeutralMode.Coast);
        recordResponseCode(motor.getLastError());
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
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "inverted=%s", Boolean.toString(inverted));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        motor.setInverted(inverted);
        recordResponseCode(motor.getLastError());
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
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "inverted=%s", Boolean.toString(inverted));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        motor.setSensorPhase(inverted);
        recordResponseCode(motor.getLastError());
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

}   //class FrcCANTalon
