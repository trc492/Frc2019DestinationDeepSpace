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
import frclib.FrcDigitalInput;
import frclib.FrcPdp;
import frclib.FrcPneumatic;
import trclib.TrcAnalogSensor;
import trclib.TrcAnalogTrigger;
import trclib.TrcDigitalTrigger;
import trclib.TrcEvent;
import trclib.TrcPidActuator;
import trclib.TrcPidController;
import trclib.TrcTimer;
import trclib.TrcUtil;

public class Pickup
{
    private static final String instanceName = "Pickup";

    private static double[] currentThresholds = new double[] { RobotInfo.PICKUP_CURRENT_THRESHOLD };

    private Robot robot;
    private FrcCANTalon pickupMotor;
    private FrcCANTalon pitchMotor;
    private TrcPidActuator pitchController;
    private FrcPneumatic hatchDeployer;
    private FrcDigitalInput cargoSensor;
    private TrcDigitalTrigger cargoTrigger;
    private TrcAnalogTrigger<TrcAnalogSensor.DataType> currentTrigger;
    private TrcPidController pitchPidController;
    private TrcEvent onFinishedEvent;
    private TrcTimer timer;
    private boolean manualOverrideEnabled;
    private FrcPneumatic alignGuide;

    public Pickup(Robot robot)
    {
        this.robot = robot;

        pickupMotor = new FrcCANTalon("PickupMotor", RobotInfo.CANID_PICKUP);
        pickupMotor.setInverted(true);                          // Set opposite directions.
        pickupMotor.setBrakeModeEnabled(false);                 // We don't really need brakes
        pickupMotor.motor.overrideLimitSwitchesEnable(false);   // No limit switches, make sure they are disabled.

        pitchMotor = new FrcCANTalon("PickupPitchMotor", RobotInfo.CANID_PICKUP_PITCH);
        pitchMotor.setInverted(false);
        pitchMotor.setBrakeModeEnabled(true);
        pitchMotor.motor.overrideLimitSwitchesEnable(true);
        pitchMotor.configFwdLimitSwitchNormallyOpen(false);
        pitchMotor.configRevLimitSwitchNormallyOpen(false);

        robot.pdp.registerEnergyUsed(
            new FrcPdp.Channel(RobotInfo.PDP_CHANNEL_PICKUP, "Pickup"),
            new FrcPdp.Channel(RobotInfo.PDP_CHANNEL_PICKUP_PITCH, "PickupPitch"));


        // TODO: Tune ALL of these constants
        TrcPidController.PidCoefficients pidCoefficients = new TrcPidController.PidCoefficients(RobotInfo.PICKUP_KP,
            RobotInfo.PICKUP_KI, RobotInfo.PICKUP_KD);
        pitchPidController = new TrcPidController("PickupPidController", pidCoefficients, RobotInfo.PICKUP_TOLERANCE,
            this::getPickupAngle);
        FrcCANTalonLimitSwitch lowerLimitSwitch = new FrcCANTalonLimitSwitch("PitchLowerSwitch", pitchMotor, false);
        pitchController = new TrcPidActuator("PickupActuator", pitchMotor, lowerLimitSwitch, pitchPidController,
            RobotInfo.PICKUP_CALIBRATE_POWER, RobotInfo.PICKUP_PID_FLOOR, RobotInfo.PICKUP_PID_CEILING,
            this::getGravityCompensation);
        pitchController.setPositionScale(RobotInfo.PICKUP_DEGREES_PER_COUNT, RobotInfo.PICKUP_MIN_POS);
        pitchController.setStallProtection(RobotInfo.PICKUP_STALL_MIN_POWER, RobotInfo.PICKUP_STALL_TIMEOUT,
            RobotInfo.PICKUP_STALL_RESET_TIMEOUT);

        cargoSensor = new FrcDigitalInput(instanceName + ".cargoSensor", RobotInfo.DIO_CARGO_PROXIMITY_SENSOR);
        cargoSensor.setInverted(false);

        cargoTrigger = new TrcDigitalTrigger(instanceName + ".cargoTrigger", cargoSensor, this::cargoDetectedEvent);
        cargoTrigger.setEnabled(false);

        hatchDeployer = new FrcPneumatic(instanceName + ".hatchDeployer", RobotInfo.CANID_PCM1,
            RobotInfo.SOL_HATCH_DEPLOYER_EXTEND, RobotInfo.SOL_HATCH_DEPLOYER_RETRACT);

        alignGuide = new FrcPneumatic(instanceName + ".alignGuide", RobotInfo.CANID_PCM1, RobotInfo.SOL_ALIGN_GUIDES);

        TrcAnalogSensor currentSensor = new TrcAnalogSensor(instanceName + ".pickupCurrent", this::getPickupCurrent);
        currentTrigger = new TrcAnalogTrigger<>(instanceName + ".currentTrigger", currentSensor, 0,
            TrcAnalogSensor.DataType.RAW_DATA, currentThresholds, this::currentTriggerEvent, false);

        timer = new TrcTimer(instanceName + ".timer");
    }

    private void currentTriggerEvent(int currZone, int prevZone, double value)
    {
        robot.globalTracer.traceInfo(instanceName + ".currentTriggerEvent",
            "Current edge event detected! currZone=%d,prevZone=%d,value=%.2f", currZone, prevZone, value);

        // If the current goes from stalling to free spinning, we just ejected a cargo
        if (currZone == 0 && prevZone > currZone)
        {
            if (onFinishedEvent != null)
            {
                onFinishedEvent.set(true);
            }
            onFinishedEvent = null;
            setPickupPower(0.0);
            currentTrigger.setEnabled(false);
        }
    }

    private void cargoDetectedEvent(boolean active)
    {
        robot.globalTracer
            .traceInfo(instanceName + ".cargoDetectedEvent", "Cargo edge event detected! active=%b", active);

        if (active)
        {
            if (onFinishedEvent != null)
            {
                onFinishedEvent.set(true);
            }
            onFinishedEvent = null;
            setPickupPower(0.0);
            cargoTrigger.setEnabled(false);
        }
    }

    /**
     * This method calculates the gravitation pull on the pickup endeffector in terms of percentage motor power.
     * Meaning it will return a power value that will hold the endeffector suspend in any valid position. Since
     * gravitational pull of the endeffector depends on its angle, this is proportional to the sine of the
     * endeffector angle (endeffector is at zero degree when it is in the vertical position).
     * EndEffectorMaxTorqueAtFulcrum = EndEffectorWeight * EndEffectorCGDistanceFromFulcrum
     * PercentageMotorPower = sin(EndEffectorAngle) * EndEffectorMaxTorqueAtFulcrum /
     * (MotorStallTorque * GearRatio)
     *
     * @return gravity compensation value.
     */
    private double getGravityCompensation()
    {
        // This needs to be negative since negative = up
        return -Math.sin(Math.toRadians(getPickupAngle())) * RobotInfo.PICKUP_PERCENT_TORQUE;
    }

    public double getPitchPower()
    {
        return pitchMotor.getPower();
    }


    public void extendAlignGuides()
    {
        alignGuide.extend();
    }

    public void retractAlignGuides()
    {
        alignGuide.retract();
    }

    public boolean isUpperLimitSwitchActive()
    {
        return pitchMotor.isUpperLimitSwitchActive();
    }

    public boolean isLowerLimitSwitchActive()
    {
        return pitchMotor.isLowerLimitSwitchActive();
    }

    public double getPickupCurrent()
    {
        return pickupMotor.motor.getOutputCurrent();
    }

    public boolean cargoDetected()
    {
        return cargoSensor.isActive();
    }

    public void cancel()
    {
        setPickupPower(0.0);
        setPitchPower(0.0);

        if (onFinishedEvent != null)
        {
            onFinishedEvent.set(true);
        }

        currentTrigger.setEnabled(false);
        cargoTrigger.setEnabled(false);
        timer.cancel();
    }

    public void deployCargo(TrcEvent event)
    {
        if (!manualOverrideEnabled)
        {
            if (event != null)
            {
                event.clear();
            }
            onFinishedEvent = event;
            cargoTrigger.setEnabled(false); // make sure the cargo trigger is disabled
            currentTrigger.setEnabled(true);
        }
        setPickupPower(RobotInfo.PICKUP_CARGO_DEPLOY_POWER);
    }

    public void deployHatch(TrcEvent event)
    {
        if (event != null)
        {
            event.clear();
        }
        hatchDeployer.timedExtend(1.0, 0.0, event);
    }

    public void pickupCargo(TrcEvent event)
    {
        if (manualOverrideEnabled)
        {
            setPickupPower(RobotInfo.PICKUP_CARGO_PICKUP_POWER);
        }
        else
        {
            if (event != null)
            {
                event.clear();
            }

            if (cargoDetected())
            {
                // Return early if we already have a cargo
                if (event != null)
                {
                    event.set(true);
                }
            }
            else
            {
                if (event != null)
                {
                    // The timer will signal the event when it expires. This is a backup in case the sensor fails.
                    // Just call the trigger method when the timer expires. Only do this if we have an event to trigger.
                    timer.cancel();
                    timer.set(RobotInfo.PICKUP_CARGO_PICKUP_TIMEOUT, e -> cargoDetectedEvent(true));
                }
                this.onFinishedEvent = event;
                currentTrigger.setEnabled(false); // make sure the current trigger is disabled
                cargoTrigger.setEnabled(true); // The cargo trigger will signal the event when it detects the cargo
                setPickupPower(RobotInfo.PICKUP_CARGO_PICKUP_POWER);
            }
        }
    }

    public void pickupHatch(TrcEvent event)
    {
        // Since this is literally driving into the hatch panel, it's already finished.
        if (event != null)
        {
            event.set(true);
        }
    }

    public void extendHatchDeployer()
    {
        hatchDeployer.extend();
    }

    public void retractHatchDeployer()
    {
        hatchDeployer.retract();
    }

    public void setManualOverrideEnabled(boolean enabled)
    {
        this.manualOverrideEnabled = enabled;
        pitchController.setManualOverride(enabled);
    }

    public void zeroCalibrate()
    {
        pitchController.zeroCalibrate();
    }

    /**
     * Get the angle of the pickup. The angle is relative to vertical.
     *
     * @return The angle in degrees.
     */
    public double getPickupAngle()
    {
        return pitchController.getPosition();
    }

    public double getRawPickupAngle()
    {
        return pitchMotor.getPosition();
    }

    /**
     * Set the angle of the pickup pitch. The angle is relative to vertical.
     *
     * @param angle The angle in degrees to set.
     */
    public void setPickupAngle(double angle)
    {
        pitchController.setTarget(angle, true);
    }

    public void setPickupAngle(double angle, TrcEvent onFinishedEvent)
    {
        setPickupAngle(angle, onFinishedEvent, 0.0);
    }

    public void setPickupAngle(double angle, TrcEvent onFinishedEvent, double timeout)
    {
        pitchController.setTarget(angle, onFinishedEvent, timeout);
    }

    public TrcPidController getPitchPidController()
    {
        return pitchPidController;
    }

    /**
     * Set the power to the pitch motor. Positive power is a greater angle relative to vertical.
     *
     * @param power Power to set to the motor, in the range [-1,1].
     */
    public void setPitchPower(double power)
    {
        setPitchPower(power, true);
    }

    /**
     * Set the power to the pitch motor. Positive power is a greater angle relative to vertical.
     *
     * @param power Power to set to the motor, in the range [-1,1].
     * @param hold  True to hold the position when power is zero.
     */
    public void setPitchPower(double power, boolean hold)
    {
        power = TrcUtil.clipRange(power, -1.0, 1.0);
        if (pitchController.isManualOverride())
        {
            pitchController.cancel();
            pitchMotor.set(power);
        }
        else
        {
            pitchController.setPower(power, hold);
        }
    }

    public void setPickupPower(double power)
    {
        power = TrcUtil.clipRange(power, -1.0, 1.0);
        pickupMotor.set(power);
    }
}
