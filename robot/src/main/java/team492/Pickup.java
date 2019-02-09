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
import frclib.FrcDigitalInput;
import frclib.FrcPneumatic;
import trclib.TrcAnalogSensor;
import trclib.TrcAnalogTrigger;
import trclib.TrcDigitalTrigger;
import trclib.TrcEvent;
import trclib.TrcPidActuator;
import trclib.TrcPidController;
import trclib.TrcUtil;

public class Pickup
{
    private static final String instanceName = "Pickup";

    private static double[] currentThresholds = new double[] { RobotInfo.PICKUP_FREE_SPIN_CURRENT,
        RobotInfo.PICKUP_STALL_CURRENT };

    private enum State
    {
        START, MONITOR, DONE
    }

    private FrcCANTalon pickupMotor;
    private FrcCANTalon pitchMotor;
    private TrcPidActuator pitchController;
    private FrcPneumatic hatchDeployer;
    private FrcDigitalInput cargoSensor;
    private TrcDigitalTrigger cargoTrigger;
    private TrcAnalogSensor currentSensor;
    private TrcAnalogTrigger<TrcAnalogSensor.DataType> currentTrigger;
    private TrcEvent onFinishedEvent;
    private Robot robot;

    public Pickup(Robot robot)
    {
        this.robot = robot;

        pickupMotor = new FrcCANTalon("PickupMaster", RobotInfo.CANID_PICKUP);
        pickupMotor.setInverted(true);                         // Set opposite directions.
        pickupMotor.setBrakeModeEnabled(false);                 // We don't really need brakes
        pickupMotor.motor.overrideLimitSwitchesEnable(false);   // No limit switches, make sure they are disabled.

        pitchMotor = new FrcCANTalon("PickupPitchMotor", RobotInfo.CANID_PICKUP_PITCH);
        pitchMotor.setInverted(false);
        pitchMotor.setBrakeModeEnabled(true);
        pitchMotor.motor.overrideLimitSwitchesEnable(false); // for debugging only
        pitchMotor.configFwdLimitSwitchNormallyOpen(false);
        pitchMotor.configRevLimitSwitchNormallyOpen(false);

        // TODO: Tune ALL of these constants
        TrcPidController.PidCoefficients pidCoefficients = new TrcPidController.PidCoefficients(RobotInfo.PICKUP_KP,
            RobotInfo.PICKUP_KI, RobotInfo.PICKUP_KD);
        TrcPidController pidController = new TrcPidController("PickupPidController", pidCoefficients,
            RobotInfo.PICKUP_TOLERANCE, this::getPickupAngle);
        pitchController = new TrcPidActuator("PICKUPActuator", pitchMotor, pidController,
            RobotInfo.PICKUP_CALIBRATE_POWER, RobotInfo.PICKUP_PID_FLOOR, RobotInfo.PICKUP_PID_CEILING,
            () -> RobotInfo.PICKUP_GRAVITY_COMP);   // CodeReview: TODO: This should not be a constant.
        pitchController.setPositionScale(RobotInfo.PICKUP_DEGREES_PER_COUNT, RobotInfo.PICKUP_MIN_POS);
        pitchController.setStallProtection(RobotInfo.PICKUP_STALL_MIN_POWER, RobotInfo.PICKUP_STALL_TIMEOUT,
            RobotInfo.PICKUP_STALL_RESET_TIMEOUT);

        cargoSensor = new FrcDigitalInput(instanceName + ".cargoSensor", RobotInfo.DIO_CARGO_PROXIMITY_SENSOR);
        cargoSensor.setInverted(false);

        cargoTrigger = new TrcDigitalTrigger(instanceName + ".cargoTrigger", cargoSensor, this::cargoDetectedEvent);
        cargoTrigger.setEnabled(false);

        hatchDeployer = new FrcPneumatic(instanceName + ".hatchDeployer", RobotInfo.CANID_PCM1,
            RobotInfo.SOL_HATCH_DEPLOYER_EXTEND, RobotInfo.SOL_HATCH_DEPLOYER_RETRACT);

        currentSensor = new TrcAnalogSensor(instanceName + ".pickupCurrent",
            () -> pickupMotor.motor.getOutputCurrent());
        currentTrigger = new TrcAnalogTrigger<>(instanceName + ".currentTrigger", currentSensor, 0,
            TrcAnalogSensor.DataType.RAW_DATA, currentThresholds, this::currentTriggerEvent);
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

    public void cancel()
    {
        setPickupPower(0.0);
        setPitchPower(0.0);

        if (onFinishedEvent != null)
        {
            onFinishedEvent.set(true);
        }
    }

    public void deployCargo(TrcEvent event)
    {
        if (event != null)
        {
            event.clear();
        }
        onFinishedEvent = event;
        cargoTrigger.setEnabled(false); // make sure the cargo trigger is disabled
        currentTrigger.setEnabled(true);
        setPickupPower(-0.7);
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
        if (cargoSensor.isActive())
        {
            // Return early if we already have a cargo
            event.set(true);
        }
        else
        {
            // The cargo trigger will signal the event when it detects the cargo
            if (event != null)
            {
                event.clear();
            }
            this.onFinishedEvent = event;
            currentTrigger.setEnabled(false); // make sure the current trigger is disabled
            cargoTrigger.setEnabled(true);
            setPickupPower(1.0);
        }
    }

    public void pickupHatch(TrcEvent event)
    {
        // Since this is literally driving into the hatch panel, it's already finished.
        event.set(true);
    }

    public void setManualOverrideEnabled(boolean enabled)
    {
        pitchController.setManualOverride(enabled);
    }

    public void zeroCalibrate()
    {
        pitchController.zeroCalibrate();
    }

    public double getPickupAngle()
    {
        return pitchController.getPosition();
    }

    public void setPickupAngle(double angle)
    {
        pitchController.setTarget(angle, true);
    }

    public void setPitchPower(double power)
    {
        setPitchPower(power, true);
    }

    public void setPitchPower(double power, boolean hold)
    {
        pitchMotor.set(power);
        //        power = TrcUtil.clipRange(power, -1.0, 1.0);
        //        pitchController.setPower(power, hold);
    }

    public void setPickupPower(double power)
    {
        power = TrcUtil.clipRange(power, -1.0, 1.0);
        pickupMotor.set(power);
    }
}
