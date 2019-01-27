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

import frclib.FrcCANSparkMax;
import frclib.FrcCANTalon;
import trclib.TrcPidActuator;
import trclib.TrcPidController;
import trclib.TrcUtil;

public class Pickup
{

    private FrcCANSparkMax pickupMotor;
    private TrcPidActuator pitchController;
    private FrcCANTalon pitchMotor;

    public Pickup()
    {
        pickupMotor = new FrcCANSparkMax("PickupMaster", RobotInfo.CANID_PICKUP, false);

        // Set opposite directions
        pickupMotor.setInverted(false);

        // We don't really need brakes
        pickupMotor.setBrakeModeEnabled(false);

        pitchMotor = new FrcCANTalon("PickupPitchMotor", RobotInfo.CANID_PICKUP_PITCH);
        pitchMotor.setBrakeModeEnabled(true);
        pitchMotor.setInverted(false);
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
            () -> RobotInfo.PICKUP_GRAVITY_COMP);
        pitchController.setPositionScale(RobotInfo.PICKUP_DEGREES_PER_COUNT, RobotInfo.PICKUP_MIN_POS);
        pitchController.setStallProtection(RobotInfo.PICKUP_STALL_MIN_POWER, RobotInfo.PICKUP_STALL_TIMEOUT,
            RobotInfo.PICKUP_STALL_RESET_TIMEOUT);
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
