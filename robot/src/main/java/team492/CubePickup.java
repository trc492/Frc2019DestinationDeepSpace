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
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
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

import frclib.FrcPneumatic;
import frclib.FrcCANTalon;
import frclib.FrcDigitalInput;
import trclib.TrcAnalogSensor;
import trclib.TrcAnalogTrigger;
import trclib.TrcDigitalTrigger;
import trclib.TrcEvent;
import trclib.TrcRobot.RunMode;
import trclib.TrcStateMachine;
import trclib.TrcTaskMgr;
import trclib.TrcTaskMgr.TaskType;
import trclib.TrcTimer;
import trclib.TrcUtil;

import team492.Robot;

public class CubePickup
{
    private enum State
    {
        START, DELAY_SAMPLING, SAMPLE_CURRENT, DONE
    }

    private static final double[] currentThresholds =
        {RobotInfo.PICKUP_FREE_SPIN_CURRENT, RobotInfo.PICKUP_STALL_CURRENT};
    private Robot robot;
    public FrcCANTalon controlMotor, slaveMotor;
    private FrcPneumatic claw, deployer;
    private FrcDigitalInput cubeProximitySensor;
    private TrcDigitalTrigger cubeProximityTrigger;
    private TrcAnalogSensor currentSensor;
    private TrcAnalogTrigger<TrcAnalogSensor.DataType> currentTrigger;
    private TrcTaskMgr.TaskObject pickupTaskObj;
    private TrcStateMachine<State> sm;
    private TrcTimer timer;
    private TrcEvent timerEvent, currentEvent;
    private TrcEvent cubeInProximityEvent = null;
    private TrcEvent cubeInPossessionEvent = null;
    private TrcEvent currentTriggerEvent = null;
    public double startTime;

    /**
     * Initialize the CubePickup class.
     */
    public CubePickup(Robot robot)
    {
        this.robot = robot;

        controlMotor = new FrcCANTalon("LeftPickupMotor", RobotInfo.CANID_LEFT_PICKUP);
        robot.pdp.registerEnergyUsed(RobotInfo.PDP_CHANNEL_LEFT_PICKUP, "LeftPickup");
        controlMotor.setInverted(false);

        slaveMotor = new FrcCANTalon("RightPickupMotor", RobotInfo.CANID_RIGHT_PICKUP);
        robot.pdp.registerEnergyUsed(RobotInfo.PDP_CHANNEL_RIGHT_PICKUP, "RightPickup");
        slaveMotor.setInverted(true);
        slaveMotor.motor.set(ControlMode.Follower, RobotInfo.CANID_LEFT_PICKUP);

        claw = new FrcPneumatic("CubePickupClaw", RobotInfo.CANID_PCM1, RobotInfo.SOL_CUBEPICKUP_CLAW_EXTEND,
            RobotInfo.SOL_CUBEPICKUP_CLAW_RETRACT);
        deployer = new FrcPneumatic("CubePickupDeploy", RobotInfo.CANID_PCM1, RobotInfo.SOL_CUBEPICKUP_ARM_EXTEND,
            RobotInfo.SOL_CUBEPICKUP_ARM_RETRACT);
        closeClaw();
        raisePickup();

        cubeProximitySensor = new FrcDigitalInput("CubeProximitySensor", RobotInfo.DIO_CUBE_PROXIMITY_SENSOR);
        cubeProximitySensor.setInverted(true);
        cubeProximityTrigger = new TrcDigitalTrigger(
            "CubeProximityTrigger", cubeProximitySensor, this::cubeProximityEvent);

        currentSensor = new TrcAnalogSensor("pickupCurrent", this::getPickupCurrent);
        currentTrigger = new TrcAnalogTrigger<TrcAnalogSensor.DataType>(
            "PickupCurrentTrigger", currentSensor, 0, TrcAnalogSensor.DataType.RAW_DATA, currentThresholds,
            this::currentTriggerEvent);

        pickupTaskObj = TrcTaskMgr.getInstance().createTask("cubePickupTask", this::pickupTask);
        sm = new TrcStateMachine<>("pickupStateMachine");
        timer = new TrcTimer("pickupTimer");
        timerEvent = new TrcEvent("TimerEvent");
        currentEvent = new TrcEvent("CurrentUpEvent");
    }

    public void turtle()
    {
        raisePickup();
        closeClaw();
    }

    public void prepareForPickup()
    {
        deployPickup();
        openClaw();
    }

    /**
     * Open the claw.
     */
    public void openClaw()
    {
        // robot.ledIndicator.indicateHasNoCube();
        claw.retract();
    }

    /**
     * Close the claw
     */
    public void closeClaw()
    {
        claw.extend();
    }

    /**
     * Get whether the claw is opened or not.
     *
     * @return true if opened, false if not.
     */
    public boolean isClawOpen()
    {
        return claw.isExtended();
    }

    /**
     * Lift the pickup so the claw is off the ground.
     */
    public void raisePickup()
    {
        deployer.retract();
    }

    /**
     * Lower the pickup so the claw is on the ground.
     */
    public void deployPickup()
    {
        deployer.extend();
    }

    /**
     * Is the pickup up or down?
     *
     * @return Returns true if the pickup is up, (claw is off ground) and
     *         returns false if the pickup is down. (claw is on ground)
     */
    public boolean isPickupDeployed()
    {
        return deployer.isExtended();
    }

    //
    // Pickup motor methods.
    //

    private void setPickupTaskEnabled(boolean enabled)
    {
        if (enabled)
        {
            pickupTaskObj.registerTask(TaskType.POSTCONTINUOUS_TASK);
            startTime = TrcUtil.getCurrentTime();
            sm.start(State.START);
        }
        else
        {
            pickupTaskObj.unregisterTask(TaskType.POSTCONTINUOUS_TASK);
            sm.stop();
        }
    }

    public double getPickupPower()
    {
        return controlMotor.getPower() + slaveMotor.getPower();
    }

    public double getPickupCurrent()
    {
        return controlMotor.motor.getOutputCurrent() + slaveMotor.motor.getOutputCurrent();
    }

    /**
     * @return Returns true of there is a cube in the pickup
     */
    public boolean cubeInProximity()
    {
        return cubeProximitySensor.isActive();
    }

    /**
     * spin the pickup motors to the given power.
     */
    private void setPickupPower(double power, boolean userStop)
    {
        final String funcName = "setPickupPower";

        if (sm.isEnabled())
        {
            if (userStop)
            {
                robot.globalTracer.traceInfo(funcName, "User Aborted Pickup (power=%.2f)", power);
            }
            cubeProximityTrigger.setEnabled(false);
            currentTrigger.setEnabled(false);
            sm.stop();
            setPickupTaskEnabled(false);
        }
        controlMotor.set(power);
    }

    public void setPickupPower(double power)
    {
        setPickupPower(power, true);
    }

    /**
     * stops the pickup motors, use after cube has been picked up or dropped.
     * caller can call this method to abort a cube pickup sequence in progress.
     */
    public void stopPickup()
    {
        setPickupPower(0.0);
    }

    /**
     * spin the motors to push out a cube
     */
    public void dropCube(double power)
    {
        setPickupPower(-power);
        // robot.ledIndicator.indicateHasNoCube();
    }

    /**
     * spins the motors to pickup a cube and signals an event when done
     */
    public void grabCube(double power, TrcEvent event)
    {
        final String funcName = "grabCube";
        if (!sm.isEnabled())
        {
            controlMotor.set(power);
            cubeInPossessionEvent = event;
            setPickupTaskEnabled(true);
        }
        else
        {
            robot.globalTracer.traceWarn(funcName, "***** Caught double trigger *****");
        }
    }

    public void setProximityTriggerEnabled(boolean enabled, TrcEvent event)
    {
        cubeInProximityEvent = event;
        cubeProximityTrigger.setEnabled(enabled);
    }

    public void setCurrentTriggerEnabled(boolean enabled, TrcEvent currentTriggerEvent)
    {
        this.currentTriggerEvent = currentTriggerEvent;
        currentTrigger.setEnabled(enabled);
    }

    private void pickupTask(TaskType taskType, RunMode runMode)
    {
        final String funcName = "pickupTask";
        State state = sm.checkReadyAndGetState();

        if (state != null)
        {
            double elapsedTime = TrcUtil.getCurrentTime() - startTime;
            double pickupCurrent;

            switch (state)
            {
                case START:
                    // Enable both proximity and current triggers.
                    // Proximity trigger will automatically close the claws.
                    // Current trigger will detect the current spike so we can start monitoring the current value
                    // to determine if it is a startup spike or stalled in cube possession.
                    cubeProximityTrigger.setEnabled(true);
                    currentTrigger.setEnabled(true);
                    sm.waitForSingleEvent(currentEvent, State.DELAY_SAMPLING);
                    break;

                case DELAY_SAMPLING:
                    robot.globalTracer.traceInfo(funcName, "[%.3f] %s: current=%.2f",
                        elapsedTime, state, getPickupCurrent());
                    timer.set(0.3, timerEvent);
                    sm.waitForSingleEvent(timerEvent, State.SAMPLE_CURRENT);
                    break;

                case SAMPLE_CURRENT:
                    pickupCurrent = getPickupCurrent();
                    // CodeReview: Should we restore 45A since we figured out the triggers were really correct?
                    if (pickupCurrent >= 35.0) //RobotInfo.PICKUP_STALL_CURRENT) 35A. Used to be 45A.
                    {
                        robot.globalTracer.traceInfo(
                            funcName, "[%.3f] %s: Detected cube in possession (pickupCurent=%.2f)",
                            elapsedTime, state, pickupCurrent);
                        sm.setState(State.DONE);
                    }
                    else
                    {
                        robot.globalTracer.traceInfo(
                            funcName, "[%.3f] %s: Detected startup spike (pickupCurrent=%.2f)",
                            elapsedTime, state, pickupCurrent);
                        sm.setState(State.DELAY_SAMPLING);
                    }
                    break;

                case DONE:
                    // We have the cube, we can stop now.
                    setPickupPower(RobotInfo.PICKUP_HOLD_CUBE_POWER, false);
                    // robot.ledIndicator.indicateHasCube();
                    if (cubeInPossessionEvent != null)
                    {
                        cubeInPossessionEvent.set(true);
                    }
                    setPickupTaskEnabled(false);
                    cubeProximityTrigger.setEnabled(false);
                    break;
            }

            robot.traceStateInfo(elapsedTime, state.toString(), 0.0, 0.0, 0.0);
        }
    }

    private void cubeProximityEvent(boolean active)
    {
        final String funcName = "cubeProximityEvent";

        robot.globalTracer.traceInfo(funcName, "[%.3f] active=%b", TrcUtil.getCurrentTime() - startTime, active);
        if (active)
        {
            // Detected cube close by, grab it.
            closeClaw();
            if (cubeInProximityEvent != null)
            {
                cubeInProximityEvent.set(true);
            }
        }
    }

    private void currentTriggerEvent(int currZone, int prevZone, double zoneValue)
    {
        final String funcName = "currentTriggerEvent";

        robot.globalTracer.traceInfo(funcName, "[%.3f] prevZone=%d, currZone=%d, pickupCurrent=%.2f",
            TrcUtil.getCurrentTime() - startTime, prevZone, currZone, zoneValue);
        if (currZone == 1 && currZone > prevZone)
        {
            // Current is ramping up.
            currentEvent.set(true);
            if (currentTriggerEvent != null)
            {
                currentTriggerEvent.set(true);
            }
        }
    }

}
