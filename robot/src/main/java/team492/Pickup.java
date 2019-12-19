package team492;

import frclib.FrcCANSparkMax;
import trclib.TrcAnalogSensor;
import trclib.TrcAnalogSensorTrigger;
import trclib.TrcEvent;
import trclib.TrcTimer;

public class Pickup
{
    private FrcCANSparkMax motor;
    private TrcAnalogSensorTrigger<TrcAnalogSensor.DataType> pickupTrigger, deployTrigger;
    private TrcTimer timer;
    private TrcEvent timerEvent;

    public Pickup()
    {
        motor = new FrcCANSparkMax("PickupMotor", RobotInfo.CANID_PICKUP, true);
        motor.setBrakeModeEnabled(true);
        motor.setInverted(true);

        TrcAnalogSensor currentSensor = new TrcAnalogSensor("CurrentSensor", motor.motor::getOutputCurrent);

        pickupTrigger = new TrcAnalogSensorTrigger<>("PickupTrigger", currentSensor, 0,
            TrcAnalogSensor.DataType.RAW_DATA, RobotInfo.PICKUP_CURR_THRESH, this::pickupEvent, false);
        deployTrigger = new TrcAnalogSensorTrigger<>("DeployTrigger", currentSensor, 0,
            TrcAnalogSensor.DataType.RAW_DATA, RobotInfo.DEPLOY_CURR_THRESH, this::deployEvent, false);

        timer = new TrcTimer("PickupTimer");
        timerEvent = new TrcEvent("TimerEvent");
    }

    public FrcCANSparkMax getMotor()
    {
        return motor;
    }

    private void pickupEvent(int currZone, int prevZone, double value)
    {
        if (timerEvent.isSignaled() && currZone == 1 && prevZone == 0)
        {
            setPower(0.0);
        }
    }

    private void deployEvent(int currZone, int prevZone, double value)
    {
        if (currZone == 0 && prevZone == 1)
        {
            setPower(0.0);
        }
    }

    private void reset()
    {
        timer.cancel();
        timerEvent.clear();
        pickupTrigger.setEnabled(false);
        deployTrigger.setEnabled(false);
    }

    public void setPower(double power)
    {
        reset();
        motor.set(power);
    }

    public void deploy()
    {
        reset();
        deployTrigger.setEnabled(true);
        motor.set(RobotInfo.DEPLOY_POWER);
    }

    public void pickup()
    {
        reset();
        pickupTrigger.setEnabled(true);
        timer.set(RobotInfo.PICKUP_BLIND_PERIOD, timerEvent);
        motor.set(RobotInfo.PICKUP_POWER);
    }
}
