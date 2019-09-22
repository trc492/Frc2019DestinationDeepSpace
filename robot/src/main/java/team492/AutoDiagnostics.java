package team492;

import com.revrobotics.CANSparkMax;
import frclib.FrcCANSparkMax;
import frclib.FrcCANTalon;
import hallib.HalDashboard;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTaskMgr;
import trclib.TrcTimer;

public class AutoDiagnostics implements TrcRobot.RobotCommand
{
    private static final double START_DELAY = 1.0;
    private static final String DRIVE_SPARK_CONN_SUB = "Diagnostics/DrvSparkConn";
    private static final String DRIVE_MOTOR_CONN_SUB = "Diagnostics/DrvMotorConn";
    private static final String DRIVE_ENCODER_SUB = "Diagnostics/DrvEncoder";
    private static final String[] motorNames = new String[] { "LF", "RF", "LR", "RR" };
    private static final String[] subs = new String[] { DRIVE_SPARK_CONN_SUB, DRIVE_MOTOR_CONN_SUB, DRIVE_ENCODER_SUB };
    private static final String USB_CAM_CONN_KEY = "Diagnostics/UsbCamConn";
    private static final String ELEVATOR_TALON_CONN_KEY = "Diagnostics/ElevatorTalonConn";
    private static final String ELEVATOR_MOTOR_CONN_KEY = "Diagnostics/ElevatorMotorConn";
    private static final String ELEVATOR_ENCODER_KEY = "Diagnostics/ElevatorEncoder";
    private static final String PICKUP_TALON_CONN_KEY = "Diagnostics/PickupTalonConn";
    private static final String PICKUP_MOTOR_CONN_KEY = "Diagnostics/PickupMotorConn";
    private static final String PITCH_TALON_CONN_KEY = "Diagnostics/PitchTalonConn";
    private static final String PITCH_MOTOR_CONN_KEY = "Diagnostics/PitchMotorConn";
    private static final String PITCH_ENCODER_KEY = "Diagnostics/PitchEncoder";

    enum State
    {
        START, DRIVE_CONN, ELEVATOR_TEST, PICKUP_TEST, PITCH_TEST, USB_CAM_CONN, DONE
    }

    private Robot robot;
    private TrcStateMachine<State> sm;
    private TrcEvent event;
    private TrcTimer timer;
    private FrcCANSparkMax[] motors;
    private TalonTest elevatorTest;
    private TalonTest pickupTest;
    private TalonTest pitchTest;

    public AutoDiagnostics(Robot robot)
    {
        this.robot = robot;
        motors = new FrcCANSparkMax[] { robot.leftFrontWheel, robot.rightFrontWheel, robot.leftRearWheel,
            robot.rightRearWheel };
        sm = new TrcStateMachine<>("AutoDiagnostics.sm");
        event = new TrcEvent("AutoDiagnostics.event");
        timer = new TrcTimer("AutoDiagnostics.timer");

        elevatorTest = new TalonTest(robot.elevator.getMotor(), ELEVATOR_TALON_CONN_KEY, ELEVATOR_MOTOR_CONN_KEY,
            ELEVATOR_ENCODER_KEY);
        pickupTest = new TalonTest(robot.pickup.getPickupMotor(), PICKUP_TALON_CONN_KEY, PICKUP_MOTOR_CONN_KEY);
        pitchTest = new TalonTest(robot.pickup.getPitchMotor(), PITCH_TALON_CONN_KEY, PITCH_MOTOR_CONN_KEY,
            PITCH_ENCODER_KEY);

        reset();
    }

    public void start()
    {
        sm.start(State.START);
    }

    public void reset()
    {
        // Set all values to false
        for (String s : subs)
        {
            for (String motor : motorNames)
            {
                HalDashboard.putBoolean(s + motor, false);
            }
        }

        HalDashboard.putBoolean(USB_CAM_CONN_KEY, false);
        HalDashboard.putBoolean(ELEVATOR_TALON_CONN_KEY, false);
        HalDashboard.putBoolean(ELEVATOR_MOTOR_CONN_KEY, false);
        HalDashboard.putBoolean(ELEVATOR_ENCODER_KEY, false);
        HalDashboard.putBoolean(PICKUP_TALON_CONN_KEY, false);
        HalDashboard.putBoolean(PICKUP_MOTOR_CONN_KEY, false);
        HalDashboard.putBoolean(PITCH_TALON_CONN_KEY, false);
        HalDashboard.putBoolean(PITCH_MOTOR_CONN_KEY, false);
        HalDashboard.putBoolean(PITCH_ENCODER_KEY, false);
    }

    private boolean sparkConnected(FrcCANSparkMax spark)
    {
        // slightly hacky but whatever
        return spark.motor.getFirmwareString() != null;
    }

    private boolean sparkMotorConnected(FrcCANSparkMax spark)
    {
        return !spark.motor.getFault(CANSparkMax.FaultID.kMotorFault);
    }

    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        State state = sm.checkReadyAndGetState();
        if (state != null)
        {
            switch (state)
            {
                case START:
                    reset();
                    timer.set(START_DELAY, event);
                    sm.waitForSingleEvent(event, State.DRIVE_CONN);
                    break;

                case DRIVE_CONN:
                    for (int i = 0; i < motorNames.length; i++)
                    {
                        HalDashboard.putBoolean(DRIVE_SPARK_CONN_SUB + motorNames[i], sparkConnected(motors[i]));
                        HalDashboard.putBoolean(DRIVE_MOTOR_CONN_SUB + motorNames[i], sparkMotorConnected(motors[i]));
                    }
                    sm.setState(State.ELEVATOR_TEST);
                    break;

                case ELEVATOR_TEST:
                    elevatorTest.startTest(event);
                    sm.waitForSingleEvent(event, State.PICKUP_TEST);
                    break;

                case PICKUP_TEST:
                    pickupTest.startTest(event);
                    sm.waitForSingleEvent(event, State.PITCH_TEST);
                    break;

                case PITCH_TEST:
                    pitchTest.startTest(event);
                    sm.waitForSingleEvent(event, State.USB_CAM_CONN);
                    break;

                case USB_CAM_CONN:
                    HalDashboard.putBoolean(USB_CAM_CONN_KEY, robot.camera.isConnected());
                    sm.setState(State.DONE);
                    break;

                case DONE:
                    cancel();
                    robot.elevator.zeroCalibrate();
                    robot.pickup.zeroCalibrate();
                    break;
            }
        }
        return !sm.isEnabled();
    }

    @Override
    public boolean isActive()
    {
        return sm.isEnabled();
    }

    @Override
    public void cancel()
    {
        sm.stop();
    }

    private static class TalonTest
    {
        private static final double RUN_POWER = 0.5;
        private static final double RUN_TIME = 1.5;

        private final String talonConnKey;
        private final String motorConnKey;
        private final String encoderKey;
        private TrcTaskMgr.TaskObject testTaskObj;
        private FrcCANTalon talon;
        private TrcEvent event;
        private TrcEvent onFinishedEvent;
        private TrcTimer timer;
        private boolean started = false;
        private boolean finished = false;
        private double startEncVal;
        private double runPower = RUN_POWER;
        private double runTime = RUN_TIME;

        public TalonTest(FrcCANTalon talon, String talonConnKey, String motorConnKey)
        {
            this(talon, talonConnKey, motorConnKey, null);
        }

        public TalonTest(FrcCANTalon talon, String talonConnKey, String motorConnKey, String encoderKey)
        {
            this.talonConnKey = talonConnKey;
            this.motorConnKey = motorConnKey;
            this.encoderKey = encoderKey;
            this.talon = talon;
            testTaskObj = TrcTaskMgr.getInstance().createTask(talon.toString() + ".testTask", this::testTask);
            timer = new TrcTimer(talon.toString() + ".testTimer");
            event = new TrcEvent(talon.toString() + ".testEvent");
        }

        public void startTest(TrcEvent onFinishedEvent)
        {
            if (onFinishedEvent != null)
            {
                onFinishedEvent.clear();
            }
            this.onFinishedEvent = onFinishedEvent;
            testTaskObj.registerTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
            started = false;
            finished = false;
            event.clear();
        }

        private void testTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode mode)
        {
            if (!started)
            {
                HalDashboard.putBoolean(talonConnKey, talon.motor.getBusVoltage() > 0.0);
                if (encoderKey != null)
                {
                    startEncVal = talon.getMotorPosition();
                }
                started = true;
                timer.set(runTime, event);
            }
            else if (event.isSignaled())
            {
                finished = true;
                HalDashboard.putBoolean(motorConnKey, talon.motor.getOutputCurrent() > 0.0);
                if (encoderKey != null)
                {
                    HalDashboard.putBoolean(encoderKey, talon.getMotorPosition() != startEncVal);
                }
                talon.set(0.0);
                onFinishedEvent.set(true);
                testTaskObj.unregisterTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
            }

            if (!finished)
            {
                talon.set(runPower);
            }
        }

        public void setRunPower(double runPower)
        {
            this.runPower = runPower;
        }

        public void setRunTime(double runTime)
        {
            this.runTime = runTime;
        }
    }
}
