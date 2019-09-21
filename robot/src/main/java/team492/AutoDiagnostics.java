package team492;

import com.revrobotics.CANSparkMax;
import frclib.FrcCANSparkMax;
import hallib.HalDashboard;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;
import trclib.TrcUtil;

import java.util.Arrays;

public class AutoDiagnostics implements TrcRobot.RobotCommand
{
    private static final double DRIVE_POWER = 0.3;
    private static final double DRIVE_TIME = 0.3;
    private static final double ENCODER_PERCENT_ERR = 0.08; // 8 percent
    private static final double START_DELAY = 0.5;
    private static final String DRIVE_SPARK_CONN_SUB = "Diagnostics/DrvSparkConn";
    private static final String DRIVE_MOTOR_CONN_SUB = "Diagnostics/DrvMotorConn";
    private static final String DRIVE_ENCODER_SUB = "Diagnostics/DrvEncoder";
    private static final String[] motorNames = new String[] { "LF", "RF", "LR", "RR" };
    private static final String[] subs = new String[] { DRIVE_SPARK_CONN_SUB, DRIVE_MOTOR_CONN_SUB, DRIVE_ENCODER_SUB };
    private static final String USB_CAM_CONN_KEY = "Diagnostics/UsbCamConn";

    enum State
    {
        START, DRIVE_SPARK_CONN, DRIVE_MOTOR_CONN, DRIVE_ENCODER_START, DRIVE_ENCODER_END, USB_CAM_CONN
    }

    private Robot robot;
    private TrcStateMachine<State> sm;
    private TrcEvent event;
    private TrcTimer timer;
    private FrcCANSparkMax[] motors;

    public AutoDiagnostics(Robot robot)
    {
        this.robot = robot;
        motors = new FrcCANSparkMax[] { robot.leftFrontWheel, robot.rightFrontWheel, robot.leftRearWheel,
            robot.rightRearWheel };
        sm = new TrcStateMachine<>("AutoDiagnostics.sm");
        event = new TrcEvent("AutoDiagnostics.event");
        timer = new TrcTimer("AutoDiagnostics.timer");

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
    }

    private boolean isConn(FrcCANSparkMax spark)
    {
        // slightly hacky but whatever
        return spark.motor.getFirmwareString() != null;
    }

    private boolean motorConn(FrcCANSparkMax spark)
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
                    sm.waitForSingleEvent(event, State.DRIVE_SPARK_CONN);
                    break;

                case DRIVE_SPARK_CONN:
                    for (int i = 0; i < motorNames.length; i++)
                    {
                        HalDashboard.putBoolean(DRIVE_SPARK_CONN_SUB + motorNames[i], isConn(motors[i]));
                    }
                    sm.setState(State.DRIVE_MOTOR_CONN);
                    break;

                case DRIVE_MOTOR_CONN:
                    for (int i = 0; i < motorNames.length; i++)
                    {
                        HalDashboard.putBoolean(DRIVE_MOTOR_CONN_SUB + motorNames[i], motorConn(motors[i]));
                    }
                    sm.setState(State.DRIVE_ENCODER_START);
                    break;

                case DRIVE_ENCODER_START:
                    Arrays.stream(motors).forEach(m -> m.resetPosition(true));
                    Arrays.stream(motors).forEach(m -> m.set(DRIVE_POWER));
                    timer.set(DRIVE_TIME, event);
                    sm.waitForSingleEvent(event, State.DRIVE_ENCODER_END);
                    break;

                case DRIVE_ENCODER_END:
                    Arrays.stream(motors).forEach(m -> m.set(0.0));
                    double[] encoders = Arrays.stream(motors).mapToDouble(FrcCANSparkMax::getPosition).toArray();
                    double mean = TrcUtil.average(encoders);
                    double tolerance = mean * ENCODER_PERCENT_ERR;
                    for (int i = 0; i < encoders.length; i++)
                    {
                        HalDashboard.putBoolean(DRIVE_ENCODER_SUB + motorNames[i],
                            TrcUtil.inRange(encoders[i], mean - tolerance, mean + tolerance));
                    }
                    sm.setState(State.USB_CAM_CONN);
                    break;

                case USB_CAM_CONN:
                    HalDashboard.putBoolean(USB_CAM_CONN_KEY, robot.camera.isConnected());
                    cancel();
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
}
