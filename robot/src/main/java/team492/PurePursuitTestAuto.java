package team492;

import trclib.TrcEvent;
import trclib.TrcHolonomicPurePursuitDrive;
import trclib.TrcPath;
import trclib.TrcPidController;
import trclib.TrcPose2D;
import trclib.TrcRobot;
import trclib.TrcWaypoint;

import java.util.Arrays;

public class PurePursuitTestAuto implements TrcRobot.RobotCommand
{
    private static final String instanceName = "PPTest";

    private TrcHolonomicPurePursuitDrive purePursuit;
    private TrcEvent event;
    private Robot robot;
    public PurePursuitTestAuto(Robot robot)
    {
        this.robot = robot;
        TrcPidController.PidCoefficients distPid = new TrcPidController.PidCoefficients(0.011, 0, 0.001);
        TrcPidController.PidCoefficients turnPid = new TrcPidController.PidCoefficients(RobotInfo.GYRO_TURN_KP);
        TrcPidController.PidCoefficients velPid = new TrcPidController.PidCoefficients(0, 0, 0, 1.0 / 223);
        purePursuit = new TrcHolonomicPurePursuitDrive("pp", robot.driveBase, 10, 3.0, 2, distPid, turnPid,
            velPid);
        event = new TrcEvent("event");
    }

    public void start()
    {
        robot.setHalfBrakeModeEnabled(false);
//        TrcPose2D[] poses = new TrcPose2D[] { new TrcPose2D(0, 0), new TrcPose2D(0, 24, 0, 0, 60, 0),
//            new TrcPose2D(-24, 84, 0, 0, 60, 0), new TrcPose2D(-24, 108) };
        TrcPose2D[] poses = new TrcPose2D[] {
            new TrcPose2D(0,0),
            new TrcPose2D(0, 24, 0, 0, 50, 0),
            new TrcPose2D(0, 96, 180, 0, 50, 0),
            new TrcPose2D(0, 120, 180)
        };
        purePursuit
            .start(new TrcPath(true, Arrays.stream(poses).map(TrcWaypoint::new).toArray(TrcWaypoint[]::new)), event, 0);
    }

    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        return event.isSignaled();
    }

    @Override
    public boolean isActive()
    {
        return purePursuit.isActive();
    }

    @Override
    public void cancel()
    {
        robot.setHalfBrakeModeEnabled(true);
        purePursuit.cancel();
    }
}
