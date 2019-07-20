package team492;

import hallib.HalDashboard;
import trclib.TrcLoopTimeCounter;
import trclib.TrcPidController;
import trclib.TrcRobot;
import trclib.TrcTaskMgr;
import trclib.TrcUtil;

public class VisionStuff
{
    private static final double X_TAPER_LOW = 0;
    private static final double X_TAPER_HIGH = 25;

    private Robot robot;
    private TrcTaskMgr.TaskObject taskObj;
    private TrcPidController xPid, yPid, turnPid;
    private TrcLoopTimeCounter loopTimer;
    private boolean active = false;

    public VisionStuff(Robot robot)
    {
        this.robot = robot;
        taskObj = TrcTaskMgr.getInstance().createTask("VisionPid", this::pidTask);
        xPid = new TrcPidController("xpid", new TrcPidController.PidCoefficients(0.025), 1, robot.vision.getVision()::getX);
        yPid = new TrcPidController("ypid", new TrcPidController.PidCoefficients(0.0025), 1, robot.vision.getVision()::getY);
        turnPid = new TrcPidController("turnpid", new TrcPidController.PidCoefficients(0.004), 1,
            robot.driveBase::getHeading);
        xPid.setAbsoluteSetPoint(true);
        xPid.setInverted(true);
        yPid.setAbsoluteSetPoint(true);
        yPid.setInverted(true);
        turnPid.setAbsoluteSetPoint(true);
        turnPid.setOutputLimit(0.5);
        loopTimer = new TrcLoopTimeCounter(1.0);

        HalDashboard.putNumber("xout", 0.0);
        HalDashboard.putNumber("visionX", 0.0);
        HalDashboard.putNumber("scale", 0.0);
        HalDashboard.putNumber("lf", robot.leftFrontWheel.getPower());
        HalDashboard.putNumber("rf", robot.rightFrontWheel.getPower());
        HalDashboard.putNumber("lr", robot.leftRearWheel.getPower());
        HalDashboard.putNumber("rr", robot.rightRearWheel.getPower());
    }

    public void start()
    {
        xPid.setTarget(0.0);
        yPid.setTarget(0.0);
        turnPid.setTarget(0.0); // TODO: nearest scoring angle
        taskObj.registerTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        active = true;
    }

    public void stop()
    {
        taskObj.unregisterTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        robot.driveBase.stop();
        active = false;
    }

    public boolean isActive()
    {
        return active;
    }

    private double getTurnOutputScale()
    {
        double x = Math.abs(robot.vision.getVision().getX());
        if (x > X_TAPER_HIGH)
        {
            return 0;
        }
        else if (x > X_TAPER_LOW)
        {
            return 1 - TrcUtil.scaleRange(x, X_TAPER_LOW, X_TAPER_HIGH, 0, 1);
        }
        return 1;
    }

    private void pidTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        double xOut = xPid.getOutput();
        double scale = getTurnOutputScale();
        double yOut = yPid.getOutput() * scale;
        double rotOut = turnPid.getOutput() * scale;
        HalDashboard.putNumber("visionX", robot.vision.getVision().getX());
        HalDashboard.putNumber("xout", xOut);
        HalDashboard.putNumber("scale", scale);
        HalDashboard.putNumber("lf", robot.leftFrontWheel.getPower());
        HalDashboard.putNumber("rf", robot.rightFrontWheel.getPower());
        HalDashboard.putNumber("lr", robot.leftRearWheel.getPower());
        HalDashboard.putNumber("rr", robot.rightRearWheel.getPower());
        loopTimer.update();
        robot.driveBase.holonomicDrive(xOut, yOut, rotOut);
        robot.dashboard.displayPrintf(14, "x=%.2f,y=%.2f,period=%.3f", robot.vision.getVision().getX(),
            robot.vision.getVision().getY(), loopTimer.getPeriod());
    }
}
