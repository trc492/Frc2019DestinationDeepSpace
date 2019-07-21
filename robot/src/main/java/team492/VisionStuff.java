package team492;

import hallib.HalDashboard;
import trclib.TrcLoopTimeCounter;
import trclib.TrcPidController;
import trclib.TrcRobot;
import trclib.TrcTaskMgr;
import trclib.TrcUtil;

public class VisionStuff
{

    private Robot robot;
    private TrcTaskMgr.TaskObject taskObj;
    private TrcPidController xPid, yPid, turnPid;
    private TrcLoopTimeCounter loopTimer;
    private boolean active = false;
    private boolean usePidY = true;
    private double lastElevatorPower = 0;

    public VisionStuff(Robot robot)
    {
        this.robot = robot;
        taskObj = TrcTaskMgr.getInstance().createTask("VisionPid", this::pidTask);
        xPid = new TrcPidController("xpid", new TrcPidController.PidCoefficients(0.025), 1, this::getHeading);
        yPid = new TrcPidController("ypid", new TrcPidController.PidCoefficients(0.0065), 1, this::getDistance);
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
        HalDashboard.putNumber("offset", 0);
        HalDashboard.putNumber("XWithOffset", 0);
    }

    public void setAutomaticY(boolean enabled)
    {
        usePidY = enabled;
    }

    public void start(boolean deployAtAngle)
    {
        xPid.setTarget(0.0);
        yPid.setTarget(13);
        turnPid.setTarget(robot.getNearestScoringAngle(deployAtAngle));
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

    public double getHeading()
    {
        if (!robot.vision.getVision().targetDetected())
        {
            return 0.0;
        }
        double offset = Math.toDegrees(Math.atan2(1, getDistance()));
        double corrected = robot.vision.getVision().getHeading() + offset;
        HalDashboard.putNumber("offset", offset);
        HalDashboard.putNumber("XWithOffset", corrected);
        return corrected;
    }

    private double getDistance()
    {
        return robot.vision.getVision().getTargetDepth();
    }

    private double getOutputScale(double taperLow, double taperHigh)
    {
        double x = Math.abs(getHeading());
        if (x > taperHigh)
        {
            return 0;
        }
        else if (x > taperLow)
        {
            return 1 - TrcUtil.scaleRange(x, taperLow, taperHigh, 0, 1);
        }
        return 1;
    }

    private void pidTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        double xOut = xPid.getOutput();
        double yOut;
        if (usePidY)
        {
            yOut = yPid.getOutput() * getOutputScale(0, 25);
        }
        else
        {
            yOut = robot.rightDriveStick.getYWithDeadband(true);
        }
        double rotOut = turnPid.getOutput() * getOutputScale(0, 20);
        HalDashboard.putNumber("visionX", robot.vision.getVision().getHeading());
        HalDashboard.putNumber("xout", xOut);
        loopTimer.update();
        robot.driveBase.holonomicDrive(xOut, yOut, rotOut);

        if (!usePidY)
        {
            double elevPower = robot.operatorStick.getYWithDeadband(true);
            if (elevPower != lastElevatorPower)
            {
                robot.elevator.setPower(elevPower);
                lastElevatorPower = elevPower;
            }
        }
    }
}
