package team492;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.text.SimpleDateFormat;
import java.util.Date;

import common.CmdTimedDrive;
import trclib.TrcUtil;
import trclib.TrcRobot.RobotCommand;

public class RobotStats implements RobotCommand
{
    private static final double RUN_TIME = 2.5;

    private CmdTimedDrive timedDrive;
    private Robot robot;
    private PrintStream fileOut;
    private double startTime;
    public RobotStats(Robot robot)
    {
        this.robot = robot;
    }
    
    public void start()
    {
        timedDrive = new CmdTimedDrive(robot, 0.0, RUN_TIME, 0.0, 1.0, 0.0);
        startTime = TrcUtil.getCurrentTime();
        try
        {
            File dir = new File("/home/lvuser/TimeDriveLogs");
            dir.mkdir();
            String timeStamp = new SimpleDateFormat("dd-MM-yy_HHmm").format(new Date());
            fileOut = new PrintStream(new FileOutputStream(new File(dir, timeStamp + "_log.csv")));
            fileOut.println("Time,LeftFrontPos,RightFrontPos,LeftRearPos,RightRearPos");
        } catch (IOException e)
        {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    public void stop()
    {
        fileOut.close();
        fileOut = null;
        timedDrive = null;
    }
    
    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        timedDrive.cmdPeriodic(elapsedTime);
        if(fileOut != null)
        {
            double time = TrcUtil.getCurrentTime() - startTime;
            double scale = RobotInfo.ENCODER_Y_INCHES_PER_COUNT;
            double lfPos = robot.leftFrontWheel.getPosition() * scale;
            double rfPos = robot.rightFrontWheel.getPosition() * scale;
            double lrPos = robot.leftRearWheel.getPosition() * scale;
            double rrPos = robot.rightRearWheel.getPosition() * scale;
            
            fileOut.printf("%.2f,%.2f,%.2f,%.2f,%.2f\n", time, lfPos, rfPos, lrPos, rrPos);
        }
        return false;
    }

}
