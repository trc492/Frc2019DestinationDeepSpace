/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
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

package frclib;

import java.io.InputStream;
import java.io.IOException;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.hal.FRCNetComm.tInstances;
import edu.wpi.first.wpilibj.hal.FRCNetComm.tResourceType; 
import edu.wpi.first.wpilibj.hal.HAL;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import hallib.HalDashboard;
import hallib.HalDbgLog;
import trclib.TrcDbgTrace;
import trclib.TrcRobot;
import trclib.TrcRobot.*;
import trclib.TrcTaskMgr;
import trclib.TrcUtil;

/**
 * This class defines and implements the FrcRobotBase object. The FrcRobotBase object implements a cooperative
 * multitasking robot. Different subsystems register themselves as CoopTasks. FrcRobotBase uses the TaskMgr to
 * task switch between different subsystem tasks at various points in the robot loop. This basically simulates
 * a cooperative multitasking scheduler that task switches between them in different modes.
 */
public abstract class FrcRobotBase extends RobotBase
{
    protected static final String moduleName = "FrcRobotBase";
    protected static final boolean debugEnabled = false;
    protected static final boolean tracingEnabled = false;
    protected static final boolean useGlobalTracer = false;
    protected static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    protected static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    protected TrcDbgTrace dbgTrace = null;
    protected boolean liveWindowEnabled = false;

    private static final boolean dashboardEnabled = true;

    /**
     * This method is called to initialize the robot.
     */
    public abstract void robotInit();

    /**
     * This method is called to prepare the robot before a robot mode is about to start.
     *
     * @param runMode specifies the current run mode.
     * @param prevMode specifies the previous run mode.
     */
    public abstract void robotStartMode(RunMode runMode, RunMode prevMode);

    /**
     * This method is called to prepare the robot right after a robot mode has been stopped.
     *
     * @param runMode specifies the current run mode.
     * @param nextMode specifies the next run mode.
     */
    public abstract void robotStopMode(RunMode runMode, RunMode nextMode);

    public TrcDbgTrace globalTracer = TrcDbgTrace.getGlobalTracer();

    private TrcTaskMgr taskMgr = TrcTaskMgr.getInstance();
    private HalDashboard dashboard = new HalDashboard();

    private static FrcRobotBase instance = null;
    private static double modeStartTime = 0.0;
    private static long loopCounter = 0;

    private final String progName;
    private RobotMode teleOpMode = null;
    private RobotMode autoMode = null;
    private RobotMode testMode = null;
    private RobotMode disabledMode = null;
    private RunMode prevMode = RunMode.INVALID_MODE;
    private RunMode currMode = RunMode.INVALID_MODE;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param progName specifies the program name.
     */
    public FrcRobotBase(final String progName)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName, tracingEnabled, traceLevel, msgLevel);
        }

        if (FrcRobotBase.instance != null)
        {
            throw new RuntimeException("FrcRobotBase has already been instantiated.");
        }

        this.progName = progName;
        FrcRobotBase.instance = this;
        // Initialize modeStartTime just in case somebody's calling getModeElapsedTime before it's initialized.
        FrcRobotBase.modeStartTime = TrcUtil.getCurrentTime();
        dashboard.clearDisplay();
    }   //FrcRobotBase

    /**
     * This method returns the saved instance. This is a static method. So other class can get to this class instance
     * by calling getInstance(). This is very useful for other classes that need to access the public fields and
     * methods.
     *
     * @return save instance of this class.
     */
    public static FrcRobotBase getInstance()
    {
        return instance;
    }   //getInstance

    /**
     * This method returns the elapsed time since the robot mode starts. This is the elapsed time after
     * RobotMode.startMode() is called.
     *
     * @return robot mode elapsed time in seconds.
     */
    public static double getModeElapsedTime()
    {
        return TrcUtil.getCurrentTime() - modeStartTime;
    }   //getModeElapsedTime

    /**
     * This method returns the loop counter. This is very useful for code to determine if it is called multiple times
     * in the same loop. For example, it can be used to optimize sensor access so that if the sensor is accessed in
     * the same loop, there is no reason to create a new bus transaction to get "fresh" data from the sensor.
     *
     * @return loop counter value.
     */
    public static long getLoopCounter()
    {
        return loopCounter;
    }   //getLoopCounter

    /**
     * This method returns the current run mode.
     *
     * @return current run mode.
     */
    public RunMode getCurrentRunMode()
    {
        return currMode;
    }   //getCurrentRunMode

    /**
     * This method is called by the subclass to set up various robot mode objects.
     *
     * @param teleOpMode specifies the TeleOp mode object.
     * @param autoMode specifies the Autonomous mode object.
     * @param testMode specifies the Test mode object.
     * @param disabledMode specifies the Disabled mode object.
     */
    public void setupRobotModes(RobotMode teleOpMode, RobotMode autoMode, RobotMode testMode, RobotMode disabledMode)
    {
        this.teleOpMode = teleOpMode;
        this.autoMode = autoMode;
        this.testMode = testMode;
        this.disabledMode = disabledMode;
    }   //setupRobotModes

    /**
     * Start the competition match. This specific startCompetition() implements "main loop" behavior like that of
     * the FRC control system in 2008 and earlier, with a primary (slow) loop that is called periodically, and a
     * "fast loop" (a.k.a. "spin loop") that is called as fast as possible with no delay between calls. This code
     * needs to track the order of the modes starting to ensure that everything happens in the right order. Repeatedly
     * run the correct method, either Autonomous or TeleOp when the robot is enabled. After running the correct method,
     * wait for some state to change, either the other mode starts or the robot is disabled. Then go back and wait for
     * the robot to be enabled again.
     */
    public void startCompetition()
    {
        final String funcName = "startCompetition";

        globalTracer.tracePrintf(
            HalDbgLog.ESC_PREFIX + HalDbgLog.SGR_FG_BLACK +
            HalDbgLog.ESC_SEP + HalDbgLog.SGR_BG_WHITE +
            HalDbgLog.ESC_SUFFIX +
            "\n****************************************\n" +
            "Host Name: %s\n" +
            "  Program: %s\n"+
            "\n****************************************\n" +
            HalDbgLog.ESC_NORMAL,
            getHostName(), progName);

        HAL.report(tResourceType.kResourceType_Framework, tInstances.kFramework_Iterative);

        robotInit();

        //
        // Tell the DS that the robot is ready to be enabled.
        //
        HAL.observeUserProgramStarting();

        liveWindowEnabled = false;
        LiveWindow.setEnabled(liveWindowEnabled);
        //
        // loop forever, calling the appropriate mode-dependent function
        //
        final double timesliceThreshold = 0.1;
        final double taskTimeThreshold = 0.05;

        while (true)
        {
            double timeSliceStart = TrcUtil.getCurrentTime();
            double startTime, elapsedTime;

            prevMode = currMode;
            //
            // Determine the current run mode.
            //
            if (isDisabled())
            {
                currMode = RunMode.DISABLED_MODE;
            }
            else if (isTest())
            {
                currMode = RunMode.TEST_MODE;
            }
            else if (isAutonomous())
            {
                currMode = RunMode.AUTO_MODE;
            }
            else if (isOperatorControl())
            {
                currMode = RunMode.TELEOP_MODE;
            }
            else
            {
                currMode = RunMode.INVALID_MODE;
            }

            if (currMode != prevMode)
            {
                //
                // Detected mode transition.
                //
                globalTracer.traceInfo(funcName, "*** Transitioning from %s to %s ***", prevMode, currMode);
                modeStartTime = TrcUtil.getCurrentTime();

                if (prevMode != RunMode.INVALID_MODE)
                {
                    //
                    // Execute all stop tasks for previous mode.
                    //
                    if (debugEnabled)
                    {
                        startTime = TrcUtil.getCurrentTime();
                        taskMgr.executeTaskType(TrcTaskMgr.TaskType.STOP_TASK, prevMode);
                        elapsedTime = TrcUtil.getCurrentTime() - startTime;
                        dbgTrace.traceInfo(funcName, "%s.stopTask took %.3fs", prevMode, elapsedTime);
                    }
                    else
                    {
                        taskMgr.executeTaskType(TrcTaskMgr.TaskType.STOP_TASK, prevMode);
                    }
                    //
                    // Stop previous mode.
                    //
                    if (debugEnabled)
                    {
                        startTime = TrcUtil.getCurrentTime();
                    }

                    if (prevMode == RunMode.DISABLED_MODE && disabledMode != null)
                    {
                        disabledMode.stopMode(prevMode, currMode);
                    }
                    else if (prevMode == RunMode.TEST_MODE && testMode != null)
                    {
                        testMode.stopMode(prevMode, currMode);
                    }
                    else if (prevMode == RunMode.AUTO_MODE && autoMode != null)
                    {
                        autoMode.stopMode(prevMode, currMode);
                    }
                    else if (prevMode == RunMode.TELEOP_MODE && teleOpMode != null)
                    {
                        teleOpMode.stopMode(prevMode, currMode);
                    }

                    if (debugEnabled)
                    {
                        elapsedTime = TrcUtil.getCurrentTime() - startTime;
                        dbgTrace.traceInfo(funcName, "%s.stopMode took %.3fs", prevMode, elapsedTime);
                    }
                    //
                    // Run robotStopMode for the previous mode.
                    //
                    if (debugEnabled)
                    {
                        startTime = TrcUtil.getCurrentTime();
                        robotStopMode(prevMode, currMode);
                        elapsedTime = TrcUtil.getCurrentTime() - startTime;
                        dbgTrace.traceInfo(funcName, "%s.robotStopMode took %.3fs", prevMode, elapsedTime);
                    }
                    else
                    {
                        robotStopMode(prevMode, currMode);
                    }
                }

                TrcRobot.setRunMode(currMode);
                if (currMode != RunMode.INVALID_MODE)
                {
                    //
                    // Run robotStartMode for the current mode.
                    //
                    if (debugEnabled)
                    {
                        startTime = TrcUtil.getCurrentTime();
                        robotStartMode(currMode, prevMode);
                        elapsedTime = TrcUtil.getCurrentTime() - startTime;
                        dbgTrace.traceInfo(funcName, "%s.robotStartMode took %.3fs", currMode, elapsedTime);
                    }
                    else
                    {
                        robotStartMode(currMode, prevMode);
                    }
                    //
                    // Start current mode.
                    //
                    if (debugEnabled)
                    {
                        startTime = TrcUtil.getCurrentTime();
                    }

                    if (currMode == RunMode.DISABLED_MODE)
                    {
                        liveWindowEnabled = false;
                        if (disabledMode != null)
                        {
                            disabledMode.startMode(prevMode, currMode);
                        }
                    }
                    else if (currMode == RunMode.TEST_MODE)
                    {
                        liveWindowEnabled = true;
                        if (testMode != null)
                        {
                            testMode.startMode(prevMode, currMode);
                        }
                    }
                    else if (currMode == RunMode.AUTO_MODE)
                    {
                        liveWindowEnabled = false;
                        if (autoMode != null)
                        {
                            autoMode.startMode(prevMode, currMode);
                        }
                    }
                    else if (currMode == RunMode.TELEOP_MODE)
                    {
                        liveWindowEnabled = false;
                        if (teleOpMode != null)
                        {
                            teleOpMode.startMode(prevMode, currMode);
                        }
                    }
                    LiveWindow.setEnabled(liveWindowEnabled);

                    if (debugEnabled)
                    {
                        elapsedTime = TrcUtil.getCurrentTime() - startTime;
                        dbgTrace.traceInfo(funcName, "%s.startMode took %.3fs", currMode, elapsedTime);
                    }
                    //
                    // Execute all start tasks for current mode.
                    //
                    if (debugEnabled)
                    {
                        startTime = TrcUtil.getCurrentTime();
                        taskMgr.executeTaskType(TrcTaskMgr.TaskType.START_TASK, currMode);
                        elapsedTime = TrcUtil.getCurrentTime() - startTime;
                        dbgTrace.traceInfo(funcName, "%s.startTask took %.3fs", currMode, elapsedTime);
                    }
                    else
                    {
                        taskMgr.executeTaskType(TrcTaskMgr.TaskType.START_TASK, currMode);
                    }
                }
            }

            //
            // Run the time slice.
            //
            double modeElapsedTime = TrcUtil.getCurrentTime() - modeStartTime;
            boolean periodReady = nextPeriodReady();
            //
            // PreContinuous
            //
            startTime = TrcUtil.getCurrentTime();
            taskMgr.executeTaskType(TrcTaskMgr.TaskType.PRECONTINUOUS_TASK, currMode);
            elapsedTime = TrcUtil.getCurrentTime() - startTime;
            if (elapsedTime > taskTimeThreshold)
            {
                globalTracer.traceWarn(funcName, "%s.preContinuousTasks took too long (%.3fs)",
                    currMode, elapsedTime);
            }
            //
            // PrePeriodic
            //
            if (periodReady)
            {
                startTime = TrcUtil.getCurrentTime();
                taskMgr.executeTaskType(TrcTaskMgr.TaskType.PREPERIODIC_TASK, currMode);
                elapsedTime = TrcUtil.getCurrentTime() - startTime;
                if (elapsedTime > taskTimeThreshold)
                {
                    globalTracer.traceWarn(funcName, "%s.prePeriodicTasks took too long (%.3fs)",
                        currMode, elapsedTime);
                }
            }
            //
            // Continuous
            //
            startTime = TrcUtil.getCurrentTime();
            if (currMode == RunMode.DISABLED_MODE && disabledMode != null)
            {
                disabledMode.runContinuous(modeElapsedTime);
            }
            else if (currMode == RunMode.TEST_MODE && testMode != null)
            {
                testMode.runContinuous(modeElapsedTime);
            }
            else if (currMode == RunMode.AUTO_MODE && autoMode != null)
            {
                autoMode.runContinuous(modeElapsedTime);
            }
            else if (currMode == RunMode.TELEOP_MODE && teleOpMode != null)
            {
                teleOpMode.runContinuous(modeElapsedTime);
            }
            elapsedTime = TrcUtil.getCurrentTime() - startTime;
            if (elapsedTime > taskTimeThreshold)
            {
                globalTracer.traceWarn(funcName, "%s.runContinuous took too long (%.3fs)",
                    currMode, elapsedTime);
            }
            //
            // Periodic
            //
            if (periodReady)
            {
                startTime = TrcUtil.getCurrentTime();
                if (currMode == RunMode.DISABLED_MODE)
                {
                    HAL.observeUserProgramDisabled();
                    if (disabledMode != null)
                    {
                        disabledMode.runPeriodic(modeElapsedTime);
                    }
                }
                else if (currMode == RunMode.TEST_MODE)
                {
                    HAL.observeUserProgramTest();
                    if (testMode != null)
                    {
                        testMode.runPeriodic(modeElapsedTime);
                    }
                }
                else if (currMode == RunMode.AUTO_MODE)
                {
                    HAL.observeUserProgramAutonomous();
                    if (autoMode != null)
                    {
                        autoMode.runPeriodic(modeElapsedTime);
                    }
                }
                else if (currMode == RunMode.TELEOP_MODE)
                {
                    HAL.observeUserProgramTeleop();
                    if (teleOpMode != null)
                    {
                        teleOpMode.runPeriodic(modeElapsedTime);
                    }
                }
                elapsedTime = TrcUtil.getCurrentTime() - startTime;
                if (elapsedTime > taskTimeThreshold)
                {
                    globalTracer.traceWarn(funcName, "%s.runPeriodic took too long (%.3fs)",
                        currMode, elapsedTime);
                }
            }
            //
            // PostContinuous
            //
            startTime = TrcUtil.getCurrentTime();
            taskMgr.executeTaskType(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK, currMode);
            elapsedTime = TrcUtil.getCurrentTime() - startTime;
            if (elapsedTime > taskTimeThreshold)
            {
                globalTracer.traceWarn(funcName, "%s.postContinuousTasks took too long (%.3fs)",
                    currMode, elapsedTime);
            }
            //
            // PostPeriodic
            //
            if (periodReady)
            {
                startTime = TrcUtil.getCurrentTime();
                taskMgr.executeTaskType(TrcTaskMgr.TaskType.POSTPERIODIC_TASK, currMode);
                elapsedTime = TrcUtil.getCurrentTime() - startTime;
                if (elapsedTime > taskTimeThreshold)
                {
                    globalTracer.traceWarn(funcName, "%s.postPeriodicTask took too long (%.3fs)",
                        currMode, elapsedTime);
                }
            }

            startTime = TrcUtil.getCurrentTime();

            SmartDashboard.updateValues();

            if (liveWindowEnabled)
            {
                LiveWindow.updateValues();
            }

            if (dashboardEnabled && periodReady)
            {
                //
                // Only update dashboard running time at periodic rate.
                //
                dashboard.displayPrintf(0, "[%3d:%06.3f] %s",
                    (int)(modeElapsedTime/60), modeElapsedTime%60, currMode);
            }

            elapsedTime = TrcUtil.getCurrentTime() - startTime;
            if (elapsedTime > taskTimeThreshold)
            {
                globalTracer.traceWarn(funcName, "%s.updates took too long (%.3fs)",
                    currMode, elapsedTime);
            }

            //
            // Do house keeping statistics.
            //
            double timeSliceUsed = TrcUtil.getCurrentTime() - timeSliceStart;
            if (timeSliceUsed > timesliceThreshold)
            {
                globalTracer.traceWarn(funcName, "%s took too long (%.3fs)", currMode, timeSliceUsed);
            }
        }
    }   //startCompetition

    /**
     * This method returns the host name of the RobotRIO.
     *
     * @return host name.
     */
    private String getHostName()
    {
        String hostName = null;

        try
        {
            byte[] buff = new byte[256];
            Process proc = Runtime.getRuntime().exec("hostname");
            InputStream inStream = proc.getInputStream();
            inStream.read(buff, 0, buff.length);
            hostName = new String(buff);
        }
        catch(IOException e)
        {
            e.printStackTrace();
        }

        return hostName;
    }   //getHostName

    /**
     * Determine if the appropriate next periodic function should be called. Call the periodic functions whenever
     * a packet is received from the Driver Station or about every 20 msec.
     */
    private boolean nextPeriodReady()
    {
        return m_ds.isNewControlData();
    }   //nextPeriodReady

}   //class FrcRobotBase
