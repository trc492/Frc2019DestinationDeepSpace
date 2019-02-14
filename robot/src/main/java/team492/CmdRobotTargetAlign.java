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

import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTaskMgr;
import trclib.TrcPixyCam2.Vector;

import org.opencv.core.Point;

public class CmdRobotTargetAlign
{
    private static final String instanceName = "CmdRobotTargetAlign";

    public enum State
    {
        START, ALIGN_ROBOT, DONE
    }

    private Robot robot;
    private TrcEvent event;
    private TrcStateMachine<State> sm;
    private TrcTaskMgr.TaskObject lineAlignmentTask;
    private TrcEvent onFinishedEvent;
    private double targetX = 0.0;
    private double targetY = 0.0;

    private LineFollowingUtils lfu;

    public CmdRobotTargetAlign(Robot robot)
    {
        this.robot = robot;
        lfu = new LineFollowingUtils();
        sm = new TrcStateMachine<>(instanceName + ".stateMachine");
        event = new TrcEvent(instanceName + ".event");
        lineAlignmentTask = TrcTaskMgr.getInstance().createTask(instanceName + ".lineAlignTask", this::lineAlignTask);
    }

    public boolean isActive()
    {
        return sm.isEnabled();
    }

    public void start(TrcEvent event)
    {
        this.onFinishedEvent = event;
        setEnabled(true);
        sm.start(State.START);
    }

    public void cancel()
    {
        if (isActive())
        {
            if (onFinishedEvent != null)
            {
                onFinishedEvent.set(true);
            }
            stop();
        }
    }

    private void stop()
    {
        sm.stop();
        robot.pidDrive.cancel();
        onFinishedEvent = null;
        setEnabled(false);
    }

    private void setEnabled(boolean enabled)
    {
        if (enabled)
        {
            lineAlignmentTask.registerTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
            sm.start(State.START);
        }
        else
        {
            lineAlignmentTask.unregisterTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        }
    }

    private void lineAlignTask(TrcTaskMgr.TaskType type, TrcRobot.RunMode mode)
    {
        State state = sm.checkReadyAndGetState();

        if (state != null)
        {
            switch (sm.getState())
            {
                case START:
                    robot.globalTracer.traceInfo(instanceName, "%s: Starting robot alignment assist.", state);
                    sm.setState(State.ALIGN_ROBOT);
                    break;

                case ALIGN_ROBOT:
                    Vector lineVector = robot.pixy.getLineVector();

                    if (lineVector == null)
                    {
                        robot.globalTracer.traceInfo(instanceName, "%s: I don't see a line! Quitting...", state);
                        sm.setState(State.DONE);
                    }
                    else
                    {
                        robot.globalTracer.traceInfo(instanceName, "%s: Line found! line=%s", state, lineVector);

                        Point origin = lfu.getRWP(lineVector.x0, lineVector.y0);
                        Point p2 = lfu.getRWP(lineVector.x1, lineVector.y1);
                        double degrees = lfu.getTurnDegrees(lfu.getAngle(origin, p2));

                        robot.globalTracer.traceInfo(instanceName,
                            "%s: Vector origin: (%d, %d) -> %.2f, %.2f", state, lineVector.x0, lineVector.y0,
                            origin.x, origin.y);
                        robot.globalTracer.traceInfo(instanceName,
                            "%s: Vector vertex: (%d, %d) -> %.2f, %.2f", state, lineVector.x1, lineVector.y1,
                            p2.x, p2.y);
                        robot.globalTracer.traceInfo(instanceName, "%s: Target heading set to %.2f °", state, 
                            degrees);
                        robot.targetHeading = robot.driveBase.getHeading() + degrees;
                        robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                        sm.waitForSingleEvent(event, State.DONE);
                    }
                    break;

                case DONE:
                    robot.globalTracer.traceInfo(instanceName, "%s: Robot alignment finished", state);
                    if (onFinishedEvent != null)
                    {
                        onFinishedEvent.set(true);
                    }
                    sm.stop();
                    break;
            }
        }
    }
}