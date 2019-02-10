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

    private int alignAngleTries = 0;

    private void lineAlignTask(TrcTaskMgr.TaskType type, TrcRobot.RunMode mode)
    {
        State state = sm.checkReadyAndGetState();
        State nextState;

        if (state != null)
        {
            switch (sm.getState())
            {
                case START:
                    alignAngleTries = 0;
                    robot.globalTracer.traceInfo(instanceName, "%s: Starting robot alignment assist.", state);
                    sm.setState(State.ALIGN_ROBOT);
                    break;

                case ALIGN_ROBOT:
                    robot.globalTracer.traceInfo(instanceName, "%s: Trying to align robot (try %d of 3)", state,
                        alignAngleTries);
                    Vector lineVector = robot.pixy.getLineVector();
                    // CodeReview: Why made it so complicated? Can't you put the waitForSingleEvent right after the
                    // if-else of alignAngleTries > 3 and then do a sm.setState(State.DONE) on the failure cases?
                    event.clear();
                    event.set(true); // Signal the event, this will be cleared later in case of PID drive

                    if (lineVector == null)
                    {
                        robot.globalTracer.traceInfo(instanceName, "%s: I don't see a line! Quitting...", state);
                        nextState = State.DONE;
                    }
                    else
                    {
                        robot.globalTracer.traceInfo(instanceName, "%s: Line found! line=%s", state, lineVector);

                        LineFollowingUtils.RealWorldPair origin = lfu.getRWP(lineVector.x0, lineVector.y0);
                        LineFollowingUtils.RealWorldPair p2 = lfu.getRWP(lineVector.x1, lineVector.y1);
                        double degrees = lfu.getTurnDegrees(lfu.getAngle(origin, p2));

                        robot.globalTracer.traceInfo(instanceName,
                            "%s: Vector origin: (%d, %d) -> %.2f in, %.2f in", state, lineVector.x0, lineVector.y0,
                            origin.getXLength(), origin.getYLength());
                        robot.globalTracer.traceInfo(instanceName,
                            "%s: Vector vertex: (%d, %d) -> %.2f in, %.2f in", state, lineVector.x1, lineVector.y1,
                            p2.getXLength(), p2.getYLength());
                        robot.globalTracer.traceInfo(instanceName, "%s: Target heading set to %.2f °", state, 
                            degrees);
                        robot.targetHeading = robot.driveBase.getHeading() + degrees;
                        robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);

                        // CodeReview: Why try 3 times if it is successful??
                        if (alignAngleTries >= 3)
                        {
                            nextState = State.DONE;
                        }
                        else
                        {
                            alignAngleTries++;
                            nextState = State.ALIGN_ROBOT;
                        }
                    }
                    sm.waitForSingleEvent(event, nextState);
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