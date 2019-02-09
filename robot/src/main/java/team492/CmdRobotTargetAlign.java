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
import trclib.TrcPixyCam2.Vector;

public class CmdRobotTargetAlign implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdRobotTargetAlign";

    public static enum State
    {
        START, ALIGN_ROBOT, DONE
    }

    private Robot robot;
    private TrcEvent event;
    private TrcStateMachine<State> sm;
    private double targetX = 0.0;
    private double targetY = 0.0;

    public CmdRobotTargetAlign(Robot robot)
    {
        this.robot = robot;
        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.START);
    }

    public boolean isEnabled()
    {
        return sm.isEnabled();
    }

    public boolean isActive()
    {
        return isEnabled();
    }

    public void cancel()
    {
        if (sm.isEnabled())
        {
            robot.pidDrive.cancel();
            sm.stop();
        }
    }

    private int alignAngleTries = 0;

    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        boolean done = !sm.isEnabled();

        if (!done)
        {
            State state = sm.checkReadyAndGetState();
            State nextState;
            boolean traceState = true;
            //
            // Print debug info.
            //
            robot.dashboard.displayPrintf(1, "State: %s", state == null ? "NotReady" : state);

            if (state != null)
            {
                switch (sm.getState())
                {
                    case START:
                        robot.globalTracer.traceInfo(moduleName, "%s: Starting robot alignment assist! ^w^", state);
                        sm.setState(State.ALIGN_ROBOT);
                        break;

                    case ALIGN_ROBOT:
                        robot.globalTracer.traceInfo(moduleName, "%s: Trying to align robot (try %d of 3)", state,
                            alignAngleTries);
                        Vector[] possiblePaths = robot.pixy.getLineVectors();

                        if (possiblePaths.length == 0)
                        {
                            robot.globalTracer.traceInfo(moduleName, "%s: I don't see a line! Quitting...", state);
                            nextState = State.DONE;
                        }
                        else
                        {
                            robot.globalTracer.traceInfo(moduleName, "%s: Found %d possible lines.", state, possiblePaths.length);
                            Vector toPick = null;
                            double bestLength = 0.0;
                            for (int i = 0; i < possiblePaths.length; i++)
                            {
                                Vector current = possiblePaths[i];
                                double curLength = Math.sqrt(((current.y1 - current.y0) * (current.y1 - current.y0))
                                    + ((current.x1 - current.x0) * (current.x1 - current.x0)));
                                if (curLength > bestLength)
                                {
                                    bestLength = curLength;
                                    toPick = current;
                                }
                            }

                            robot.globalTracer.traceInfo(moduleName, "%s: Line found! Index: %d, length: %.2f pixels", state, toPick.index, bestLength);

                            LineFollowingUtils.RealWorldPair origin = LineFollowingUtils.getRWP(toPick.x0, toPick.y0,
                                RobotInfo.WIDTH_COEFFICIENT, RobotInfo.HEIGHT_COEFFICIENT);
                            LineFollowingUtils.RealWorldPair p2 = LineFollowingUtils.getRWP(toPick.x1, toPick.y1,
                                RobotInfo.WIDTH_COEFFICIENT, RobotInfo.HEIGHT_COEFFICIENT);
                            double degrees = LineFollowingUtils.getTurnDegrees(LineFollowingUtils.getAngle(origin, p2));

                            robot.globalTracer.traceInfo(moduleName, "%s: Vector origin: (%d, %d) -> %.2f in, %.2f in", state, toPick.x0, toPick.y0, origin.getXLength(), origin.getYLength());
                            robot.globalTracer.traceInfo(moduleName, "%s: Vector vertex: (%d, %d) -> %.2f in, %.2f in", state, toPick.x1, toPick.y1, p2.getXLength(), p2.getYLength());
                            robot.globalTracer.traceInfo(moduleName, "%s: Target heading set to %.2f °", state, degrees);

                            robot.targetHeading = degrees;
                            robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event);
                        }

                        if (alignAngleTries >= 3)
                        {
                            nextState = State.DONE;
                        }
                        else
                        {
                            alignAngleTries++;
                            nextState = State.ALIGN_ROBOT;
                        }
                        
                        sm.waitForSingleEvent(event, nextState);
                        break;

                    case DONE:
                        robot.globalTracer.traceInfo(moduleName, "%s: Robot alignment finished :3", state);
                        done = true;
                        sm.stop();
                        break;
                }

                if (traceState)
                {
                    robot.traceStateInfo(elapsedTime, state.toString(), targetX, targetY, robot.targetHeading);
                }
            }
        }
        return done;
    }

}