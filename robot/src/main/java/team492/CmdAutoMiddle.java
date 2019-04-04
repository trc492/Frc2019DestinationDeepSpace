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

package team492;

import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;

/**
 * This auto command is just so that if vision fails at the start the match, the drivers can just slap a button to
 * automatically do SOMETHING, at the very least. This is a low priority thing to test/tune, I just had some time on
 * my hands, so I wrote out one of them. Do not test this unless you have finished the higher priority things.
 */

public class CmdAutoMiddle implements TrcRobot.RobotCommand
{
    private static final String instanceName = "CmdAutoMiddle";

    private enum State
    {
        START, DRIVE_OFF_PLATFORM, DRIVE_TO_TARGET, DEPLOY, DONE
    }

    private TrcStateMachine<State> sm;
    private TrcEvent event;
    private Robot robot;
    private boolean rightSide;

    public CmdAutoMiddle(Robot robot)
    {
        this.robot = robot;

        sm = new TrcStateMachine<>(instanceName + ".stateMachine");
        event = new TrcEvent(instanceName + ".event");
    }

    public void start(boolean rightSide)
    {
        this.rightSide = rightSide;
        sm.start(State.START);
    }

    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        boolean done = !sm.isEnabled();
        if (done)
        {
            return true;
        }

        State state = sm.checkReadyAndGetState();

        if (state != null)
        {
            switch (state)
            {
                case START:
                case DRIVE_OFF_PLATFORM:
                    robot.targetHeading = 0.0;
                    robot.pidDrive.setTarget(0.0, RobotInfo.HAB_1_DRIVE_OFF_DIST, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DRIVE_TO_TARGET);
                    break;

                case DRIVE_TO_TARGET:
                    double xTarget = rightSide ? 10 : -10;
                    double yTarget = RobotInfo.DRIVE_TO_CARGO_SHIP_FRONT_DIST;
                    robot.pidDrive.setTarget(xTarget, yTarget, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DEPLOY);
                    break;

                case DEPLOY:
                    robot.autoAlign.start(RobotInfo.DeployLevel.LOW, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case DONE:
                    done = true;
                    cancel();
                    break;
            }
        }
        return done;
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
        robot.stopSubsystems();
    }
}
