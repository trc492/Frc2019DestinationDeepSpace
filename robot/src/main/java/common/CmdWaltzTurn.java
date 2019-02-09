/*
 * Copyright (c) 2018 Titan Robotics Club (http://www.titanrobotics.com)
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

package common;

import team492.Robot;
import trclib.TrcEvent;
import trclib.TrcPidDrive.TurnMode;
import trclib.TrcRobot;
import trclib.TrcStateMachine;

/**
 * This class implements a waltz turn command sequence. It is useful for avoiding a pushing match with our
 * opponent. If our opponent is trying to engage a pushing match with us, the driver can push a button and
 * cause the robot to pivot left or right 180 degrees and continue on with the robot driving in reverse.
 */
public class CmdWaltzTurn implements TrcRobot.RobotCommand
{
    private static enum State
    {
        WALTZ_TURN,
        DONE
    }   //enum State

    private static final String moduleName = "CmdWaltzTurn";

    private Robot robot;
    private boolean clockwiseTurn = false;
    private boolean driveInverted = false;

    private TrcEvent event;
    private TrcStateMachine<State> sm;
    private TurnMode prevTurnMode;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     */
    public CmdWaltzTurn(Robot robot)
    {
        this.robot = robot;

        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        prevTurnMode = robot.pidDrive.getTurnMode();
    }   //CmdWaltzTurn

    /**
     * This method is called to start the command sequence doing a clockwise or counter-clockwise waltz turn.
     *
     * @param clockwiseTurn specifies true for a clockwise turn, false for counter-clockwise turn.
     * @param driveInverted specifies the current inverted drive state of the robot.
     */
    public void start(boolean clockwiseTurn, boolean driveInverted)
    {
        this.clockwiseTurn = clockwiseTurn;
        this.driveInverted = driveInverted;
        cancel();
        sm.start(State.WALTZ_TURN);
    }   //start

    /**
     * This method is called to abort the waltz turn in progress.
     */
    @Override
    public void cancel()
    {
        if (robot.pidDrive.isActive())
        {
            robot.pidDrive.cancel();
            robot.pidDrive.setTurnMode(prevTurnMode);
        }
        sm.stop();
    }   //cancel

    @Override
    public boolean isActive()
    {
        return sm.isEnabled();
    }

    /**
     * This method must be called periodically by the caller to drive the command sequence forward.
     *
     * @param elapsedTime specifies the elapsed time in seconds since the start of the robot mode.
     * @return true if the command sequence is completed, false otherwise.
     */
    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        State state = sm.checkReadyAndGetState();

        if (state == null)
        {
            robot.dashboard.displayPrintf(1, "State: Disabled");
        }
        else
        {
            robot.dashboard.displayPrintf(1, "State: %s", state);

            switch (state)
            {
                case WALTZ_TURN:
                    //
                    // Do the waltz turn.
                    //
                    robot.targetHeading = robot.driveBase.getHeading();
                    robot.targetHeading += clockwiseTurn? 180.0: -180.0;

                    prevTurnMode = robot.pidDrive.getTurnMode();
                    robot.pidDrive.setTurnMode(driveInverted? TurnMode.PIVOT_FORWARD: TurnMode.PIVOT_BACKWARD);
                    robot.pidDrive.setTarget(0.0, 0.0, robot.targetHeading, false, event, 2.0);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done.
                    //
                    robot.pidDrive.setTurnMode(prevTurnMode);
                    sm.stop();
                    break;
            }
            robot.traceStateInfo(elapsedTime, state.toString(), 0.0, 0.0, robot.targetHeading);
        }

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //class CmdWaltzTurn
