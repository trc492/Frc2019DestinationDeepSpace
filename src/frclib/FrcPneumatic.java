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

import edu.wpi.first.wpilibj.Solenoid;
import trclib.TrcDbgTrace;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTaskMgr;
import trclib.TrcTimer;
import trclib.TrcUtil;

/**
 * This class implements a platform dependent pneumatic object. A pneumatic object consists of multiple pneumatic
 * channels. Pneumatic channels can be used to control a pneumatic valve or something that simply needs some voltage
 * to be turned on and off. For example, a 12V LED strip can be controlled by a pneumatic channel. An RGB LED strip
 * can be controlled by 3 pneumatic channels. Therefore, this class provides methods to turn on and off multiple
 * pneumatic channels in many different ways. It can even turn on and off the channels in a sequence of ON and OFF
 * with different time periods and in a loop so the RGB LED strip can be blink in different colors in various
 * blinking patterns.
 */
public class FrcPneumatic
{
    private static final String moduleName = "FrcPneumatic";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    /**
     * This class implements an action for setting all pneumatic channels to ON or OFF for a set period of time.
     */
    public class SolenoidAction
    {
        public byte solMask;
        public double period;
    }   //class SolenoidAction

    private enum State
    {
        START,
        DONE
    }   //enum State
    
    private String instanceName;
    private TrcTaskMgr.TaskObject pneumaticTaskObj;
    private TrcStateMachine<State> solSM;
    private TrcTimer solTimer;
    private TrcEvent timerEvent;
    private SolenoidAction[] pulseActions = new SolenoidAction[3];
    private SolenoidAction[] actionList;
    private boolean repeatActions;
    private TrcEvent notifyEvent;
    private int actionIndex;
    private int numActions;
    private Solenoid[] solenoids;
    private boolean cylinderExtended = false;

    /**
     * This method is called by all variations of the constructor to do common initialization.
     *
     * @param instanceName specifies the instance name.
     */
    private void initPneumatic(final String instanceName)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
        TrcTaskMgr taskMgr = TrcTaskMgr.getInstance();
        pneumaticTaskObj = taskMgr.createTask(instanceName + ".pneumaticTask", this::pneumaticTask);
        solSM = new TrcStateMachine<>(instanceName);
        solTimer = new TrcTimer(instanceName);
        timerEvent = new TrcEvent(instanceName);
        for (int i = 0; i < pulseActions.length; i++)
        {
            pulseActions[i] = new SolenoidAction();
        }
        actionList = null;
        repeatActions = false;
        notifyEvent = null;
        actionIndex = 0;
        numActions = 0;
    }   //initPneumatic

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param module specifies the CAN ID of the Pneumatics Control Module.
     * @param channel specifies the pneumatic channel assigned to this pneumatic object instance.
     */
    public FrcPneumatic(final String instanceName, final int module, final int channel)
    {
        solenoids = new Solenoid[1];
        solenoids[0] = new Solenoid(module, channel);
        initPneumatic(instanceName);
    }   //FrcPneumatic

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param module specifies the CAN ID of the Pneumatics Control Module.
     * @param channel1 specifies one of the pneumatic channels assigned to this pneumatic object instance.
     * @param channel2 specifies one of the pneumatic channels assigned to this pneumatic object instance.
     */
    public FrcPneumatic(final String instanceName, final int module, final int channel1, final int channel2)
    {
        solenoids = new Solenoid[2];
        solenoids[0] = new Solenoid(module, channel1);
        solenoids[1] = new Solenoid(module, channel2);
        initPneumatic(instanceName);
    }   //FrcPneumatic

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param module specifies the CAN ID of the Pneumatics Control Module.
     * @param channel1 specifies one of the pneumatic channels assigned to this pneumatic object instance.
     * @param channel2 specifies one of the pneumatic channels assigned to this pneumatic object instance.
     * @param channel3 specifies one of the pneumatic channels assigned to this pneumatic object instance.
     */
    public FrcPneumatic(
        final String instanceName, final int module, final int channel1, final int channel2, final int channel3)
    {
        solenoids = new Solenoid[3];
        solenoids[0] = new Solenoid(module, channel1);
        solenoids[1] = new Solenoid(module, channel2);
        solenoids[2] = new Solenoid(module, channel3);
        initPneumatic(instanceName);
    }   //FrcPneumatic

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param module specifies the CAN ID of the Pneumatics Control Module.
     * @param channels specifies an array of pneumatic channels assigned to this pneumatic object instance.
     */
    public FrcPneumatic(final String instanceName, final int module, int[] channels)
    {
        solenoids = new Solenoid[channels.length];
        for (int i = 0; i < solenoids.length; i++)
        {
            solenoids[i] = new Solenoid(module, channels[i]);
        }
        initPneumatic(instanceName);
    }   //FrcPneumatic

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method sets the state of the pneumatic channels specified in a bit mask.
     *
     * @param bitMask specifies the bit mask of the pneumatic channels to be set.
     * @param on specifies true to set the channels ON and false to set them OFF.
     */
    public void set(byte bitMask, boolean on)
    {
        final String funcName = "set";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "bitMask=%x,on=%s",
                bitMask, Boolean.toString(on));
        }

        cancel();
        for (int i = 0; i < solenoids.length; i++)
        {
            if (((1 << i) & bitMask) != 0)
            {
                solenoids[i].set(on);
            }
        }

        if (solenoids.length <= 2)
        {
            if ((bitMask & 0x1) != 0 && on)
            {
                // Extend channel fired.
                cylinderExtended = true;
            }
            else if ((bitMask & 0x2) != 0 && on)
            {
                // Retract channel fired.
                cylinderExtended = false;
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //set

    /**
     * This method sets the specified pneumatic channels to ON or OFF.
     *
     * @param onMask specifies the bit mask of the channels to be set to ON.
     * @param offMask specifies the bit mask of the channels to be set to OFF.
     */
    public void set(byte onMask, byte offMask)
    {
        set(offMask, false);
        set(onMask, true);
    }   //set

    /**
     * This method sets the specified pneumatic channels to ON or OFF for a specified period of time and optionally
     * signals an event when the period has expired.
     *
     * @param solMask specifies the bit mask of all channels. 1's specify ON channels and 0's specify OFF channels.
     * @param onPeriod specifies the period of time the ON channels remained ON and turn them off afterwards.
     * @param event specifies the event to be signaled when the ON period expires, null if none specified.
     */
    public void set(byte solMask, double onPeriod, TrcEvent event)
    {
        pulseActions[0].solMask = solMask;
        pulseActions[0].period = onPeriod;
        pulseActions[1].solMask = 0;
        pulseActions[1].period = 0.0;
        set(pulseActions, 2, false, event);
    }   //set

    /**
     * This method sets all pneumatic channels to the states specified by solMask1 for period1 then sets them to the
     * states specified by solMask2 for period2. If repeat is true, the period1 and period2 patterns repeat.
     * Otherwise, the specified event is signaled. If repeat is true, the event parameter is ignored.
     *
     * @param solMask1 specifies first bit mask of all channels. 1's specify ON channels and 0's specify OFF channels.
     * @param period1 specifies the period of time the mask 1 channels will remained in their states.
     * @param solMask2 specifies second bit mask of all channels. 1's specify ON channels and 0's specify OFF channels.
     * @param period2 specifies the period of time the mask 2 channels will remained in their states.
     * @param repeat specifies true if this mask1/mask2 pattern will be repeated, false otherwise.
     * @param event specifies the event to be signaled when period2 has expired, null if none specified.
     *        This parameter is ignored if repeat is true.
     */
    public void set(
            byte solMask1,
            double period1,
            byte solMask2,
            double period2,
            boolean repeat,
            TrcEvent event)
    {
        pulseActions[0].solMask = solMask1;
        pulseActions[0].period = period1;
        pulseActions[1].solMask = solMask2;
        pulseActions[1].period = period2;
        pulseActions[2].solMask = 0;
        pulseActions[2].period = 0.0;
        set(pulseActions, 3, repeat, event);
    }   //set

    /**
     * This method sets the ON/OFF pattern sequence of all pneumatic channels. If repeat is true, the sequence will
     * be repeated. Otherwise, the specified event is signaled. If repeat is true, the event parameter is ignored.
     *
     * @param actionList specifies the sequence of ON/OFF actions of all the pneumatic channels and their periods.
     * @param numActions specifies the number of valid actions in the array. The array length could be longer than
     *        numActions.
     * @param repeat specifies true if the sequence will be repeated, false otherwise.
     * @param event specifies the event to be signaled when the end of sequence has been reached, null if none
     *        specified. This parameter is ignored if repeat is true.
     */
    public void set(
            SolenoidAction[] actionList,
            int numActions,
            boolean repeat,
            TrcEvent event)
    {
        final String funcName = "set";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "numActions=%d,repeat=%s,event=%s",
                    numActions, Boolean.toString(repeat),
                    event != null? event.toString(): "null");
        }

        if (actionList.length > 0 && actionList[0].period > 0.0)
        {
            cancel();
            this.actionList = actionList;
            this.numActions = numActions;
            if (numActions > actionList.length)
            {
                this.numActions = actionList.length;
            }
            this.repeatActions = repeat;
            if (event != null)
            {
                event.clear();
            }
            this.notifyEvent = event;
            actionIndex = 0;
            setTaskEnabled(true);
            solSM.start(State.START);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //set

    /**
     * This method sets the state of a 1 or 2-channel pneumatic cylinder.
     *
     * @param state specifies true to extend the cylinder, false to retract.
     */
    public void setState(boolean state)
    {
        final String funcName = "setState";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "state=%s", state);
        }

        if (state)
        {
            extend();
        }
        else
        {
            retract();
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setState

    /**
     * This method extends a 1 or 2-channel pneumatic cylinder.
     */
    public void extend()
    {
        final String funcName = "extend";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (solenoids.length == 2)
        {
            //
            // Two-valve cylinder: first channel is the extend valve and
            // second channel is the retract valve.
            //
            set((byte)(1 << 0), (byte)(1 << 1));
        }
        else if (solenoids.length == 1)
        {
            //
            // One-valve spring loaded cylinder: only one extend channel.
            //
            set((byte)1, true);
        }
        else
        {
            throw new UnsupportedOperationException("Method supports only one or two-valve cylinders.");
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //extend

    /**
     * This method extends a 1 or 2-channel pneumatic cylinder for the specified period and deactivates it. If an
     * event is specified, it is signaled at the end of the period.
     *
     * @param period specifies the period to extend the pneumatic cylinder.
     * @param event specifies the event to signal at the end of the period, null if none specified.
     */
    public void extend(double period, TrcEvent event)
    {
        final String funcName = "extend";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "period=%f,event=%s",
                    period, event != null? event.toString(): "null");
        }

        if (solenoids.length == 1 || solenoids.length == 2)
        {
            set((byte)(1 << 0), period, event);
        }
        else
        {
            throw new UnsupportedOperationException(
                    "Method supports only one or two-valve cylinders.");
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //extend

    /**
     * This method retracts a 1 or 2-channel pneumatic cylinder.
     */
    public void retract()
    {
        final String funcName = "retract";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (solenoids.length == 2)
        {
            //
            // Two-valve cylinder: first channel is the extend valve and
            // second channel is the retract valve.
            //
            set((byte)(1 << 1), (byte)(1 << 0));
        }
        else if (solenoids.length == 1)
        {
            //
            // One-valve spring loaded cylinder: only one extend channel.
            //
            set((byte)1, false);
        }
        else
        {
            throw new UnsupportedOperationException(
                    "Method supports only one or two-valve cylinders.");
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //retract

    /**
     * This method retracts a 1 or 2-channel pneumatic cylinder for the specified period and deactivates it. If an
     * event is specified, it is signaled at the end of the period.
     *
     * @param period specifies the period to retract the pneumatic cylinder.
     * @param event specifies the event to signal at the end of the period, null if none specified.
     */
    public void retract(double period, TrcEvent event)
    {
        final String funcName = "retract";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "period=%f,event=%s",
                    period, event != null? event.toString(): "null");
        }

        if (solenoids.length == 2)
        {
            //
            // Two-valve cylinder: first channel is the extend valve and
            // second channel is the retract valve.
            //
            set((byte)(1 << 1), period, event);
        }
        else
        {
            throw new UnsupportedOperationException(
                    "Method supports only two-valve cylinders.");
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //extend

    /**
     * This method extends the pneumatic cylinder for extendPeriod and then retracts it for retractPeriod and then
     * deactivates it at the end.
     *
     * @param extendPeriod specifies the period to extend the pneumatic cylinder.
     * @param retractPeriod specifies the period to retract the pneumatic cylinder after it's been extended.
     * @param event specifies the event to signal at the end of the retract period, null if none specified.
     */
    public void timedExtend(double extendPeriod, double retractPeriod, TrcEvent event)
    {
        final String funcName = "timedExtend";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "extendPeriod=%.3f,retractPeriod=%.3f,event=%s",
                extendPeriod, retractPeriod, event);
        }

        if (solenoids.length == 2)
        {
            //
            // Two-valve cylinder: first channel is the extend valve and
            // second channel is the retract valve.
            //
            set((byte)(1 << 0), extendPeriod, (byte)(1 << 1), retractPeriod, false, event);
        }
        else
        {
            throw new UnsupportedOperationException("Method supports only two-valve cylinders.");
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //timedExtend

    /**
     * This method extends the pneumatic cylinder for extendPeriod and then retracts it.
     *
     * @param extendPeriod specifies the period to extend the pneumatic cylinder.
     */
    public void timedExtend(double extendPeriod)
    {
        timedExtend(extendPeriod, 0.0, null);
    }   //timedExtend

    /**
     * This method returns the state of the one or two-valve pneumatic cylinder.
     *
     * @return true if the cylinder is extended, false if it is retracted.
     */
    public boolean isExtended()
    {
        final String funcName = "isExtended";
        boolean state = false;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (solenoids.length <= 2)
        {
            state = cylinderExtended;
        }
        else
        {
            throw new UnsupportedOperationException("Method supports only 1 or 2-valve cylinders.");
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", state);
        }

        return state;
    }   //isExtended

    /**
     * This method enables/disabled the pneumatic task that executes the action sequence.
     *
     * @param enabled specifies true to enable and false to disable.
     */
    private void setTaskEnabled(boolean enabled)
    {
        final String funcName = "setTaskEnabled";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC, "enabled=%b", enabled);
        }

        if (enabled)
        {
            pneumaticTaskObj.registerTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        }
        else
        {
            pneumaticTaskObj.unregisterTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC);
        }
    }   //setTaskEnabled

    /**
     * This method disables the pneumatic task and cancels all pending actions.
     */
    private void cancel()
    {
        final String funcName = "cancel";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC);
        }

        if (solSM.isEnabled())
        {
            setTaskEnabled(false);
            solTimer.cancel();
            solSM.stop();
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC);
        }
    }   //cancel

    /**
     * This method is call periodically to execute the action sequence.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is running.
     */
    public void pneumaticTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = "pneumaticTask";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "taskType=%s,runMode=%s", taskType, runMode);
        }

        if (solSM.isReady())
        {
            State state = solSM.getState();
            if (debugEnabled)
            {
                dbgTrace.traceInfo(funcName, "Executing state %d.", state);
            }

            switch (state)
            {
                case START:
                    if (actionIndex < numActions)
                    {
                        //
                        // Turn the each of the solenoids ON/OFF.
                        //
                        if (debugEnabled)
                        {
                            dbgTrace.traceInfo(
                                    funcName,
                                    "[%f] Executing action %d/%d",
                                    TrcUtil.getCurrentTime(),
                                    actionIndex, numActions);
                        }

                        for (int i = 0; i < solenoids.length; i++)
                        {
                            if (((1 << i) & actionList[actionIndex].solMask) != 0)
                            {
                                solenoids[i].set(true);
                                if (debugEnabled)
                                {
                                    dbgTrace.traceInfo(
                                            funcName, "Set solenoid %d ON.", i);
                                }
                            }
                            else
                            {
                                solenoids[i].
                                set(false);
                                if (debugEnabled)
                                {
                                    dbgTrace.traceInfo(
                                            funcName, "Set solenoid %d OFF.", i);
                                }
                            }
                        }

                        if (solenoids.length <= 2)
                        {
                            if ((actionList[actionIndex].solMask & 0x1) != 0)
                            {
                                // Extend channel fired.
                                cylinderExtended = true;
                            }
                            else if ((actionList[actionIndex].solMask & 0x2) != 0)
                            {
                                // Retract channel fired.
                                cylinderExtended = false;
                            }
                        }

                        //
                        // Set timer and wait for it if necessary.
                        //
                        if (actionList[actionIndex].period > 0.0)
                        {
                            if (debugEnabled)
                            {
                                dbgTrace.traceInfo(
                                        funcName,
                                        "Set timer for %f",
                                        actionList[actionIndex].period);
                            }
                            solTimer.set(
                                    actionList[actionIndex].period,
                                    timerEvent);
                            solSM.addEvent(timerEvent);
                            solSM.waitForEvents(state);
                        }
                        //
                        // Move to the next action.
                        //
                        actionIndex++;
                        if (repeatActions && actionIndex >= numActions)
                        {
                            actionIndex = 0;
                        }
                    }
                    else
                    {
                        //
                        // No more action, we are done.
                        //
                        solSM.setState(State.DONE);;
                    }
                    break;

                case DONE:
                default:
                    //
                    // We are done.
                    //
                    if (debugEnabled)
                    {
                        dbgTrace.traceInfo(funcName, "Done!");
                    }
                    setTaskEnabled(false);
                    if (notifyEvent != null)
                    {
                        notifyEvent.set(true);
                        notifyEvent = null;
                    }
                    actionList = null;
                    repeatActions = false;
                    actionIndex = 0;
                    numActions = 0;
                    solSM.stop();
                    break;
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //pneumaticTask

}   //class FrcPneumatic
