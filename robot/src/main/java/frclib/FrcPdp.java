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

package frclib;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SensorUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import trclib.TrcDbgTrace;
import trclib.TrcRobot;
import trclib.TrcTaskMgr;
import trclib.TrcUtil;

/**
 * This class extends the WPI PowerDistricbutonPanel class to provide monitoring of energy consumption of registered
 * power channel.
 */
public class FrcPdp extends PowerDistributionPanel
{
    private static final String moduleName = "FrcPdp";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    public static final int kPDPChannels = SensorUtil.kPDPChannels;
    private TrcDbgTrace dbgTrace = null;

    private final TrcTaskMgr.TaskObject energyUsedTaskObj;
    private String[] channelNames = new String[kPDPChannels];
    private double[] channelEnergyUsed = new double[kPDPChannels];
    private double lastTimestamp = 0.0;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param canId specifies the CAN ID of the PDP.
     */
    public FrcPdp(int canId)
    {
        super(canId);

        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName, tracingEnabled, traceLevel, msgLevel);
        }

        energyUsedTaskObj = TrcTaskMgr.getInstance().createTask(moduleName + ".energyUsedTask", this::energyUsedTask);

        for (int i = 0; i < kPDPChannels; i++)
        {
            channelNames[i] = null;
            channelEnergyUsed[i] = 0.0;
        }
    }   //FrcPdp

    /**
     * This method enables/disables the energy monitoring task. When the task is enabled, it also clears the
     * totalEnergyUsed array.
     *
     * @param enabled specifies true to enable the task, false to disable.
     */
    public synchronized void setTaskEnabled(boolean enabled)
    {
        final String funcName = "setTaskEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enabled=%b", enabled);
        }

        if (enabled)
        {
            for (int i = 0; i < kPDPChannels; i++)
            {
                channelEnergyUsed[i] = 0.0;
            }

            lastTimestamp = TrcUtil.getCurrentTime();
            energyUsedTaskObj.registerTask(TrcTaskMgr.TaskType.STANDALONE_TASK);
        }
        else
        {
            energyUsedTaskObj.unregisterTask(TrcTaskMgr.TaskType.STANDALONE_TASK);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setTaskEnabled

    /**
     * This method registers a PDP channel for monitoring its energy used.
     *
     * @param channel specifies the channel to be registered.
     * @param name specifies the channel name.
     * @return true if registered successfully, false if channel is invalid or already registered.
     */
    public synchronized boolean registerEnergyUsed(int channel, String name)
    {
        final String funcName = "registerEnergyUsed";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "channel=%d,name=%s", channel, name);
        }

        boolean success = channel >= 0 && channel < kPDPChannels && channelNames[channel] == null;
        if (success)
        {
            channelNames[channel] = name;
            channelEnergyUsed[channel] = 0.0;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%b", success);
        }

        return success;
    }   //registerEnergyUsed

    /**
     * This method registers all currently unregistered PDP channels for monitoring its energy used
     * with a default name based on the channel number.
     */
    public synchronized void registerEnergyUsedForAllUnregisteredChannels()
    {
        final String funcName = "registerEnergyUsedForAllUnregisteredChannels";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        for (int i = 0; i < kPDPChannels; i++)
        {
            if (channelNames[i] == null) {
                channelNames[i] = "Unassigned_" + i;
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //registerEnergyUsed

    /**
     * This method unregisters a PDP channel for monitoring its energy used.
     *
     * @param channel specifies the channel to be unregistered.
     * @return true if unregistered successfully, false if channel is not registered.
     */
    public synchronized boolean unregisterEnergyUsed(int channel)
    {
        final String funcName = "unregisterEnergyUsed";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "channel=%d", channel);
        }

        boolean success = channelNames[channel] != null;
        if (success)
        {
            channelNames[channel] = null;
            channelEnergyUsed[channel] = 0.0;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%b", success);
        }

        return success;
    }   //unregisterEnergyUsed

    /**
     * This method returns the energy consumed so far by the specified channel in the unit of Watt-Hour.
     *
     * @param channel specifies the PDP channel.
     * @return energy consumed by the channel in Watt-Hour if registered, null if not registered.
     */
    public synchronized double getEnergyUsed(int channel)
    {
        final String funcName = "getEnergyUsed";
        double energyUsed = channelNames[channel] != null? channelEnergyUsed[channel]: 0.0;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "channel=%d", channel);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", energyUsed);
        }

        return energyUsed;
    }   //getEnergyUsed

    /**
     * This method returns the name of the registered PDP channel.
     *
     * @param channel specifies the PDP channel.
     * @return PDP channel name if registered, null if not registered.
     */
    public synchronized String getChannelName(int channel)
    {
        final String funcName = "getChannelName";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "channel=%d", channel);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", channelNames[channel]);
        }

        return channelNames[channel];
    }   //getChannelName

    /**
     * This method is called periodically to integrate the power consumption of each channel.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is running.
     */
    public synchronized void energyUsedTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = "energyUsedTask";
        double currTime = TrcUtil.getCurrentTime();
        double voltage = getVoltage();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "taskType=%s,runMode=%s", taskType, runMode);
        }

        for (int i = 0; i < kPDPChannels; i++)
        {
            if (channelNames[i] != null)
            {
                channelEnergyUsed[i] += voltage*getCurrent(i)*(currTime - lastTimestamp)/3600.0;
            }
        }

        lastTimestamp = currTime;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //energyUsedTask

    public Sendable getPdpSendable()
    {
        return new PdpInfo(moduleName);
    }

    private class PdpInfo implements Sendable
    {
        private String name;
        private String subsystem;

        public PdpInfo(String name)
        {
            this.name = name;
        }

        @Override
        public String getName()
        {
            return name;
        }

        @Override
        public void setName(String name)
        {
            this.name = name;
        }

        @Override
        public String getSubsystem()
        {
            return subsystem;
        }

        @Override
        public void setSubsystem(String subsystem)
        {
            this.subsystem = subsystem;
        }

        @Override
        public void initSendable(SendableBuilder builder)
        {
            builder.setSmartDashboardType("PowerDistributionPanel");
            for (int i = 0; i < kPDPChannels; ++i) {
                final int chan = i;
                builder.addDoubleProperty("Chan" + i, () -> FrcPdp.this.getCurrent(chan), null);
            }
            builder.addDoubleProperty("Voltage", FrcPdp.this::getVoltage, null);
            builder.addDoubleProperty("TotalCurrent", FrcPdp.this::getTotalCurrent, null);
        }
    }

}   //class FrcPdp
