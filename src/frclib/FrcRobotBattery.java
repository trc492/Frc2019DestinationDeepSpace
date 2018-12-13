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

import trclib.TrcDbgTrace;
import trclib.TrcRobotBattery;

/**
 * This class extends the TrcRobotBattery which provides a task to monitor the robot battery levels and the methods to
 * access the highest and the lowest battery levels during the monitoring session.
 */
public class FrcRobotBattery extends TrcRobotBattery
{
    private FrcPdp pdp;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param pdp specifies the PDP object.
     */
    public FrcRobotBattery(FrcPdp pdp)
    {
        super(true, true, true);
        this.pdp = pdp;
    }   //FrcRobotBattery

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param canId specifies the CAN ID of the PDP.
     */
    public FrcRobotBattery(int canId)
    {
        this(new FrcPdp(canId));
    }   //FrcRobotBattery

    //
    // Implements TrcRobotBattery abstract methods.
    //

    /**
     * This method returns the robot battery voltage.
     *
     * @return robot battery voltage in volts.
     */
    @Override
    public double getVoltage()
    {
        final String funcName = "getVoltage";
        double voltage = pdp.getVoltage();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", voltage);
        }

        return voltage;
    }   //getVoltage

    /**
     * This method returns the robot battery current.
     *
     * @return robot battery current in amps.
     */
    @Override
    public double getCurrent()
    {
        final String funcName = "getCurrent";
        double totalCurrent = pdp.getTotalCurrent();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", totalCurrent);
        }

        return totalCurrent;
    }   //getCurrent

    /**
     * This method returns the robot battery power.
     *
     * @return robot battery current in watts.
     */
    @Override
    public double getPower()
    {
        final String funcName = "getPower";
        double totalPower = pdp.getTotalPower();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", totalPower);
        }

        return totalPower;
    }   //getPower

}   //class FrcRobotBattery
