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

import edu.wpi.first.wpilibj.Spark;
import trclib.TrcDbgTrace;
import trclib.TrcRevBlinkin;

/**
 * This class implements a platform dependent REV Blinkin device. It provides a platform dependent method that
 * sets the color pattern value to the device.
 */
public class FrcRevBlinkin extends TrcRevBlinkin
{
    private Spark device;
    private double currValue = 0.0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param channel specifies the PWM channel the device is on.
     */
    public FrcRevBlinkin(final String instanceName, int channel)
    {
        super(instanceName);
        device = new Spark(channel);
        set(currValue);
    }   //FrcRevBlinkin

    /**
     * This method is provided by the platform dependent subclass that extends this class. It gets the current set
     * LED pattern value.
     *
     * @return currently set LED pattern value.
     */
    @Override
    public double get()
    {
        return currValue;
    }   //get

    /**
     * This method sets the LED pattern value to the physical REV Blinkin device in a platform dependent way.
     * This method is intended to be called by the super class, not by the client of this class. The client
     * should call the setPattern method instead.
     *
     * @param value specifies the color pattern value.
     */
    @Override
    public void set(double value)
    {
        final String funcName = "set";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "value=%.2f", value);
        }

        currValue = value;
        device.set(value);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //set

}   //class FrcRevBlinkin
