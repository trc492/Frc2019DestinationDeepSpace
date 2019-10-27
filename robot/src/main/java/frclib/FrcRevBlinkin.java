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
    private LEDPattern currPattern = LEDPattern.SolidBlack;

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
        setPattern(currPattern);
    }   //FrcRevBlinkin

    //
    // Implements TrcRevBlinkin abstract methods.
    //

    /**
     * This method gets the current set LED pattern.
     *
     * @return currently set LED pattern.
     */
    @Override
    public LEDPattern getPattern()
    {
        final String funcName = "getPattern";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", currPattern);
        }

        return currPattern;
    }   //getPattern

    /**
     * This method sets the LED pattern to the physical REV Blinkin device.
     *
     * @param pattern specifies the color pattern.
     */
    @Override
    public void setPattern(LEDPattern pattern)
    {
        final String funcName = "setPattern";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "pattern=%s", pattern);
        }

        currPattern = pattern;
        device.set(pattern.value);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setPattern

}   //class FrcRevBlinkin
