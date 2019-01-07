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

import edu.wpi.first.wpilibj.DigitalOutput;
import trclib.TrcDbgTrace;
import trclib.TrcDigitalOutput;

/**
 * This class implements a platform dependent digital output extending TrcDigitalOutput. It provides
 * implementation of the abstract methods in TrcDigitalOutput.
 */
public class FrcDigitalOutput extends TrcDigitalOutput
{
    private DigitalOutput digitalOutput;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param channel specifies the digital I/O channel.
     */
    public FrcDigitalOutput(String instanceName, int channel)
    {
        super(instanceName);
        digitalOutput = new DigitalOutput(channel);
    }   //FrcDigitalOutput

    //
    // Implements TrcDigitalOutput abstract methods.
    //

    /**
     * This method sets the state of the output port.
     *
     * @param state specifies state of the output port.
     */
    @Override
    public void setState(boolean state)
    {
        final String funcName = "setState";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "state=%b", state);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        digitalOutput.set(state);
    }   //setState

}   //class FrcDigitalOutput
