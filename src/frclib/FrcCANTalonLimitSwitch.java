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

import trclib.TrcDbgTrace;
import trclib.TrcDigitalInput;

/**
 * This class implements a platform dependent digital input sensor extending TrcDigitalInput. It provides
 * implementation of the abstract methods in TrcDigitalInput. The digital input sensor in this case is one
 * of the CAN Talon limit switches. This allows the CAN Talon limit switch to be used as a Digital Trigger
 * for operations such as auto zero calibration and limit switch notification callback.
 */
public class FrcCANTalonLimitSwitch extends TrcDigitalInput
{
    private FrcCANTalon canTalon;
    private boolean upperLimitSwitch = false;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param canTalon specifies the CAN Talon motor controller that hosted the limit switch.
     * @param upperLimitSwitch specifies true for the upper limit switch, false for lower limit switch.
     */
    public FrcCANTalonLimitSwitch(String instanceName, FrcCANTalon canTalon, boolean upperLimitSwitch)
    {
        super(instanceName);
        this.canTalon = canTalon;
        this.upperLimitSwitch = upperLimitSwitch;
    }   //FrcCANTalonLimitSwitch

    //
    // Implements TrcDigitalInput abstract methods.
    //

    /**
     * This method returns the state of the digital input sensor.
     *
     * @return true if the digital input sensor is active, false otherwise.
     */
    @Override
    public boolean isActive()
    {
        final String funcName = "isActive";
        boolean state = upperLimitSwitch? canTalon.isUpperLimitSwitchActive(): canTalon.isLowerLimitSwitchActive();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Boolean.toString(state));
        }

        return state;
    }   //isActive

}   //class FrcCANTalonLimitSwitch
