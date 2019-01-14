/*
 * Copyright (c) 2017 Titan Robotics Club (http://www.titanrobotics.com)
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

import java.util.Arrays;

import edu.wpi.first.wpilibj.I2C;
import trclib.TrcDbgTrace;
import trclib.TrcI2cLEDPanel;

/**
 * This class implements a platform dependent I2C LED panel device. It extends the platform independent counterpart
 * and provides platform dependent access to the I2C device.
 */
public class FrcI2cLEDPanel extends TrcI2cLEDPanel
{
    public static final I2C.Port DEF_I2C_PORT = I2C.Port.kOnboard;
    public static final int DEF_I2C_ADDRESS = 0x8;

    private FrcI2cDevice device = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param port specifies the I2C port on the RoboRIO.
     * @param devAddress specifies the I2C address of the device.
     */
    public FrcI2cLEDPanel(final String instanceName, I2C.Port port, int devAddress)
    {
        super(instanceName);
        device = new FrcI2cDevice(instanceName, port, devAddress);
    }   //FrcI2cLEDPanel

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param port specifies the I2C port on the RoboRIO.
     */
    public FrcI2cLEDPanel(final String instanceName, I2C.Port port)
    {
        this(instanceName, port, DEF_I2C_ADDRESS);
    }   //FrcI2cLEDPanel

    /**
     * This method checks if the device is enabled.
     *
     * @return true if the device is enabled, false otherwise.
     */
    public boolean isEnabled()
    {
        final String funcName = "isEnabled";
        boolean enabled = device.isEnabled();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", enabled);
        }

        return enabled;
    }   //isEnable

    /**
     * This method enables/disables the device.
     *
     * @param enabled specifies true to enable the device, false to disable.
     */
    public void setEnabled(boolean enabled)
    {
        final String funcName = "setEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enabled=%b", enabled);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        device.setEnabled(enabled);
    }   //setEnabled

    //
    // Implements TrcI2cLEDPanel abstract methods.
    //

    /**
     * This method writes the data buffer to the device asynchronously.
     *
     * @param data specifies the data buffer.
     */
    @Override
    public void asyncWriteData(byte[] data)
    {
        final String funcName = "asyncWriteData";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "data=%s", Arrays.toString(data));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        device.asyncWrite(null, data, data.length, null, null);
    }   //asyncWriteData

}   //class FrcI2cLEDPanel
