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

import edu.wpi.first.wpilibj.SerialPort;
import trclib.TrcDbgTrace;
import trclib.TrcEmic2TextToSpeech;
import trclib.TrcSerialBusDevice;

/**
 * This class implements a platform dependent Emic2 text to speech device that is connected to a Serial Port.
 * It extends the TrcEmicTextToSpeech class to provide platform dependent asynchronous access to the serial port.
 */
public class FrcEmic2TextToSpeech extends TrcEmic2TextToSpeech
{
    public static final SerialPort.Port DEF_SERIAL_PORT = SerialPort.Port.kMXP;
    public static final int DEF_BAUD_RATE = 9600;
    public static final int DEF_DATA_BITS = 8;
    public static final SerialPort.Parity DEF_PARITY = SerialPort.Parity.kNone;
    public static final SerialPort.StopBits DEF_STOP_BITS = SerialPort.StopBits.kOne;

    private TrcSerialBusDevice tts = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param port specifies the serial port on the RoboRIO.
     * @param baudRate specifies the baud rate.
     * @param dataBits specifies the number of data bits.
     * @param parity specifies the parity type.
     * @param stopBits specifies the number of stop bits.
     */
    public FrcEmic2TextToSpeech(
        final String instanceName, SerialPort.Port port, int baudRate, int dataBits, SerialPort.Parity parity,
        SerialPort.StopBits stopBits)
    {
        super(instanceName);

        tts = new FrcSerialPortDevice(instanceName, port, baudRate, dataBits, parity, stopBits, true);
    }   //FrcEmic2TextToSpeech

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param port specifies the serial port on the RoboRIO.
     * @param baudRate specifies the baud rate.
     */
    public FrcEmic2TextToSpeech(final String instanceName, SerialPort.Port port, int baudRate)
    {
        this(instanceName, port, baudRate, DEF_DATA_BITS, DEF_PARITY, DEF_STOP_BITS);
    }   //FrcEmic2TextToSpeech

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param port specifies the serial port on the RoboRIO.
     */
    public FrcEmic2TextToSpeech(final String instanceName, SerialPort.Port port)
    {
        this(instanceName, port, DEF_BAUD_RATE, DEF_DATA_BITS, DEF_PARITY, DEF_STOP_BITS);
    }   //FrcEmic2TextToSpeech

    /**
     * This method checks if the device is enabled.
     *
     * @return true if device is enabled, false otherwise.
     */
    public boolean isEnabled()
    {
        final String funcName = "isEnabled";
        boolean enabled = tts.isEnabled();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Boolean.toString(enabled));
        }

        return enabled;
    }   //isEnable

    /**
     * This method enables/disables the device to start/stop the communication
     *
     * @param enabled specifies true to enable device, false to disable.
     */
    public void setEnabled(boolean enabled)
    {
        final String funcName = "setEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enanbled=%s", Boolean.toString(enabled));
        }

        tts.setEnabled(enabled);
        if (enabled)
        {
            start();
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setEnabled

    //
    // Implements TrcEmic2TextToSpeech abstract methods.
    //

    /**
     * This method issues an asynchronous read of a string from the device.
     *
     * @param requestId specifies the ID to identify the request. Can be null if none was provided.
     */
    @Override
    public void asyncReadString(RequestId requestId)
    {
        final String funcName = "asyncReadString";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        tts.asyncRead(requestId, 0, null, this::notify);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //asyncReadString

    /**
     * This method writes the string to the device asynchronously.
     *
     * @param text specifies the text string to be written to the device.
     * @param preemptive specifies true for immediate write without queuing, false otherwise.
     */
    @Override
    public void asyncWriteString(String text, boolean preemptive)
    {
        final String funcName = "asyncWriteString";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "text=%s,length=%d,preemptive=%s",
                text, text.length(), Boolean.toString(preemptive));
        }

        byte[] data = text.getBytes();
        if (preemptive)
        {
            tts.preemptiveWrite(-1, data, data.length);
        }
        else
        {
            tts.asyncWrite(null, data, data.length, null, null);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //asyncWriteString

}   //class FrcEmic2TextToSpeech
