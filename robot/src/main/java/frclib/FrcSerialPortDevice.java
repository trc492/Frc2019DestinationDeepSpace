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

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.*;
import trclib.TrcDbgTrace;
import trclib.TrcSerialBusDevice;

/**
 * This class implements a platform dependent serial port device (UART). It extends TrcSerialBusDevice to provide
 * asynchronous request queue support and creates the SerialPort device using WPILib.
 */
public class FrcSerialPortDevice extends TrcSerialBusDevice
{
    private SerialPort device;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param port specifies the serial port (on-board or on the MXP).
     * @param baudRate specifies the serial baud rate.
     * @param dataBits specifies the number of data bits.
     * @param parity specifies the parity type.
     * @param stopBits specifies the number of stop bits.
     * @param useRequestQueue specifies true to use a request queue, false otherwise.
     */
    public FrcSerialPortDevice(
        final String instanceName, Port port, int baudRate, int dataBits, Parity parity, StopBits stopBits,
        boolean useRequestQueue)
    {
        super(instanceName, useRequestQueue);
        device = new SerialPort(baudRate, port, dataBits, parity, stopBits);
    }   //FrcSerialPortDevice

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param baudRate specifies the serial baud rate.
     * @param dataBits specifies the number of data bits.
     * @param parity specifies the parity type.
     * @param stopBits specifies the number of stop bits.
     * @param useRequestQueue specifies true to use a request queue, false otherwise.
     */
    public FrcSerialPortDevice(
        final String instanceName, int baudRate, int dataBits, Parity parity, StopBits stopBits,
        boolean useRequestQueue)
    {
        this(instanceName, Port.kOnboard, baudRate, dataBits, parity, stopBits, useRequestQueue);
    }   //FrcSerialPort

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param port specifies the serial port (on-board or on the MXP).
     * @param baudRate specifies the serial baud rate.
     * @param useRequestQueue specifies true to use a request queue, false otherwise.
     */
    public FrcSerialPortDevice(final String instanceName, Port port, int baudRate, boolean useRequestQueue)
    {
        this(instanceName, port, baudRate, 8, Parity.kNone, StopBits.kOne, useRequestQueue);
    }   //FrcSerialPort

    //
    // Implements TrcSerialBusDevice abstract methods.
    //

    /**
     * This method is called to read data from the device with the specified length.
     *
     * @param address specifies the data address if any (not applicable for SerialPort).
     * @param length specifies the number of bytes to read. If zero, read all that has been received.
     * @return a byte array containing the data read.
     */
    @Override
    public byte[] readData(int address, int length)
    {
        final String funcName = "readData";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.CALLBK, "addr=%d,len=%d", address, length);
        }

        if (length == 0)
        {
            length = device.getBytesReceived();
        }
        byte[] data = device.read(length);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.CALLBK, "=%s", Arrays.toString(data));
        }

        return data;
    }   //readData

    /**
     * This method is called to write data to the device with the specified data buffer and length.
     *
     * @param address specifies the data address if any (not applicable for SerialPort).
     * @param buffer specifies the buffer containing the data to be written to the device.
     * @param length specifies the number of bytes to write.
     * @return number of bytes written.
     */
    @Override
    public int writeData(int address, byte[] buffer, int length)
    {
        final String funcName = "writeData";
        int bytesWritten = device.write(buffer, length);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.CALLBK, "addr=%d,data=%s,len=%d",
                address, Arrays.toString(buffer), length);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.CALLBK, "=%d", bytesWritten);
        }

        return bytesWritten;
    }   //writeData

}   //class FtcSerialPortDevice
