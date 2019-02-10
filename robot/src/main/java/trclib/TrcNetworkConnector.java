/*
 * Copyright (c) 2019 Titan Robotics Club (http://www.titanrobotics.com)
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

package trclib;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.net.Socket;

import jsonrpc.client.Transport;

public class TrcNetworkConnector implements Transport
{
    private static final String moduleName = "FrcAdbBridge";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private int port;
    private Socket socket = null;
    private BufferedWriter bufferedWriter = null;
    private BufferedReader bufferedReader = null;
    
    public TrcNetworkConnector(int port)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName, tracingEnabled, traceLevel, msgLevel);
        }

        this.port = port;
    }   //TrcNetworkConnector
    
    public boolean openConnector()
    {
        final String funcName = "openConnector";
        boolean success = true;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (socket == null)
        {
            try
            {
                socket = new Socket("127.0.0.1", port);
                bufferedWriter = new BufferedWriter(new OutputStreamWriter(socket.getOutputStream()));
                bufferedReader = new BufferedReader(new InputStreamReader(socket.getInputStream()));
            }
            catch (Exception e)
            {
                TrcDbgTrace.getGlobalTracer().traceErr(funcName, "Failed to open connector: %s", e.getMessage());
                closeConnector();
                success = false;
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", success);
        }

        return success;
    }   //openConnector
    
    public boolean closeConnector()
    {
        final String funcName = "closeConnector";
        boolean success = true;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        try
        {
            if (bufferedReader != null)
            {
                bufferedReader.close();
                bufferedReader = null;
            }

            if (bufferedWriter != null)
            {
                bufferedWriter.close();
                bufferedWriter = null;
            }

            if (socket != null)
            {
                socket.close();
                socket = null;
            }
        }
        catch (IOException e)
        {
            TrcDbgTrace.getGlobalTracer().traceErr(funcName, "Failed to close connector: %s", e.getMessage());
            success = false;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", success);
        }

        return success;
    }   //closeConnector

    public boolean writeLine(String msg)
    {
        final String funcName = "writeLine";
        boolean success = true;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "msg=%s", msg);
        }

        if (socket != null)
        {
            if (!socket.isClosed() && socket.isConnected())
            {
                try
                {
                    bufferedWriter.write(msg + "\n");
                    bufferedWriter.flush();
                }
                catch (IOException e)
                {
                    TrcDbgTrace.getGlobalTracer().traceErr(funcName, "Failed to write to connector: %s", e.getMessage());
                    success = false;
                }
            }
        }
        else
        {
            success = false;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", success);
        }

        return success;
    }   //writeLine

    public String readLine()
    {
        final String funcName = "readLine";
        String line = null;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (socket != null)
        {
            if (!socket.isClosed() && socket.isConnected())
            {
                try
                {
                    line = bufferedReader.readLine();
                    if (line != null)
                    {
                        line += "\n";
                    }
                }
                catch (IOException e)
                {
                    TrcDbgTrace.getGlobalTracer().traceErr(funcName, "Failed to read from connector: %s",
                        e.getMessage());
                }
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", line);
        }

        return line;
    }   //readLine

    @Override
    public String pass(String request) throws IOException
    {
        final String funcName = "pass";
        String reply = null;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "request=%s", request);
        }

        if (writeLine(request))
        {
            reply = readLine();
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", reply);
        }

        return reply;
    }   //pass

}   //class TrcNetworkConnector
