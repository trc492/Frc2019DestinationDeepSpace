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

package hallib;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import trclib.TrcDbgTrace;

/**
 * This class extends the SmartDashboard class and provides a way to send named data to the Driver Station to be
 * displayed, it also simulates an LCD display similar to the NXT Mindstorms. The Mindstorms has only 8 lines but
 * this dashboard can support as many lines as the Driver Station can support. By default, we set the number of lines
 * to 16. By changing a constant here, you can have as many lines as you want. This dashboard display is very useful
 * for displaying debug information.
 */
public class HalDashboard extends SmartDashboard
{
    private static final String moduleName = "HalDashboard";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    public static final int MAX_NUM_TEXTLINES = 16;
    private static final String displayKeyFormat = "%02d";

    private static HalDashboard instance = null;
    private static String[] display = new String[MAX_NUM_TEXTLINES];

    /**
     * Constructor: Creates an instance of the object.
     */
    public HalDashboard()
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName, tracingEnabled, traceLevel, msgLevel);
        }

        instance = this;
        clearDisplay();
    }   //HalDashboard

    /**
     * This static method allows any class to get an instance of the dashboard so that it can display information
     * on its display.
     *
     * @return global instance of the dashboard object.
     */
    public static HalDashboard getInstance()
    {
        return instance;
    }   //getInstance

    /**
     * This method displays a formatted message to the display on the Driver Station.
     *
     * @param lineNum specifies the line number on the display.
     * @param format specifies the format string.
     * @param args specifies variable number of substitution arguments.
     */
    public void displayPrintf(int lineNum, String format, Object... args)
    {
        if (lineNum >= 0 && lineNum < display.length)
        {
            display[lineNum] = String.format(format, args);
            SmartDashboard.putString(String.format(displayKeyFormat, lineNum), display[lineNum]);
        }
    }   //displayPrintf

    /**
     * This method clears all the display lines.
     */
    public void clearDisplay()
    {
        final String funcName = "clearDisplay";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        for (int i = 0; i < display.length; i++)
        {
            display[i] = "";
        }
        refreshDisplay();
    }   //clearDisplay

    /**
     * This method refresh the display lines to the Driver Station.
     */
    public void refreshDisplay()
    {
        final String funcName = "refreshDisplay";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        for (int i = 0; i < display.length; i++)
        {
            SmartDashboard.putString(String.format(displayKeyFormat, i), display[i]);
        }
    }   //refreshDisplay

    /**
     * This method returns the value associated with the given key. If the key does not already exist, it will
     * create the key and put the default value in it and also return the default value.
     *
     * @param key specifies the key.
     * @param defaultValue specifies the default value if the key does not already exist.
     * @return value associated with the key or the default value if key does not exist.
     */
    public static double getNumber(String key, double defaultValue)
    {
        double value = defaultValue;

        if (SmartDashboard.containsKey(key))
        {
            value = SmartDashboard.getNumber(key, defaultValue);
        }
        else
        {
            SmartDashboard.putNumber(key, defaultValue);
        }

        return value;
    }   //getNumber

    /**
     * This method returns the value associated with the given key. If the key does not already exist, it will
     * create the key and put the default value in it and also return the default value.
     *
     * @param key specifies the key.
     * @param defaultValue specifies the default value if the key does not already exist.
     * @return value associated with the key or the default value if key does not exist.
     */
    public static String getString(String key, String defaultValue)
    {
        String value = defaultValue;

        if (SmartDashboard.containsKey(key))
        {
            value = SmartDashboard.getString(key, defaultValue);
        }
        else
        {
            SmartDashboard.putString(key, defaultValue);
        }

        return value;
    }   //getString

}   //class HalDashboard
