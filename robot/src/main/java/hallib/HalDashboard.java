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
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
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

import java.nio.ByteBuffer;
import java.util.Set;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.NamedSendable;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import trclib.TrcDbgTrace;

/**
 * This class extends the SmartDashboard class and provides a way to send named
 * data to the Driver Station to be displayed, it also simulates an LCD display
 * similar to the NXT Mindstorms. The Mindstorms has only 8 lines but this
 * dashboard can support as many lines as the Driver Station can support. By
 * default, we set the number of lines to 16. By changing a constant here, you
 * can have as many lines as you want. This dashboard display is very useful for
 * displaying debug information.
 */
public class HalDashboard
{
    private static final String moduleName = "HalDashboard";
    private static final boolean tracingEnabled = false;
    private static final boolean debugEnabled = false;
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
            dbgTrace = useGlobalTracer ? TrcDbgTrace.getGlobalTracer()
                : new TrcDbgTrace(moduleName, tracingEnabled, traceLevel, msgLevel);
        }

        instance = this;
        clearDisplay();
    } // HalDashboard

    /**
     * This static method allows any class to get an instance of the dashboard
     * so that it can display information on its display.
     *
     * @return global instance of the dashboard object.
     */
    public static HalDashboard getInstance()
    {
        return instance;
    } // getInstance

    /**
     * This method displays a formatted message to the display on the Driver
     * Station.
     *
     * @param lineNum
     *                    specifies the line number on the display.
     * @param format
     *                    specifies the format string.
     * @param args
     *                    specifies variable number of substitution arguments.
     */
    public void displayPrintf(int lineNum, String format, Object... args)
    {
        if (lineNum >= 0 && lineNum < display.length)
        {
            display[lineNum] = String.format(format, args);
            SmartDashboard.putString(String.format(displayKeyFormat, lineNum), display[lineNum]);
        }
    } // displayPrintf

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
    } // clearDisplay

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
    } // refreshDisplay

    /**
     * This method returns the value associated with the given key. If the key
     * does not already exist, it will create the key and put the default value
     * in it and also return the default value.
     *
     * @param key
     *                         specifies the key.
     * @param defaultValue
     *                         specifies the default value if the key does not
     *                         already exist.
     * @return value associated with the key or the default value if key does
     *         not exist.
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
    } // getNumber

    /**
     * This method returns the value associated with the given key. If the key
     * does not already exist, it will create the key and put the default value
     * in it and also return the default value.
     *
     * @param key
     *                         specifies the key.
     * @param defaultValue
     *                         specifies the default value if the key does not
     *                         already exist.
     * @return value associated with the key or the default value if key does
     *         not exist.
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
    } // getString

    /**
     * Maps the specified key to the specified value in this table. The key can
     * not be null. The value can be retrieved by calling the get method with a
     * key that is equal to the original key.
     *
     * @param key
     *                 the key
     * @param data
     *                 the value
     * @throws IllegalArgumentException
     *                                      If key is null
     */
    public static synchronized void putData(String key, Sendable data)
    {
        SmartDashboard.putData(key, data);
    }

    /**
     * Maps the specified key (where the key is the name of the
     * {@link NamedSendable} to the specified value in this table. The value can
     * be retrieved by calling the get method with a key that is equal to the
     * original key.
     *
     * @param value
     *                  the value
     * @throws IllegalArgumentException
     *                                      If key is null
     */
    public static void putData(Sendable value)
    {
        putData(value.getName(), value);
    }

    /**
     * Returns the value at the specified key.
     *
     * @param key
     *                the key
     * @return the value
     * @throws IllegalArgumentException
     *                                      if the key is null
     */
    public static synchronized Sendable getData(String key)
    {
        return SmartDashboard.getData(key);
    }

    /**
     * Gets the entry for the specified key.
     * 
     * @param key
     *                the key name
     * @return Network table entry.
     */
    public static NetworkTableEntry getEntry(String key)
    {
        return SmartDashboard.getEntry(key);
    }

    /**
     * Checks the table and tells if it contains the specified key.
     *
     * @param key
     *                the key to search for
     * @return true if the table as a value assigned to the given key
     */
    public static boolean containsKey(String key)
    {
        return SmartDashboard.containsKey(key);
    }

    /**
     * Get the keys stored in the SmartDashboard table of NetworkTables.
     *
     * @param types
     *                  bitmask of types; 0 is treated as a "don't care".
     * @return keys currently in the table
     */
    public static Set<String> getKeys(int types)
    {
        return SmartDashboard.getKeys(types);
    }

    /**
     * Get the keys stored in the SmartDashboard table of NetworkTables.
     *
     * @return keys currently in the table.
     */
    public static Set<String> getKeys()
    {
        return SmartDashboard.getKeys();
    }

    /**
     * Makes a key's value persistent through program restarts. The key cannot
     * be null.
     *
     * @param key
     *                the key name
     */
    public static void setPersistent(String key)
    {
        getEntry(key).setPersistent();
    }

    /**
     * Stop making a key's value persistent through program restarts. The key
     * cannot be null.
     *
     * @param key
     *                the key name
     */
    public static void clearPersistent(String key)
    {
        getEntry(key).clearPersistent();
    }

    /**
     * Returns whether the value is persistent through program restarts. The key
     * cannot be null.
     *
     * @param key
     *                the key name
     * @return True if the value is persistent.
     */
    public static boolean isPersistent(String key)
    {
        return getEntry(key).isPersistent();
    }

    /**
     * Sets flags on the specified key in this table. The key can not be null.
     *
     * @param key
     *                  the key name
     * @param flags
     *                  the flags to set (bitmask)
     */
    public static void setFlags(String key, int flags)
    {
        getEntry(key).setFlags(flags);
    }

    /**
     * Clears flags on the specified key in this table. The key can not be null.
     *
     * @param key
     *                  the key name
     * @param flags
     *                  the flags to clear (bitmask)
     */
    public static void clearFlags(String key, int flags)
    {
        getEntry(key).clearFlags(flags);
    }

    /**
     * Returns the flags for the specified key.
     *
     * @param key
     *                the key name
     * @return the flags, or 0 if the key is not defined
     */
    public static int getFlags(String key)
    {
        return getEntry(key).getFlags();
    }

    /**
     * Deletes the specified key in this table. The key can not be null.
     *
     * @param key
     *                the key name
     */
    public static void delete(String key)
    {
        SmartDashboard.delete(key);
    }

    /**
     * Put a boolean in the table.
     * 
     * @param key
     *                  the key to be assigned to
     * @param value
     *                  the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public static boolean putBoolean(String key, boolean value)
    {
        return getEntry(key).setBoolean(value);
    }

    /**
     * Gets the current value in the table, setting it if it does not exist.
     * 
     * @param key
     *                         the key
     * @param defaultValue
     *                         the default value to set if key does not exist.
     * @return False if the table key exists with a different type
     */
    public static boolean setDefaultBoolean(String key, boolean defaultValue)
    {
        return getEntry(key).setDefaultBoolean(defaultValue);
    }

    /**
     * Returns the boolean the key maps to. If the key does not exist or is of
     * different type, it will return the default value.
     * 
     * @param key
     *                         the key to look up
     * @param defaultValue
     *                         the value to be returned if no value is found
     * @return the value associated with the given key or the given default
     *         value if there is no value associated with the key
     */
    public static boolean getBoolean(String key, boolean defaultValue)
    {
        return getEntry(key).getBoolean(defaultValue);
    }

    /**
     * Put a number in the table.
     * 
     * @param key
     *                  the key to be assigned to
     * @param value
     *                  the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public static boolean putNumber(String key, double value)
    {
        return getEntry(key).setDouble(value);
    }

    /**
     * Gets the current value in the table, setting it if it does not exist.
     * 
     * @param key
     *                         the key
     * @param defaultValue
     *                         the default value to set if key does not exist.
     * @return False if the table key exists with a different type
     */
    public static boolean setDefaultNumber(String key, double defaultValue)
    {
        return getEntry(key).setDefaultDouble(defaultValue);
    }

    /**
     * Put a string in the table.
     * 
     * @param key
     *                  the key to be assigned to
     * @param value
     *                  the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public static boolean putString(String key, String value)
    {
        return getEntry(key).setString(value);
    }

    /**
     * Gets the current value in the table, setting it if it does not exist.
     * 
     * @param key
     *                         the key
     * @param defaultValue
     *                         the default value to set if key does not exist.
     * @return False if the table key exists with a different type
     */
    public static boolean setDefaultString(String key, String defaultValue)
    {
        return getEntry(key).setDefaultString(defaultValue);
    }

    /**
     * Put a boolean array in the table.
     * 
     * @param key
     *                  the key to be assigned to
     * @param value
     *                  the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public static boolean putBooleanArray(String key, boolean[] value)
    {
        return getEntry(key).setBooleanArray(value);
    }

    /**
     * Put a boolean array in the table.
     * 
     * @param key
     *                  the key to be assigned to
     * @param value
     *                  the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public static boolean putBooleanArray(String key, Boolean[] value)
    {
        return getEntry(key).setBooleanArray(value);
    }

    /**
     * Gets the current value in the table, setting it if it does not exist.
     * 
     * @param key
     *                         the key
     * @param defaultValue
     *                         the default value to set if key does not exist.
     * @return False if the table key exists with a different type
     */
    public static boolean setDefaultBooleanArray(String key, boolean[] defaultValue)
    {
        return getEntry(key).setDefaultBooleanArray(defaultValue);
    }

    /**
     * Gets the current value in the table, setting it if it does not exist.
     * 
     * @param key
     *                         the key
     * @param defaultValue
     *                         the default value to set if key does not exist.
     * @return False if the table key exists with a different type
     */
    public static boolean setDefaultBooleanArray(String key, Boolean[] defaultValue)
    {
        return getEntry(key).setDefaultBooleanArray(defaultValue);
    }

    /**
     * Returns the boolean array the key maps to. If the key does not exist or
     * is of different type, it will return the default value.
     * 
     * @param key
     *                         the key to look up
     * @param defaultValue
     *                         the value to be returned if no value is found
     * @return the value associated with the given key or the given default
     *         value if there is no value associated with the key
     */
    public static boolean[] getBooleanArray(String key, boolean[] defaultValue)
    {
        return getEntry(key).getBooleanArray(defaultValue);
    }

    /**
     * Returns the boolean array the key maps to. If the key does not exist or
     * is of different type, it will return the default value.
     * 
     * @param key
     *                         the key to look up
     * @param defaultValue
     *                         the value to be returned if no value is found
     * @return the value associated with the given key or the given default
     *         value if there is no value associated with the key
     */
    public static Boolean[] getBooleanArray(String key, Boolean[] defaultValue)
    {
        return getEntry(key).getBooleanArray(defaultValue);
    }

    /**
     * Put a number array in the table.
     * 
     * @param key
     *                  the key to be assigned to
     * @param value
     *                  the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public static boolean putNumberArray(String key, double[] value)
    {
        return getEntry(key).setDoubleArray(value);
    }

    /**
     * Put a number array in the table.
     * 
     * @param key
     *                  the key to be assigned to
     * @param value
     *                  the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public static boolean putNumberArray(String key, Double[] value)
    {
        return getEntry(key).setNumberArray(value);
    }

    /**
     * Gets the current value in the table, setting it if it does not exist.
     * 
     * @param key
     *                         the key
     * @param defaultValue
     *                         the default value to set if key does not exist.
     * @return False if the table key exists with a different type
     */
    public static boolean setDefaultNumberArray(String key, double[] defaultValue)
    {
        return getEntry(key).setDefaultDoubleArray(defaultValue);
    }

    /**
     * Gets the current value in the table, setting it if it does not exist.
     * 
     * @param key
     *                         the key
     * @param defaultValue
     *                         the default value to set if key does not exist.
     * @return False if the table key exists with a different type
     */
    public static boolean setDefaultNumberArray(String key, Double[] defaultValue)
    {
        return getEntry(key).setDefaultNumberArray(defaultValue);
    }

    /**
     * Returns the number array the key maps to. If the key does not exist or is
     * of different type, it will return the default value.
     * 
     * @param key
     *                         the key to look up
     * @param defaultValue
     *                         the value to be returned if no value is found
     * @return the value associated with the given key or the given default
     *         value if there is no value associated with the key
     */
    public static double[] getNumberArray(String key, double[] defaultValue)
    {
        return getEntry(key).getDoubleArray(defaultValue);
    }

    /**
     * Returns the number array the key maps to. If the key does not exist or is
     * of different type, it will return the default value.
     * 
     * @param key
     *                         the key to look up
     * @param defaultValue
     *                         the value to be returned if no value is found
     * @return the value associated with the given key or the given default
     *         value if there is no value associated with the key
     */
    public static Double[] getNumberArray(String key, Double[] defaultValue)
    {
        return getEntry(key).getDoubleArray(defaultValue);
    }

    /**
     * Put a string array in the table.
     * 
     * @param key
     *                  the key to be assigned to
     * @param value
     *                  the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public static boolean putStringArray(String key, String[] value)
    {
        return getEntry(key).setStringArray(value);
    }

    /**
     * Gets the current value in the table, setting it if it does not exist.
     * 
     * @param key
     *                         the key
     * @param defaultValue
     *                         the default value to set if key does not exist.
     * @return False if the table key exists with a different type
     */
    public static boolean setDefaultStringArray(String key, String[] defaultValue)
    {
        return getEntry(key).setDefaultStringArray(defaultValue);
    }

    /**
     * Returns the string array the key maps to. If the key does not exist or is
     * of different type, it will return the default value.
     * 
     * @param key
     *                         the key to look up
     * @param defaultValue
     *                         the value to be returned if no value is found
     * @return the value associated with the given key or the given default
     *         value if there is no value associated with the key
     */
    public static String[] getStringArray(String key, String[] defaultValue)
    {
        return getEntry(key).getStringArray(defaultValue);
    }

    /**
     * Put a raw value (byte array) in the table.
     * 
     * @param key
     *                  the key to be assigned to
     * @param value
     *                  the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public static boolean putRaw(String key, byte[] value)
    {
        return getEntry(key).setRaw(value);
    }

    /**
     * Put a raw value (bytes from a byte buffer) in the table.
     * 
     * @param key
     *                  the key to be assigned to
     * @param value
     *                  the value that will be assigned
     * @param len
     *                  the length of the value
     * @return False if the table key already exists with a different type
     */
    public static boolean putRaw(String key, ByteBuffer value, int len)
    {
        return getEntry(key).setRaw(value, len);
    }

    /**
     * Gets the current value in the table, setting it if it does not exist.
     * 
     * @param key
     *                         the key
     * @param defaultValue
     *                         the default value to set if key does not exist.
     * @return False if the table key exists with a different type
     */
    public static boolean setDefaultRaw(String key, byte[] defaultValue)
    {
        return getEntry(key).setDefaultRaw(defaultValue);
    }

    /**
     * Returns the raw value (byte array) the key maps to. If the key does not
     * exist or is of different type, it will return the default value.
     * 
     * @param key
     *                         the key to look up
     * @param defaultValue
     *                         the value to be returned if no value is found
     * @return the value associated with the given key or the given default
     *         value if there is no value associated with the key
     */
    public static byte[] getRaw(String key, byte[] defaultValue)
    {
        return getEntry(key).getRaw(defaultValue);
    }

    /**
     * Puts all sendable data to the dashboard.
     */
    public static synchronized void updateValues()
    {
        SmartDashboard.updateValues();
    }

} // class HalDashboard
