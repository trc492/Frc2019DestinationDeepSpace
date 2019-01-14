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

import edu.wpi.first.wpilibj.AnalogInput;
import trclib.TrcAnalogInput;
import trclib.TrcDbgTrace;
import trclib.TrcFilter;
import trclib.TrcUtil;

/**
 * This class implements a platform dependent AnalogInput sensor extending TrcAnalogInput. It provides implementation
 * of the abstract methods in TrcAnalogInput.
 */
public class FrcAnalogInput extends TrcAnalogInput
{
    private static final double maxVoltage = 5.0;
    private AnalogInput sensor;
    private double sensorData;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param channel specifies the analog input channel.
     * @param filters specifies an array of filter objects, one for each axis, to filter sensor data. If no filter
     *                is used, this can be set to null.
     */
    public FrcAnalogInput(String instanceName, int channel, TrcFilter[] filters)
    {
        super(instanceName, 1, 0, filters);
        sensor = new AnalogInput(channel);
    }   //FrcAnalogInput

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param channel specifies the analog input channel.
     */
    public FrcAnalogInput(String instanceName, int channel)
    {
        this(instanceName, channel, null);
    }   //FrcAnalogInput

    /**
     * This method calibrates the sensor.
     */
    public void calibrate()
    {
        calibrate(DataType.INPUT_DATA);
    }   //calibrate

    //
    // Implements TrcAnalogInput abstract methods.
    //

    /**
     * This method returns the raw sensor data of the specified type.
     *
     * @param index specifies the data index (not used).
     * @param dataType specifies the data type.
     * @return raw sensor data of the specified type.
     */
    @Override
    public SensorData<Double> getRawData(int index, DataType dataType)
    {
        final String funcName = "getRawData";
        SensorData<Double> data;

        if (dataType == DataType.RAW_DATA)
        {
            sensorData = sensor.getVoltage();
        }
        else if (dataType == DataType.INPUT_DATA || dataType == DataType.NORMALIZED_DATA)
        {
            sensorData = sensor.getVoltage();
            if (dataType == DataType.NORMALIZED_DATA)
            {
                sensorData /= maxVoltage;
            }
        }
        else
        {
            throw new UnsupportedOperationException(
                    "AnalogInput sensor only support INPUT_DATA/NORMALIZED_DATA types.");
        }
        data = new SensorData<>(TrcUtil.getCurrentTime(), sensorData);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp:%.3f,value=%f)", data.timestamp, data.value);
        }

        return data;
    }   //getRawData

}   //class FrcAnalogInput
