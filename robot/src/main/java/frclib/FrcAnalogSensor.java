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

package frclib;

import edu.wpi.first.wpilibj.AnalogInput;

/**
 * This class implements an analog sensor that provides method to read the sensor and scale to proper unit.
 */
public class FrcAnalogSensor extends AnalogInput
{
    private final String instanceName;
    private final double sensorScale;
    private final double sensorBias;

    /**
     * Constructor: Create an instance of the object. 
     *
     * @param instanceName specifies the instance name.
     * @param analogChannel specifies the analog channel the pressure sensor is on.
     * @param sensorScale specifies the scale factor to convert voltage into proper unit.
     * @param sensorBias specifies the sensor voltage bias.
     */
    public FrcAnalogSensor(String instanceName, int analogChannel, double sensorScale, double sensorBias)
    {
        super(analogChannel);
        this.instanceName = instanceName;
        this.sensorScale = sensorScale;
        this.sensorBias = sensorBias;
    }   //FrcAnalogSensor

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method reads the sensor value and converts it to the scaled unit.
     *
     * @return scaled sensor value.
     */
    public double getScaledValue()
    {
        return (getVoltage() - sensorBias) * sensorScale;
    }   //getScaledValue

}   //class FrcAnalogSensor
