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

package frclib;

import edu.wpi.first.wpilibj.GyroBase;
import trclib.TrcDbgTrace;
import trclib.TrcGyro;
import trclib.TrcUtil;

public class FrcGyro extends TrcGyro
{
    private GyroBase gyro;
    private double zSign = 1.0;

    public FrcGyro(final String instanceName, GyroBase gyro)
    {
        super(instanceName, 1, GYRO_HAS_Z_AXIS, null);
        this.gyro = gyro;
        gyro.calibrate();
    }   //FrcGyro

    //
    // Implements TrcGyro abstract methods.
    //

    /**
     * This method returns the raw data of the specified type for the x-axis which is not supported.
     *
     * @param dataType specifies the data type.
     * @return throws UnsupportedOperation exception.
     */
    public SensorData<Double> getRawXData(DataType dataType)
    {
        final String funcName = "getRawXData";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=null");
        }

        throw new UnsupportedOperationException("Gyro does not support x-axis.");
    }   //getRawXData

    /**
     * This method returns the raw data of the specified type for the y-axis which is not supported.
     *
     * @param dataType specifies the data type.
     * @return throws UnsupportedOperation exception.
     */
    public SensorData<Double> getRawYData(DataType dataType)
    {
        final String funcName = "getRawYData";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=null");
        }

        throw new UnsupportedOperationException("Gyro does not support y-axis.");
    }   //getRawYData

    /**
     * This method returns the raw data of the specified type for the z-axis.
     *
     * @param dataType specifies the data type.
     * @return raw data of the specified type for the z-axis in degrees per second.
     */
    public SensorData<Double> getRawZData(DataType dataType)
    {
        final String funcName = "getRawZData";
        double value = 0.0;

        if (dataType == DataType.ROTATION_RATE)
        {
            value = gyro.getRate();
        }
        else if (dataType == DataType.HEADING)
        {
            value = gyro.getAngle();
        }
        SensorData<Double> data = new SensorData<>(TrcUtil.getCurrentTime(), value);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp:%.3f,value:%f", data.timestamp, data.value);
        }

        return data;
    }   //getRawZData

    /**
     * This method inverts the x-axis. This is useful if the orientation of the gyro x-axis is such that the data
     * goes the wrong direction.
     *
     * @param inverted specifies true to invert x-axis, false otherwise.
     */
    public void setXInverted(boolean inverted)
    {
        throw new UnsupportedOperationException("Gyro does not support x-axis.");
    }   //setXInverted

    /**
     * This method inverts the y-axis. This is useful if the orientation of the gyro y-axis is such that the data
     * goes the wrong direction.
     *
     * @param inverted specifies true to invert y-axis, false otherwise.
     */
    public void setYInverted(boolean inverted)
    {
        throw new UnsupportedOperationException("Gyro does not support y-axis.");
    }   //setYInverted

    /**
     * This method inverts the z-axis. This is useful if the orientation of the gyro z-axis is such that the data
     * goes the wrong direction.
     *
     * @param inverted specifies true to invert z-axis, false otherwise.
     */
    public void setZInverted(boolean inverted)
    {
        zSign = inverted? -1.0: 1.0;
    }   //setZInverted

    /**
     * This method returns the rotation rate on the x-axis.
     *
     * @return X rotation rate.
     */
    public SensorData<Double> getXRotationRate()
    {
        throw new UnsupportedOperationException("Gyro does not support x-axis.");
    }   //getXRotationRate

    /**
     * This method returns the rotation rate on the y-axis.
     *
     * @return Y rotation rate.
     */
    public SensorData<Double> getYRotationRate()
    {
        throw new UnsupportedOperationException("Gyro does not support y-axis.");
    }   //getYRotationRate

    /**
     * This method returns the rotation rate on the z-axis.
     *
     * @return Z rotation rate.
     */
    public SensorData<Double> getZRotationRate()
    {
        return new SensorData<>(TrcUtil.getCurrentTime(), zSign*gyro.getRate());
    }   //getZRotationRate

    /**
     * This method returns the heading of the x-axis. If there is an integrator, we call the integrator to get
     * the heading. Else if we have a CardinalConverter, we call the converter to get the heading else we call
     * the platform dependent gyro to get the raw heading value.
     *
     * @return X heading.
     */
    public SensorData<Double> getXHeading()
    {
        throw new UnsupportedOperationException("Gyro does not support x-axis.");
    }   //getXHeading

    /**
     * This method returns the heading of the y-axis. If there is an integrator, we call the integrator to get
     * the heading. Else if we have a CardinalConverter, we call the converter to get the heading else we call
     * the platform dependent gyro to get the raw heading value.
     *
     * @return Y heading.
     */
    public SensorData<Double> getYHeading()
    {
        throw new UnsupportedOperationException("Gyro does not support y-axis.");
    }   //getYHeading

    /**
     * This method returns the heading of the z-axis. If there is an integrator, we call the integrator to get
     * the heading. Else if we have a CardinalConverter, we call the converter to get the heading else we call
     * the platform dependent gyro to get the raw heading value.
     *
     * @return Z heading.
     */
    public SensorData<Double> getZHeading()
    {
        return new SensorData<>(TrcUtil.getCurrentTime(), zSign*gyro.getAngle());
    }   //getZHeading

    /**
     * This method resets the integrator on the x-axis.
     */
    public void resetXIntegrator()
    {
        throw new UnsupportedOperationException("Gyro does not support x-axis.");
    }   //resetXIntegrator

    /**
     * This method resets the integrator on the y-axis.
     */
    public void resetYIntegrator()
    {
        throw new UnsupportedOperationException("Gyro does not support y-axis.");
    }   //resetYIntegrator

    /**
     * This method resets the integrator on the z-axis.
     */
    public void resetZIntegrator()
    {
        gyro.reset();
    }   //resetZIntegrator

}   //class FrcGyro
