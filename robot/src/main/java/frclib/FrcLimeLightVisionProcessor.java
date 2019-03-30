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

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableValue;
import trclib.TrcUtil;

import java.util.function.DoubleUnaryOperator;

public class FrcLimeLightVisionProcessor extends FrcRemoteVisionProcessor
{
    public enum RingLightMode
    {
        AUTO(0), OFF(1), BLINK(2), ON(3);

        private int value;

        RingLightMode(int value)
        {
            this.value = value;
        }

        public int getValue()
        {
            return value;
        }
    }

    private NetworkTableEntry tv, heading, area;
    private NetworkTableEntry ledMode, pipeline;
    private DoubleUnaryOperator depthApproximator;

    public FrcLimeLightVisionProcessor(String instanceName)
    {
        this(instanceName, "camtran");
    }

    public FrcLimeLightVisionProcessor(String instanceName, DoubleUnaryOperator depthApproximator)
    {
        this(instanceName, "ta");
        this.depthApproximator = depthApproximator;
    }

    private FrcLimeLightVisionProcessor(String instanceName, String dataKey)
    {
        super(instanceName, "limelight", dataKey);
        tv = super.networkTable.getEntry("tv");
        ledMode = super.networkTable.getEntry("ledMode");
        pipeline = super.networkTable.getEntry("pipeline");
        heading = super.networkTable.getEntry("tx");
        area = super.networkTable.getEntry("ta");
    }

    public boolean targetDetected()
    {
        return tv.getDouble(0.0) == 1.0;
    }

    public void selectPipeline(int pipeline)
    {
        this.pipeline.setDouble(pipeline);
    }

    public double getHeading()
    {
        return heading.getDouble(0.0);
    }

    public double getArea()
    {
        return area.getDouble(0.0);
    }

    @Override
    public void setRingLightEnabled(boolean enabled)
    {
        setRingLightEnabled(enabled ? RingLightMode.ON : RingLightMode.OFF);
    }

    private boolean availableData(NetworkTableValue data)
    {
        return targetDetected() && ((depthApproximator == null && data.isDoubleArray()) || (depthApproximator != null
            && data.isDouble()));
    }

    public void setRingLightEnabled(RingLightMode mode)
    {
        ledMode.setDouble(mode.getValue());
    }

    @Override
    protected RelativePose processData(NetworkTableValue data)
    {
        if (!availableData(data))
        {
            return null;
        }

        RelativePose pose = new RelativePose();
        pose.time = TrcUtil.getCurrentTime();
        if (depthApproximator == null)
        {
            // According to LL docs, format is [x, y, z, pitch, yaw, roll]
            double[] values = data.getDoubleArray();
            pose.x = -values[0]; // x axis is relative to target instead of robot, so invert
            pose.y = -values[2]; // robot y axis is camera z axis, relative to target, invert
            pose.r = TrcUtil.magnitude(pose.x, pose.y);
            pose.theta = getHeading();
            pose.objectYaw = values[4];
        }
        else
        {
            pose.theta = getHeading();
            pose.r = depthApproximator.applyAsDouble(getArea());
            pose.x = Math.sin(Math.toRadians(pose.theta)) * pose.r;
            pose.y = Math.cos(Math.toRadians(pose.theta)) * pose.r;
            pose.objectYaw = 0.0;
        }
        return pose;
    }
}
