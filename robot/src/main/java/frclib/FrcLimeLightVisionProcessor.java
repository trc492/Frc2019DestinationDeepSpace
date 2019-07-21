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
import trclib.TrcUtil;

import java.util.function.DoubleSupplier;
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

    private NetworkTableEntry tv, heading;
    private NetworkTableEntry ledMode, pipeline;
    private DoubleSupplier depthSupplier = () -> 0.0;

    public FrcLimeLightVisionProcessor(String instanceName, String depthInput, DoubleUnaryOperator depthApproximator)
    {
        this(instanceName);
        setDepthApproximator(depthInput, depthApproximator);
    }

    public FrcLimeLightVisionProcessor(String instanceName)
    {
        super(instanceName, "limelight");
        tv = super.networkTable.getEntry("tv");
        ledMode = super.networkTable.getEntry("ledMode");
        pipeline = super.networkTable.getEntry("pipeline");
        heading = super.networkTable.getEntry("tx");
    }

    public double getTargetDepth()
    {
        return depthSupplier.getAsDouble();
    }

    public void setDepthApproximator(String input, DoubleUnaryOperator depthApproximator)
    {
        depthSupplier = () -> depthApproximator.applyAsDouble(networkTable.getEntry(input).getDouble(0.0));
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

    public double getTargetArea()
    {
        return get("ta");
    }

    public double getTargetHeight()
    {
        return get("tvert");
    }

    @Override
    public void setRingLightEnabled(boolean enabled)
    {
        setRingLightEnabled(enabled ? RingLightMode.ON : RingLightMode.OFF);
    }

    public void setRingLightEnabled(RingLightMode mode)
    {
        ledMode.setDouble(mode.getValue());
    }

    @Override
    protected RelativePose processData()
    {
        if (!targetDetected())
        {
            return null;
        }

        RelativePose pose = new RelativePose();
        pose.time = TrcUtil.getCurrentTime();
        pose.theta = getHeading();
        pose.r = depthSupplier.getAsDouble();
        pose.x = Math.sin(Math.toRadians(pose.theta)) * pose.r;
        pose.y = Math.cos(Math.toRadians(pose.theta)) * pose.r;
        pose.objectYaw = 0.0;
        return pose;
    }
}
