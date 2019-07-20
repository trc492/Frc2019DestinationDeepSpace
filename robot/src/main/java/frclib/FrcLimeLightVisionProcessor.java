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

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import trclib.TrcUtil;

import java.util.Arrays;
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

    private NetworkTableEntry tv, heading, area, height, camtran, ty;
    private NetworkTableEntry ledMode, pipeline;
    private DoubleUnaryOperator depthApproximator;
    private volatile double cacheX, cacheY;
    private boolean use3D = true;

    public FrcLimeLightVisionProcessor(String instanceName, DoubleUnaryOperator depthApproximator)
    {
        this(instanceName);
        this.depthApproximator = depthApproximator;
        use3D = false;
    }

    public FrcLimeLightVisionProcessor(String instanceName)
    {
        super(instanceName, "limelight");
        tv = super.networkTable.getEntry("tv");
        ledMode = super.networkTable.getEntry("ledMode");
        pipeline = super.networkTable.getEntry("pipeline");
        heading = super.networkTable.getEntry("tx");
        area = super.networkTable.getEntry("ta");
        camtran = super.networkTable.getEntry("camtran");
        height = super.networkTable.getEntry("tvert");
        ty = super.networkTable.getEntry("ty");

        int flag = EntryListenerFlags.kImmediate | EntryListenerFlags.kNew | EntryListenerFlags.kUpdate;
        heading.addListener(e -> cacheX = e.value.getDouble(), flag);
        ty.addListener(e -> cacheY = depthApproximator != null ? depthApproximator.applyAsDouble(e.value.getDouble()) : 0.0, flag);
    }

    public double getX()
    {
        // TODO: Move this OUT of the frclib layer
        return cacheX + 12.04; // technically not heading, but whatever
    }

    public double getY()
    {
        // this is robot y, not image y
        return cacheY;
    }

    /**
     * Override the automatic switching/fallback behavior of using camtran mode.
     *
     * @param override If true, use 3d. Else, use area to approximate depth.
     */
    public void setUse3DOverride(boolean override)
    {
        if (depthApproximator == null && !override)
        {
            throw new IllegalStateException("Must set the depth approximator before disabling 3D!");
        }
        use3D = override;
    }

    public void setDepthApproximator(DoubleUnaryOperator depthApproximator)
    {
        // TODO: differentiate between the input variable? idk
        this.depthApproximator = depthApproximator;
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
        return area.getDouble(0.0);
    }

    public double getTargetHeight()
    {
        return height.getDouble(0.0);
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
        double[] data3d = camtran.getDoubleArray(new double[6]);
        if (use3D && Arrays.stream(data3d).anyMatch(d -> d != 0.0))
        {
            // According to LL docs, format is [x, y, z, pitch, yaw, roll]
            pose.x = -data3d[0]; // x axis is relative to target instead of robot, so invert
            pose.y = -data3d[2]; // robot y axis is camera z axis, relative to target, invert
            pose.r = TrcUtil.magnitude(pose.x, pose.y);
            pose.theta = getHeading();
            pose.objectYaw = data3d[4];
        }
        else
        {
            pose.theta = getHeading();
            pose.r = depthApproximator.applyAsDouble(getTargetHeight());
            pose.x = Math.sin(Math.toRadians(pose.theta)) * pose.r;
            pose.y = Math.cos(Math.toRadians(pose.theta)) * pose.r;
            pose.objectYaw = 0.0;
        }
        return pose;
    }
}
