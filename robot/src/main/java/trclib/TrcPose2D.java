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

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealVector;

public class TrcPose2D
{
    public double x, y, heading, xVel, yVel;

    public TrcPose2D(double x, double y)
    {
        this(x, y, 0, 0, 0);
    }

    public TrcPose2D(double x, double y, double heading)
    {
        this(x, y, heading, 0, 0);
    }

    public TrcPose2D(double x, double y, double heading, double xVel, double yVel)
    {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.xVel = xVel;
        this.yVel = yVel;
    }

    public RealVector getPositionVector()
    {
        return MatrixUtils.createRealVector(new double[] { x, y });
    }

    public RealVector getVelocityVector()
    {
        return MatrixUtils.createRealVector(new double[] { xVel, yVel });
    }

    public TrcPose2D minus(TrcPose2D pose)
    {
        return new TrcPose2D(x - pose.x, y - pose.y, heading, xVel, yVel);
    }

    public TrcPose2D inReferenceFrameOf(TrcPose2D pose)
    {
        TrcPose2D transformed = this.minus(pose);
        RealVector newPos = TrcUtil.rotateCCW(transformed.getPositionVector(), pose.heading);
        RealVector newVel = TrcUtil.rotateCCW(transformed.getVelocityVector(), pose.heading);

        transformed.x = newPos.getEntry(0);
        transformed.y = newPos.getEntry(1);

        transformed.xVel = newVel.getEntry(0);
        transformed.yVel = newVel.getEntry(1);
        return transformed;
    }
}
