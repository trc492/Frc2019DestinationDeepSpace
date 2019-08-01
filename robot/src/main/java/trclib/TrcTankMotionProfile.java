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

package trclib;

import java.util.Arrays;

public class TrcTankMotionProfile
{
    public static TrcTankMotionProfile loadProfileFromCsv(String leftPath, String rightPath)
    {
        return loadProfileFromCsv(leftPath, rightPath, false);
    }

    public static TrcTankMotionProfile loadProfileFromCsv(String leftPath, String rightPath, boolean loadFromResources)
    {
        return new TrcTankMotionProfile(TrcWaypoint.loadPointsFromCsv(leftPath, loadFromResources),
            TrcWaypoint.loadPointsFromCsv(rightPath, loadFromResources));
    }

    private TrcPath leftPath, rightPath;

    public TrcTankMotionProfile(TrcWaypoint[] leftPoints, TrcWaypoint[] rightPoints)
    {
        if (leftPoints.length != rightPoints.length)
        {
            throw new IllegalArgumentException("leftPoints and rightPoints must have the same length!");
        }
        this.leftPath = new TrcPath(true, leftPoints);
        this.rightPath = new TrcPath(true, rightPoints);
    }

    public TrcWaypoint[] getLeftPoints()
    {
        return leftPath.getAllWaypoints();
    }

    public TrcWaypoint[] getRightPoints()
    {
        return rightPath.getAllWaypoints();
    }

    /**
     * Scale the motion profile by distance and time
     * 
     * @param worldUnitsPerEncoderTick Number of world units in a single encoder tick
     * @param timeStep Speed denomination in seconds.
     */
    public void scale(double worldUnitsPerEncoderTick, double timeStep)
    {
        for (TrcWaypoint point : leftPath.getAllWaypoints())
        {
            point.encoderPosition /= worldUnitsPerEncoderTick;
            point.velocity /= worldUnitsPerEncoderTick / timeStep;
            point.acceleration /= worldUnitsPerEncoderTick / Math.pow(timeStep, 2);
            point.jerk /= worldUnitsPerEncoderTick / Math.pow(timeStep, 3);
        }

        for (TrcWaypoint point : rightPath.getAllWaypoints())
        {
            point.encoderPosition /= worldUnitsPerEncoderTick;
            point.velocity /= worldUnitsPerEncoderTick / timeStep;
            point.acceleration /= worldUnitsPerEncoderTick / Math.pow(timeStep, 2);
            point.jerk /= worldUnitsPerEncoderTick / Math.pow(timeStep, 3);
        }
    }

    public int getNumPoints()
    {
        return leftPath.getSize();
    }

    public double getMinTimeStep()
    {
        double minTimeStep = Double.POSITIVE_INFINITY;

        for (int i = 0; i < leftPath.getSize(); i++)
        {
            if (leftPath.getWaypoint(i).timeStep < minTimeStep)
            {
                minTimeStep = leftPath.getWaypoint(i).timeStep;
            }
        }

        for (int i = 0; i < rightPath.getSize(); i++)
        {
            if (rightPath.getWaypoint(i).timeStep < minTimeStep)
            {
                minTimeStep = rightPath.getWaypoint(i).timeStep;
            }
        }

        if (minTimeStep == Double.POSITIVE_INFINITY)
        {
            throw new IllegalStateException("Empty motion profile!");
        }
        else
        {
            return minTimeStep;
        }
    }

    public TrcTankMotionProfile copy()
    {
        TrcWaypoint[] left = Arrays.copyOf(getLeftPoints(), leftPath.getSize());
        TrcWaypoint[] right = Arrays.copyOf(getRightPoints(), rightPath.getSize());
        return new TrcTankMotionProfile(left, right);
    }
}
