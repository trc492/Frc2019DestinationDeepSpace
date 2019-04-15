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

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TrcTankMotionProfile
{

    public static TrcTankMotionProfile loadProfileFromCsv(String leftPath, String rightPath)
    {
        return loadProfileFromCsv(leftPath, rightPath, false);
    }

    public static TrcTankMotionProfile loadProfileFromCsv(String leftPath, String rightPath, boolean loadFromResources)
    {
        return new TrcTankMotionProfile(TrcMotionProfilePoint.loadPointsFromCsv(leftPath, loadFromResources),
            TrcMotionProfilePoint.loadPointsFromCsv(rightPath, loadFromResources));
    }

    private TrcMotionProfilePoint[] leftPoints, rightPoints;

    public TrcTankMotionProfile(TrcMotionProfilePoint[] leftPoints, TrcMotionProfilePoint[] rightPoints)
    {
        if (leftPoints.length != rightPoints.length)
        {
            throw new IllegalArgumentException("leftPoints and rightPoints must have the same length!");
        }
        this.leftPoints = leftPoints;
        this.rightPoints = rightPoints;
    }

    public TrcMotionProfilePoint[] getLeftPoints()
    {
        return leftPoints;
    }

    public TrcMotionProfilePoint[] getRightPoints()
    {
        return rightPoints;
    }

    /**
     * Scale the motion profile by distance and time
     * 
     * @param worldUnitsPerEncoderTick Number of world units in a single encoder tick
     * @param timeStep Speed denomination in seconds.
     */
    public void scale(double worldUnitsPerEncoderTick, double timeStep)
    {
        for (TrcMotionProfilePoint point : leftPoints)
        {
            point.encoderPosition /= worldUnitsPerEncoderTick;
            point.velocity /= worldUnitsPerEncoderTick / timeStep;
            point.acceleration /= worldUnitsPerEncoderTick / Math.pow(timeStep, 2);
            point.jerk /= worldUnitsPerEncoderTick / Math.pow(timeStep, 3);
        }

        for (TrcMotionProfilePoint point : rightPoints)
        {
            point.encoderPosition /= worldUnitsPerEncoderTick;
            point.velocity /= worldUnitsPerEncoderTick / timeStep;
            point.acceleration /= worldUnitsPerEncoderTick / Math.pow(timeStep, 2);
            point.jerk /= worldUnitsPerEncoderTick / Math.pow(timeStep, 3);
        }
    }

    public int getNumPoints()
    {
        return leftPoints.length;
    }

    public double getMinTimeStep()
    {
//        OptionalDouble minTimeStep = Stream.concat(Arrays.stream(leftPoints), Arrays.stream(rightPoints))
//            .mapToDouble(p -> p.timeStep).min();
//        if (minTimeStep.isPresent())
//        {
//            return minTimeStep.getAsDouble();
//        }
//        throw new IllegalStateException("Empty motion profile!");
        double minTimeStep = Double.POSITIVE_INFINITY;

        for (int i = 0; i < leftPoints.length; i++)
        {
            if (leftPoints[i].timeStep < minTimeStep)
            {
                minTimeStep = leftPoints[i].timeStep;
            }
        }

        for (int i = 0; i < rightPoints.length; i++)
        {
            if (rightPoints[i].timeStep < minTimeStep)
            {
                minTimeStep = leftPoints[i].timeStep;
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
//        TrcMotionProfilePoint[] left = Arrays.stream(leftPoints).map(TrcMotionProfilePoint::new)
//            .toArray(TrcMotionProfilePoint[]::new);
//        TrcMotionProfilePoint[] right = Arrays.stream(rightPoints).map(TrcMotionProfilePoint::new)
//            .toArray(TrcMotionProfilePoint[]::new);
        TrcMotionProfilePoint[] left = Arrays.copyOf(leftPoints, leftPoints.length);
        TrcMotionProfilePoint[] right = Arrays.copyOf(rightPoints, rightPoints.length);
        return new TrcTankMotionProfile(left, right);
    }
}
