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

import java.util.LinkedList;
import java.util.Queue;

public class TrcLoopTimeCounter
{
    private Queue<Double> periodQueue;
    private Double lastTime;
    private double timeSum;
    private double averageWindow;
    private double maxPeriod;
    private double minPeriod;

    public TrcLoopTimeCounter(double averageWindow)
    {
        periodQueue = new LinkedList<>();
        this.averageWindow = averageWindow;
        reset();
    }

    public void reset()
    {
        timeSum = 0.0;
        periodQueue.clear();
        lastTime = null;
        maxPeriod = 0.0;
        minPeriod = Double.POSITIVE_INFINITY;
    }

    public void update()
    {
        double currTime = TrcUtil.getCurrentTime();
        if (lastTime == null)
        {
            lastTime = currTime;
            return;
        }

        double deltaTime = currTime - lastTime;
        if (deltaTime > maxPeriod)
        {
            maxPeriod = deltaTime;
        }
        if (deltaTime < minPeriod)
        {
            minPeriod = deltaTime;
        }

        periodQueue.add(deltaTime);
        timeSum += deltaTime;

        while (timeSum > averageWindow)
        {
            timeSum -= periodQueue.remove();
        }
        lastTime = currTime;
    }

    public double getPeriod()
    {
        if (periodQueue.isEmpty())
        {
            return 0.0;
        }
        return timeSum / (double) periodQueue.size();
    }

    public double getFrequency()
    {
        if (timeSum == 0.0)
        {
            return 0.0;
        }
        return (double) periodQueue.size() / timeSum;
    }

    public double getMaxPeriod()
    {
        return maxPeriod;
    }

    public double getMinFrequency()
    {
        if (maxPeriod == 0.0)
        {
            return 0.0;
        }
        return 1.0 / maxPeriod;
    }

    public double getMinPeriod()
    {
        return minPeriod;
    }

    public double getMaxFrequency()
    {
        if (minPeriod == 0.0)
        {
            return 0.0;
        }
        return 1.0 / minPeriod;
    }
}
