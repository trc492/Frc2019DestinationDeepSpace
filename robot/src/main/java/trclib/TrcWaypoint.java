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

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;

public class TrcWaypoint
{
    public static TrcWaypoint[] loadPointsFromCsv(String path, boolean loadFromResources)
    {
        if (!path.endsWith(".csv"))
        {
            throw new IllegalArgumentException(String.format("%s is not a csv file!", path));
        }
        try
        {
            BufferedReader in;
            if (loadFromResources)
            {
                in = new BufferedReader(
                    new InputStreamReader(TrcWaypoint.class.getClassLoader().getResourceAsStream(path)));
            }
            else
            {
                in = new BufferedReader(new FileReader(path));
            }

            List<TrcWaypoint> points = new ArrayList<>();
            String line;
            in.readLine(); // Get rid of the first line
            while ((line = in.readLine()) != null)
            {
                String[] tokens = line.split(",");
                double[] parts = new double[tokens.length];
                for (int i = 0; i < parts.length; i++)
                {
                    parts[i] = Double.parseDouble(tokens[i]);
                }
                if (parts.length != 8)
                {
                    throw new IllegalArgumentException("There must be 8 columns in the csv file!");
                }
                TrcWaypoint point = new TrcWaypoint(parts[0], parts[1], parts[2], parts[3],
                    parts[4], parts[5], parts[6], parts[7]);
                points.add(point);
            }
            in.close();
            return points.toArray(new TrcWaypoint[0]);
        }
        catch (IOException e)
        {
            throw new RuntimeException(e);
        }
    }

    public double timeStep, x, y, encoderPosition, velocity, acceleration, jerk, heading;

    public TrcWaypoint(double timeStep, double x, double y, double position, double velocity,
        double acceleration, double jerk, double heading)
    {
        this.timeStep = timeStep;
        this.x = x;
        this.y = y;
        this.encoderPosition = position;
        this.velocity = velocity;
        this.acceleration = acceleration;
        this.jerk = jerk;
        this.heading = heading;
    }

    public TrcWaypoint(TrcWaypoint other)
    {
        this.timeStep = other.timeStep;
        this.x = other.x;
        this.y = other.y;
        this.encoderPosition = other.encoderPosition;
        this.velocity = other.velocity;
        this.acceleration = other.acceleration;
        this.jerk = other.jerk;
        this.heading = other.heading;
    }

    public double distanceTo(TrcWaypoint point)
    {
        return TrcUtil.magnitude(point.x - x, point.y - y);
    }
}
