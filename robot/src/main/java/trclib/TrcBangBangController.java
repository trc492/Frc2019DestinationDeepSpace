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

public class TrcBangBangController
{
    private double forwardPower, reversePower;
    private TrcMotor motor;
    private TrcTaskMgr.TaskObject bangBangTaskObj;
    private double target;
    private TrcEvent onFinishedEvent;

    public TrcBangBangController(String instanceName, double forwardPower, double reversePower, TrcMotor motor)
    {
        this.forwardPower = forwardPower;
        this.reversePower = reversePower;
        this.motor = motor;

        bangBangTaskObj = TrcTaskMgr.getInstance().createTask(instanceName + ".bangBangTask", this::bangBangTask);
    }

    private void stop()
    {
        motor.set(0.0);
        bangBangTaskObj.unregisterTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        onFinishedEvent = null;
    }

    public void setTarget(double position, TrcEvent onFinishedEvent)
    {
        bangBangTaskObj.registerTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        target = position;
        if (onFinishedEvent != null)
        {
            onFinishedEvent.clear();
        }
        this.onFinishedEvent = onFinishedEvent;
    }

    private void bangBangTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        if (motor.getPosition() < target)
        {
            motor.set(forwardPower);
        }
        else if (motor.getPosition() > target)
        {
            motor.set(reversePower);
        }
        else
        {
            motor.set(0.0);
            if (onFinishedEvent != null)
            {
                onFinishedEvent.set(true);
            }
            stop();
        }
    }
}
