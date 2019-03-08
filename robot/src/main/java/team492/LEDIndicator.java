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

package team492;

import frclib.FrcRevBlinkin;
import trclib.TrcRevBlinkin.LEDPattern;

public class LEDIndicator
{
    private final LEDPattern normalPattern = LEDPattern.FixedFireMedium;
    private final LEDPattern cargoPattern = LEDPattern.SolidOrange;
    private final LEDPattern visionPattern = LEDPattern.SolidAqua;
    private final LEDPattern[] patterns = new LEDPattern[] { normalPattern, cargoPattern, visionPattern };

    private FrcRevBlinkin blinkin;

    public LEDIndicator()
    {
        blinkin = new FrcRevBlinkin("LEDIndicator", RobotInfo.PWM_REV_BLINKIN);
        blinkin.setPatternPriorities(patterns);
        blinkin.setPatternState(normalPattern, true);
    }

    public void reset()
    {
        blinkin.resetAllPatternStates();
        blinkin.setPatternState(normalPattern, true);
    }

    public void signalCargoDetected(boolean detected)
    {
        blinkin.setPatternState(cargoPattern, detected);
    }

    public void signalVisionDetected(boolean detected)
    {
        blinkin.setPatternState(visionPattern, detected);
    }

}
