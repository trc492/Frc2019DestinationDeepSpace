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

import frclib.FrcLimeLightVisionProcessor;
import frclib.FrcRemoteVisionProcessor;

public class VisionTargeting
{
    private static final boolean USE_RASPI = false;
    private static final double CAMERA_HEIGHT = 41.25;
    private static final double TARGET_HEIGHT = 29;
    private static final double CAMERA_ANGLE = -24.7;

    private FrcLimeLightVisionProcessor vision;

    public VisionTargeting()
    {
        // This equation is the best fit line for a few data points to convert target height -> depth
        vision = new FrcLimeLightVisionProcessor("LimeLight");

        vision.setDepthApproximator(y -> -(CAMERA_HEIGHT - TARGET_HEIGHT) / Math.tan(Math.toRadians(y + CAMERA_ANGLE)));
        vision.setUse3DOverride(false);
        vision.setOffsets(RobotInfo.CAMERA_OFFSET, RobotInfo.CAMERA_DEPTH);
        //vision.setFreshnessTimeout(RobotInfo.CAMERA_DATA_TIMEOUT);
    }

    public FrcLimeLightVisionProcessor getVision()
    {
        return vision;
    }

    public FrcRemoteVisionProcessor.RelativePose getLastPose()
    {
        return vision.getLastPose();
    }

    public void setRingLightEnabled(boolean enabled)
    {
        vision.setRingLightEnabled(enabled);
    }

    public FrcRemoteVisionProcessor.RelativePose getMedianPose(int numFrames, boolean requireAll)
    {
        return vision.getMedianPose(numFrames, requireAll);
    }
}
