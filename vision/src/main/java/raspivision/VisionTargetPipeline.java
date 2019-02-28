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

package raspivision;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;

import edu.wpi.first.vision.VisionPipeline;
import org.opencv.core.*;

import org.opencv.imgproc.*;

public class VisionTargetPipeline implements VisionPipeline
{
    private static final double ROTATED_RECT_RATIO_MIN = 0.7 * 2 / 5.5; // 80% of the aspect ratio of the vision tape
    private static final double ROTATED_RECT_RATIO_MAX = 1.3 * 2 / 5.5; // 120% of the aspect ratio of the vision tape

    //Outputs
    private Mat input = new Mat();
    private Mat hsvThresholdOutput = new Mat();
    private ArrayList<MatOfPoint> findContoursOutput = new ArrayList<>();
    private ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<>();
    private ArrayList<MatOfPoint> convexHullsOutput = new ArrayList<>();
    private List<TargetData> detectedTargets = new ArrayList<>();
    private TargetData selectedTarget = null;

    public double[] hsvThresholdHue = { 50.0, 90.0 };
    public double[] hsvThresholdSaturation = { 40, 255 };
    public double[] hsvThresholdLuminance = { 50, 255 };

    /**
     * This is the primary method that runs the entire pipeline and updates the outputs.
     */
    public void process(Mat source0)
    {
        long start = System.currentTimeMillis();

        try
        {
            // Clear the detected targets
            detectedTargets.clear();
            selectedTarget = null;

            source0.copyTo(input);
            // Step HSV_Threshold0:
            Mat hsvThresholdInput = source0;
            hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdLuminance,
                hsvThresholdOutput);

            // Step Find_Contours0:
            Mat findContoursInput = hsvThresholdOutput;
            findContours(findContoursInput, true, findContoursOutput);

            // Step Filter_Contours0:
            ArrayList<MatOfPoint> filterContoursContours = findContoursOutput;
            double filterContoursMinArea = 50.0;
            double filterContoursMinPerimeter = 0.0;
            double filterContoursMinWidth = 0.0;
            double filterContoursMaxWidth = 300.0;
            double filterContoursMinHeight = 0.0;
            double filterContoursMaxHeight = 200;
            double[] filterContoursSolidity = { 50, 100 };
            double filterContoursMaxVertices = 1000000.0;
            double filterContoursMinVertices = 0.0;
            double filterContoursMinRatio = 0.55;
            double filterContoursMaxRatio = 0.85;
            filterContours(filterContoursContours, filterContoursMinArea, filterContoursMinPerimeter,
                filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight,
                filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio,
                filterContoursMaxRatio, filterContoursOutput);

            // Step Convex_Hulls0:
            ArrayList<MatOfPoint> convexHullsContours = filterContoursOutput;
            convexHulls(convexHullsContours, convexHullsOutput);

            if (convexHullsOutput.isEmpty())
            {
                return;
            }

            // Convert contours to vision targets
            ArrayList<VisionTarget> visionTargets = convexHullsOutput.stream().map(this::mapContourToVisionTarget)
                .filter(Objects::nonNull).sorted(this::compareVisionTargets)
                .collect(Collectors.toCollection(ArrayList::new));

            if (visionTargets.isEmpty())
            {
                return;
            }

            // Trim mismatched targets
            while (!visionTargets.get(0).isLeftTarget)
            {
                visionTargets.remove(0);
                if (visionTargets.isEmpty())
                {
                    return;
                }
            }
            while (visionTargets.get(visionTargets.size() - 1).isLeftTarget)
            {
                visionTargets.remove(visionTargets.size() - 1);
                if (visionTargets.isEmpty())
                {
                    return;
                }
            }

            if (isValid(visionTargets))
            {
                detectedTargets = detectTargets(visionTargets);
                int width = source0.width();
                selectedTarget = detectedTargets.stream().min(Comparator.comparingInt(e -> Math.abs(e.x - width)))
                    .orElseThrow(IllegalStateException::new);
            }
        }
        finally
        {
            long elapsed = System.currentTimeMillis() - start;
            if (elapsed >= 100)
            {
                //System.err.println("Vision pipeline took too long!");
            }
        }
    }

    public Mat getHsvThresholdOutput()
    {
        return hsvThresholdOutput;
    }

    public TargetData getSelectedTarget()
    {
        return selectedTarget;
    }

    public List<TargetData> getDetectedTargets()
    {
        return detectedTargets;
    }

    public Mat getInput()
    {
        return input;
    }

    /**
     * This method is a generated getter for the output of a Convex_Hulls.
     *
     * @return ArrayList<MatOfPoint> output from Convex_Hulls.
     */
    public ArrayList<MatOfPoint> convexHullsOutput()
    {
        return convexHullsOutput;
    }

    /**
     * Segment an image based on hue, saturation, and luminance ranges.
     *
     * @param input The image on which to perform the HSV threshold.
     * @param hue   The min and max hue
     * @param sat   The min and max saturation
     * @param val   The min and max value
     * @param out   The image in which to store the output.
     */
    private void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val, Mat out)
    {
        Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HSV);
        Core.inRange(out, new Scalar(hue[0], val[0], sat[0]), new Scalar(hue[1], val[1], sat[1]), out);
    }

    private void findContours(Mat input, boolean externalOnly, List<MatOfPoint> contours)
    {
        Mat hierarchy = new Mat();
        contours.clear();
        int mode;
        if (externalOnly)
        {
            mode = Imgproc.RETR_EXTERNAL;
        }
        else
        {
            mode = Imgproc.RETR_LIST;
        }
        int method = Imgproc.CHAIN_APPROX_SIMPLE;
        Imgproc.findContours(input, contours, hierarchy, mode, method);
    }

    /**
     * Filters out contours that do not meet certain criteria.
     *
     * @param inputContours  is the input list of contours
     * @param output         is the the output list of contours
     * @param minArea        is the minimum area of a contour that will be kept
     * @param minPerimeter   is the minimum perimeter of a contour that will be kept
     * @param minWidth       minimum width of a contour
     * @param maxWidth       maximum width
     * @param minHeight      minimum height
     * @param maxHeight      maximimum height
     * @param solidity       the minimum and maximum solidity of a contour
     * @param minVertexCount minimum vertex Count of the contours
     * @param maxVertexCount maximum vertex Count
     * @param minRatio       minimum ratio of width to height
     * @param maxRatio       maximum ratio of width to height
     */
    private void filterContours(List<MatOfPoint> inputContours, double minArea, double minPerimeter, double minWidth,
        double maxWidth, double minHeight, double maxHeight, double[] solidity, double maxVertexCount,
        double minVertexCount, double minRatio, double maxRatio, List<MatOfPoint> output)
    {
        final MatOfInt hull = new MatOfInt();
        output.clear();
        //operation
        for (int i = 0; i < inputContours.size(); i++)
        {
            final MatOfPoint contour = inputContours.get(i);
            final Rect bb = Imgproc.boundingRect(contour);
            if (bb.width < minWidth || bb.width > maxWidth)
                continue;
            if (bb.height < minHeight || bb.height > maxHeight)
                continue;
            final double area = Imgproc.contourArea(contour);
            if (area < minArea)
                continue;
            if (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true) < minPerimeter)
                continue;
            Imgproc.convexHull(contour, hull);
            MatOfPoint mopHull = new MatOfPoint();
            mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
            for (int j = 0; j < hull.size().height; j++)
            {
                int index = (int) hull.get(j, 0)[0];
                double[] point = new double[] { contour.get(index, 0)[0], contour.get(index, 0)[1] };
                mopHull.put(j, 0, point);
            }
            final double solid = 100 * area / Imgproc.contourArea(mopHull);
            if (solid < solidity[0] || solid > solidity[1])
                continue;
            if (contour.rows() < minVertexCount || contour.rows() > maxVertexCount)
                continue;
            final double ratio = bb.width / (double) bb.height;
            if (ratio < minRatio || ratio > maxRatio)
                continue;
            output.add(contour);
        }
    }

    /**
     * Compute the convex hulls of contours.
     *
     * @param inputContours  The contours on which to perform the operation.
     * @param outputContours The contours where the output will be stored.
     */
    private void convexHulls(List<MatOfPoint> inputContours, ArrayList<MatOfPoint> outputContours)
    {
        final MatOfInt hull = new MatOfInt();
        outputContours.clear();
        for (int i = 0; i < inputContours.size(); i++)
        {
            final MatOfPoint contour = inputContours.get(i);
            final MatOfPoint mopHull = new MatOfPoint();
            Imgproc.convexHull(contour, hull);
            mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
            for (int j = 0; j < hull.size().height; j++)
            {
                int index = (int) hull.get(j, 0)[0];
                double[] point = new double[] { contour.get(index, 0)[0], contour.get(index, 0)[1] };
                mopHull.put(j, 0, point);
            }
            outputContours.add(mopHull);
        }
    }

    /**
     * Checks if the list of targets is valid.
     *
     * @param targets The targets to validate.
     * @return True if valid, false otherwise.
     */
    private boolean isValid(List<VisionTarget> targets)
    {
        // Invalid if no targets or odd number of targets.
        if (targets.size() == 0 || targets.size() % 2 == 1)
        {
            return false;
        }
        // Verify that there are alternating pairs of targets. (Left right left right)
        for (int i = 0; i < targets.size(); i += 2)
        {
            VisionTarget left = targets.get(i);
            VisionTarget right = targets.get(i + 1);
            if (!left.isLeftTarget || right.isLeftTarget)
            {
                return false;
            }
        }
        return true;
    }

    private double getCorrectedAngle(RotatedRect calculatedRect)
    {
        if (calculatedRect.size.width < calculatedRect.size.height)
        {
            return calculatedRect.angle + 180;
        }
        else
        {
            return calculatedRect.angle + 90;
        }
    }

    private List<TargetData> detectTargets(List<VisionTarget> targets)
    {
        List<TargetData> targetDatas = new ArrayList<>(); // Yes I know datas isn't a word.
        // Pair vision targets and get the enclosing bounding box.
        for (int i = 0; i < targets.size(); i += 2)
        {
            VisionTarget left = targets.get(i);
            VisionTarget right = targets.get(i + 1);
            int leftBound = left.x - left.w / 2;
            int rightBound = right.x + right.w / 2;
            int topBound = Math.max(left.y + left.h / 2, right.y + right.h / 2);
            int bottomBound = Math.min(left.y - left.h / 2, right.y - right.h / 2);
            TargetData data = new TargetData((leftBound + rightBound) / 2, (topBound + bottomBound) / 2,
                rightBound - leftBound, Math.abs(bottomBound - topBound));
            data.leftTarget = left;
            data.rightTarget = right;
            targetDatas.add(data);
        }
        return targetDatas;
    }

    private boolean isValidRect(RotatedRect rect)
    {
        double ratio = Math.min(rect.size.width, rect.size.height) / Math.max(rect.size.width, rect.size.height);
        return ROTATED_RECT_RATIO_MIN <= ratio && ratio <= ROTATED_RECT_RATIO_MAX;
    }

    private VisionTarget mapContourToVisionTarget(MatOfPoint m)
    {
        MatOfPoint2f contour = new MatOfPoint2f();
        m.convertTo(contour, CvType.CV_32F);
        RotatedRect rect = Imgproc.minAreaRect(contour);

        if (!isValidRect(rect))
        {
            return null;
        }

        VisionTarget target = new VisionTarget();
        target.rotatedRect = rect;
        target.isLeftTarget = getCorrectedAngle(rect) <= 90;
        Rect bounds = rect.boundingRect();
        target.contour = contour;
        target.x = bounds.x + bounds.width / 2;
        target.y = bounds.y + bounds.height / 2;
        target.w = bounds.width;
        target.h = bounds.height;
        return target;
    }

    private int compareVisionTargets(VisionTarget target1, VisionTarget target2)
    {
        return Integer.compare(target1.x, target2.x);
    }

    /**
     * Represents a single retroflective tape.
     */
    class VisionTarget
    {
        public boolean isLeftTarget;
        public int x, y, w, h;
        public MatOfPoint2f contour;
        public RotatedRect rotatedRect;

        public boolean equals(Object o)
        {
            if (!(o instanceof VisionTarget))
                return false;
            VisionTarget target = (VisionTarget) o;
            return target.isLeftTarget == isLeftTarget && target.x == x && target.y == y && target.w == w
                && target.h == h;
        }
    }
}

