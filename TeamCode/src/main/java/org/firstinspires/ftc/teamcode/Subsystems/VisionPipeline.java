package org.firstinspires.ftc.teamcode.Subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Size;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.*;

public class VisionPipeline extends OpenCvPipeline {

    private Mat topDownHSV = new Mat();
    private Mat rightSideUp = new Mat();
    private Mat rightSideUpHSV = new Mat();
    private Mat transformedFrame = new Mat();
    private Mat topDown = new Mat();
    private Mat threshold = new Mat();
    private Mat outputVisibleFrame = new Mat();
    private Mat distTransform = new Mat();
    private Mat distanceTransformThreshold = new Mat();

    ArrayList<MatOfPoint> contourPoints = new ArrayList<>();

    public int targetIndex;

    public VisionPipeline(int colorIndex) {
        targetIndex = colorIndex;
    }

    // Output information
    public double desiredSampleX = -1;
    public double desiredSampleY = -1;
    public double desiredSampleTheta = -1;

    // To tell when the camera frame changes
    public boolean parity = false;

    // Color bounds
    public Scalar lowerRed = new Scalar(146, 38, 169);
    public Scalar upperRed = new Scalar(255, 255, 255);
    public Scalar lowerYellow = new Scalar(17, 10, 191);
    public Scalar upperYellow = new Scalar(101, 255, 255);
    public Scalar lowerBlue = new Scalar(112, 84, 181);
    public Scalar upperBlue = new Scalar(133, 255, 255);

    // Nonlinear mapping constants
    public double A = 0.00017, B = -6.2, C = -1.9, D = 1.6, E = 0.5;

    // Ideal sample position
    public double targetX = 320;
    public double targetY = 320;

    // Edge detection parameters/initializations
    private Mat edges = new Mat();
    private Mat blurredThreshold = new Mat();
    private Mat lines = new Mat();

    public int minPointsForLine = 15, minLineLength = 18, maxGapForLine = 14, blurKernel = 17;
    public double maxSquaredLineDistance = 9000, minAcceptableLineAngleCosine = 0.97, widthDifferenceTolerance = 3.5, sampleWidth = 33, cannyParam1 = 50, cannyParam2 = 150;

    private final Telemetry telemetry;
    public SampleEdgeDetectionPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    private double macLaurinTan(double theta) {
        // MacLaurin series for tan(theta)
        return theta + Math.pow(theta, 3) / 3 + Math.pow(theta, 5) * 2/15 + Math.pow(theta, 7) * 17/315;
    }

    private Mat nonlinearlyStretch(Mat skewedFrame, int height, int width) {
        // Create an empty result matrix with double the height of the input
        Mat transformedFrame = new Mat(height * 2, width, CvType.CV_8UC1, new Scalar(0));

        int old_y = 0;

        for (int y = 0; y < height; y++) {
            // Normalize the y-coordinate
            double normalized_y = 1 - (double) y / height;

            // Apply the nonlinear formula to find the new y-coordinate
            double new_y = A * macLaurinTan(B * normalized_y + C) + D;

            // Scale new_y to image height (accounting for doubled height)
            new_y *= height;

            // Clamp new_y to be within valid row bounds
            new_y = Math.max(0, Math.min(new_y, height * 2 - 1));  // Make sure it doesn't exceed twice the height

            // Only process pixels from the thresholded image (skewedFrame)
            for (int x = 0; x < width; x++) {
                // Check if the pixel is part of the thresholded region (value = 255)
                if (skewedFrame.get(y, x)[0] == 255) {
                    // Stretch vertically, from old_y to new_y
                    for (int row_y = (int) new_y; row_y > (int) old_y; row_y--) {
                        double proportionToContract = E * 0.5 * row_y / height;
                        int newX = (int) (width * 0.5 * proportionToContract) + (int) (x * (1 - proportionToContract));
                        transformedFrame.put(row_y, newX, 255);  // Place the transformed pixel
                    }
                }
            }

            // Update old_y for the next iteration
            old_y = (int) new_y;
        }

        return transformedFrame;
    }

    private void setThreshold(Mat initialFrame, int colorIndex) {
        // Set threshold to a binary Mat
        switch(colorIndex) {
            case 0:
                Core.inRange(initialFrame, lowerRed, upperRed, threshold);
                break;
            case 2:
                Core.inRange(initialFrame, lowerBlue, upperBlue, threshold);
                break;
            case 1:
                Core.inRange(initialFrame, lowerYellow, upperYellow, threshold);
                break;
        }
    }

    private void detectLineSegments(Mat initialThreshold) {
        Imgproc.blur(initialThreshold, blurredThreshold, new Size(blurKernel, blurKernel));
        Imgproc.Canny(blurredThreshold, edges, cannyParam1, cannyParam2);
        Imgproc.HoughLinesP(edges, lines, 1, Math.PI / 180, minPointsForLine, minLineLength, maxGapForLine);

        List<double[]> lineList = new ArrayList<>();
        for(int i = 0; i < lines.rows(); i++) {
            lineList.add(lines.get(i, 0));
        }

        // Sort lines by squared distance to the target point
        lineList.sort((line1, line2) -> {
            double distSq1 = lineDistanceSquared(line1);
            double distSq2 = lineDistanceSquared(line2);
            return Double.compare(distSq1, distSq2);
        });

        for(double[] lineA : lineList) {

            // Express the line as Ax + By + C = 0
            double A = lineA[3] - lineA[1];
            double B = lineA[0] - lineA[2];
            double C = - A * lineA[0] - B * lineA[1];

            for(double[] lineB : lineList) {
                if(parallelEnough(lineB, lineA) && Math.abs(sampleWidth - approximatePerpendicularDistance(A, B, C, lineB)) < widthDifferenceTolerance && closeEnoughAndReal(lineA, lineB)) {
                    Imgproc.line(topDown, new Point(lineA[0], lineA[1]), new Point(lineA[2], lineA[3]), new Scalar(127), 5);
                    Imgproc.line(topDown, new Point(lineB[0], lineB[1]), new Point(lineB[2], lineB[3]), new Scalar(127), 5);

                    // Set output values
                    desiredSampleTheta = Math.atan2(lineA[3] - lineA[1], lineA[2] - lineA[0]) * 180 / Math.PI + 90;
                    desiredSampleX = 0.25 * (lineA[2] + lineA[0] + lineB[2] + lineB[0]);
                    desiredSampleY = 0.25 * (lineA[3] + lineA[1] + lineB[3] + lineB[1]);

                    return;
                }
            }
        }
    }

    public double approximatePerpendicularDistance(double line1A, double line1B, double line1C, double[] line2) {
        return Math.abs(line1A * line2[0] + line1B * line2[1] + line1C)/Math.sqrt(line1A * line1A + line1B * line1B);
    }


    private double lineDistanceSquared(double[] line) {
        double x1 = line[0];
        double y1 = line[1];
        double x2 = line[2];
        double y2 = line[3];

        double midX = (x1 + x2) / 2.0;
        double midY = (y1 + y2) / 2.0;

        // Calculate squared distance from midpoint to the target point
        double dx = midX - targetX;
        double dy = midY - targetY;
        return dx * dx + dy * dy;
    }

    private boolean parallelEnough(double[] line1, double[] line2) {
        double vx1 = line1[2] - line1[0];
        double vy1 = line1[3] - line1[1];
        double vx2 = line2[2] - line2[0];
        double vy2 = line2[3] - line2[1];

        double dotProduct = vx1 * vx2 + vy1 * vy2;

        // Calculate magnitudes of the vectors
        double magnitude1 = Math.sqrt(vx1 * vx1 + vy1 * vy1);
        double magnitude2 = Math.sqrt(vx2 * vx2 + vy2 * vy2);

        if (magnitude1 == 0 || magnitude2 == 0) return false;

        return dotProduct / (magnitude1 * magnitude2) > minAcceptableLineAngleCosine;
    }

    private boolean closeEnoughAndReal(double[] line1, double[] line2) {
        double x1 = line1[0], y1 = line1[1], x2 = line1[2], y2 = line1[3];
        double px = line2[0], py = line2[1], qx = line2[2], qy = line2[3];

        // Check if any distance exceeds the maximum allowed squared distance or midpoint is not on the sample
        if (squaredDistance(x1, y1, px, py) > maxSquaredLineDistance || topDown.get((int)(0.5 * y1 + 0.5 * py), (int)(0.5 * x1 + 0.5 * px))[0] == 0) return false;
        if (squaredDistance(x2, y2, px, py) > maxSquaredLineDistance || topDown.get((int)(0.5 * y2 + 0.5 * py), (int)(0.5 * x2 + 0.5 * px))[0] == 0) return false;
        if (squaredDistance(x1, y1, qx, qy) > maxSquaredLineDistance || topDown.get((int)(0.5 * y1 + 0.5 * qy), (int)(0.5 * x1 + 0.5 * qx))[0] == 0) return false;
        if (squaredDistance(x2, y2, qx, qy) > maxSquaredLineDistance || topDown.get((int)(0.5 * y2 + 0.5 * qy), (int)(0.5 * x2 + 0.5 * qx))[0] == 0) return false;

        return true;
    }

    private double squaredDistance(double x1, double y1, double x2, double y2) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        return dx * dx + dy * dy;
    }


    @Override
    public Mat processFrame(Mat input) {
        parity = !parity;
        // Set frame right side up and define shape
        Core.rotate(input, rightSideUp, Core.ROTATE_180);
        int h = rightSideUp.height();
        int w = rightSideUp.width();

        // Convert to HSV color space and pass the frame through a color threshold
        Imgproc.cvtColor(rightSideUp, rightSideUpHSV, Imgproc.COLOR_RGB2HSV);
        setThreshold(rightSideUpHSV, targetIndex);

        // Carry out the nonlinear stretchage
        topDown = nonlinearlyStretch(threshold, h, w);

        detectLineSegments(topDown);

        // Show telemetry
        telemetry.addData("x", desiredSampleX);
        telemetry.addData("y", desiredSampleY);
        telemetry.addData("p", parity);
        telemetry.update();

        Imgproc.line(topDown, new Point(desiredSampleX + 30 * Math.cos(desiredSampleTheta * Math.PI/180.0), desiredSampleY + 30 * Math.sin(desiredSampleTheta * Math.PI/180.0)), new Point(desiredSampleX + 80 * Math.cos(desiredSampleTheta * Math.PI/180.0), desiredSampleY + 80 * Math.sin(desiredSampleTheta * Math.PI/180.0)), new Scalar(127), 2);
        Imgproc.line(topDown, new Point(desiredSampleX - 30 * Math.cos(desiredSampleTheta * Math.PI/180.0), desiredSampleY - 30 * Math.sin(desiredSampleTheta * Math.PI/180.0)), new Point(desiredSampleX - 80 * Math.cos(desiredSampleTheta * Math.PI/180.0), desiredSampleY - 80 * Math.sin(desiredSampleTheta * Math.PI/180.0)), new Scalar(127), 2);

        return topDown;
    }
}