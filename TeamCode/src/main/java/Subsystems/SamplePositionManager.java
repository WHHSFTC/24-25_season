/*
 * Copyright (c) 2019 OpenFTC Team
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
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;


@TeleOp
abstract public class SamplePositionManager extends OpMode
{
    OpenCvWebcam webcam;
    VisionPipeline detectionPipeline;

    boolean previousParity = true;
    double initialX, initialY, initialHeading, nextInitialX, nextInitialY, nextInitialHeading;
    double lerpProportionToDeviate = 1.0; // lerp may be irrelevant due to variation in the point to which the estimation is relative but we shall see, I feel like it should just be 1
    double relativeSampleX, relativeSampleY, relativeSampleRotation;

    double[] strafeCorrections = new double[3];
    int inactionCycles = 0;
    double yDistanceToIntake = 100; // needs adjustment
    private HardwareMap hardwareMap;
    int cameraMonitorViewId =
            hardwareMap
                    .appContext
                    .getResources()
                    .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

    public void initialize(int colorToSeek) {

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "outputCamera"), cameraMonitorViewId);

        // Red is 0; yellow is 1; blue is 2
        detectionPipeline = new VisionPipeline(colorToSeek);
        webcam.setPipeline(detectionPipeline);

        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(352, 288, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addLine("ERROR: " + errorCode);
            }
        });

        telemetry.addLine("Initialized");
        telemetry.update();
    }

    public void closeCamera() {
        webcam.stopStreaming();
    }

    public void doSampleEstimationCycle(double odoX, double odoY, double odoHeading) {
        boolean newParity = detectionPipeline.parity;
        // if the camera captures a new frame, reset the initial pose
        if(previousParity != newParity) {
            initialX = nextInitialX;
            initialY = nextInitialY;
            initialHeading = nextInitialHeading;
            nextInitialX = odoX;
            nextInitialY = odoY;
            nextInitialHeading = odoHeading;
        }
        previousParity = newParity;

        // update sample position relative to initial robot pose (its possible that this should happen only when parity switches i.e. when the camera finishes processing its frame and the results are in)
        if(detectionPipeline.desiredSampleX != -1 && detectionPipeline.desiredSampleY != -1 && detectionPipeline.desiredSampleTheta != -1) {
            inactionCycles = 0;

            relativeSampleX = detectionPipeline.desiredSampleX;
            relativeSampleY = detectionPipeline.desiredSampleY;
            relativeSampleRotation = detectionPipeline.desiredSampleTheta;

        } else {
            inactionCycles++;
        }

        double dX = odoX - initialX;
        double dY = odoY - initialY;
        double dHeading = odoHeading - initialHeading;

        double m = Math.sqrt((dX - relativeSampleX) * (dX - relativeSampleX) + (dY - relativeSampleY) * (dY - relativeSampleY));
        double A = dHeading + Math.atan2(relativeSampleX - dX, relativeSampleY - dY);

        strafeCorrections[0] = m * Math.cos(A);
        strafeCorrections[1] = m * Math.sin(A) - yDistanceToIntake;
        strafeCorrections[2] = (relativeSampleRotation + Math.atan2(dX,dY))*-0.140056 + 0.73333;


        telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
        telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
        telemetry.addData("odo X", odoX);
        telemetry.addData("odo Y", odoY);
        telemetry.addData("odo Heading", odoHeading);
        telemetry.update();
    }

    public double[] getCorrectionValues() {
        return strafeCorrections;
    }

    public boolean trustworthyOutput() {
        return inactionCycles < 15;
    }
}