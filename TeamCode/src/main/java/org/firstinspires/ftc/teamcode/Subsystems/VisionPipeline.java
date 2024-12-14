package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.*; // TODO: make sure imports are right
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class VisionPipeline extends OpenCvPipeline {
    private int output = -1;
    private Mat threshold = new Mat();

    public VisionPipeline(boolean blue, boolean outputSide, boolean hardToSeeRightSpot) {

    }

    @Override
    public Mat processFrame(Mat input) {
        output = 0;
        return threshold;
    }
        public int getOutput() {
            return output;
        }
        public String getPipelineTelemetry() {
            return "hello";
        }
    }
