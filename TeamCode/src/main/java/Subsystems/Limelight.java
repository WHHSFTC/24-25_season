package Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Limelight {
    private Limelight3A limelight;
    private double tx, ty, ta, tv; // Vision variables

    public Limelight(String llName, HardwareMap hardwareMap, int blueDetector, int redDetector, int blueSnap, int redSnap) {
        limelight = hardwareMap.get(Limelight3A.class, llName);
    }

    public void llPipeLineSwitch(int pipeline) {
        limelight.pipelineSwitch(pipeline);
    }

    public void start() {
        limelight.start();
    }

    public double[] getLLResults() {
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            tv = result.getTyNC();
            if (tv != 0) tv = 1;
            else tv = 0;
            tx = result.getTx(); // Horizontal offset
            ty = result.getTy(); // Vertical offset
            ta = result.getTa(); // Target area
        } else { //default to 0 to prevent unwanted movement
            tv = 0.0;
            tx = 0.0;
            ty = 0.0;
            ta = 0.0;
        }
        return new double[]{tv, tx, ty, ta};
    }
}
