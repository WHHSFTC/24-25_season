package OpModes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.ServoHubConfiguration;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import java.lang.reflect.Array;
import java.util.List;
import java.util.Timer;

//@Disabled
@Config
@TeleOp
abstract public class intothedeep_opmode extends OpMode{
    FtcDashboard dashboard;
    static TelemetryPacket packet;

    Gamepad gamepad1prev = new Gamepad();
    Gamepad gamepad2prev = new Gamepad();
    List<LynxModule> bothHubs;
    VoltageSensor voltageSensor;

    private AnalogInput ultrasonicSensorChamber;
    double chamberVoltage = 0.0;
    double chamberCM = 0.0;
    double chamberMM = 0.0;

    DcMotor rf;
    DcMotor lf;
    DcMotor rb;
    DcMotor lb;
    DcMotor extendo;
    DcMotor ls;
    DcMotor rs;
    DcMotor ms;

    TouchSensor extendoSlidesLimit;
    TouchSensor carabinerLimit;

    RevColorSensorV3 intakeColor;

    Servo alpha;
    Servo beta;
    Servo intakeClaw;
    Servo intakeWrist;
    ServoImplEx outputClaw;
    Servo deltaRight;
    Servo deltaLeft;
    Servo outputWrist;
    Servo springToggle;
    ElapsedTime stateTimer = new ElapsedTime();
    ElapsedTime slidesTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    //intake constants
    public static double intakeClawOpenPos = 0.11;//used to be 0.24
    public static double intakeClawOpenPosTele = 0.19;
    public static double intakeClawBarelyClosedPos = 0.36;
    public static double intakeClawClosedPos = 0.39;
    public static double alphaTransferPos = 0.57; //0.53
    public static double betaTransferPos = 0.57; //0.53
    public static double alphaIntakePos = 0.84;
    public static double betaIntakePos = 0.84;
    public static double alphaLowerPos = 0.93; // 0.94
    public static double betaLowerPos = 0.93; // 0.94 //0.9075
    public static double intakeWristStraightPos = 0.23;
    public static double intakeWristRightLimit = 0.0;
    public static double intakeWristLeftLimit = 0.46;
    public static double intakeWristPos;

    //slides constants
    public static double slideMinEx = 0.0;
    public static double slideMinVe = -5.0;
    public static double slideMaxEx = 420.0;
    public static double slideMaxVe = 1910.0;
    public static double slideSpecimenVe = 550.0;
    public static double slideHangVe = 1200.0;
    public static double swingSizeEx = 60.0;
    public static double slidePositionTargetEx;
    public static double slidePositionTargetVe;
    double verticalPower;
    SlidesPID slidesPidExtendo;
    SlidesPID slidesPidVertical;

    //output constants
    public static double deltaRightPreTransfer = 0.36;  //0.36
    public static double deltaLeftPreTransfer = 0.39; //0.37
    public static double deltaRightTransferPos = 0.51; //0.49
    public static double deltaLeftTransferPos = 0.25; //0.30 //0.29 //0.27
    public static double deltaRightSamplePos = 0.28; // 0.28 was askew
    public static double deltaLeftSamplePos = 0.48;
    public static double deltaRightSpecimenPos = 0.11;
    public static double deltaLeftSpecimenPos = 0.36;
    public static double outputWristStraightPos = 0.94;
    public static double outputWristSwitchPos = 0.27;
    public static double outputWristSpecimenPos = 0.15;
    public static double outputClawClosedPos = 0.32;
    public static double outputClawOpenPos = 0.55; //0.54

    //hang constants
    public static double springToggleOnPos = 0.36;
    public static double springToggleOffPos = 0.69;

    //times
    final double INTAKELOWER_TIME = 0.10;
    final double INTAKECLAW_TIME = 0.35; //0.25
    final double OUTPUTCLAW_TIME = 0.1; //0.3
    final double OUTPUTARM_READY = 0.23; //0.45

    //loop constants
    double timeGap;
    double loopCumulativeTime = 0.0;
    double loopCounter = 0.0;

    boolean turtle;
    boolean carabinerPressed;
    boolean extendoPressed;
    boolean sampleOutputState;
    boolean specimenOutputState;
    boolean ozoneOutputState;
    boolean sideIntakeState;
    public static double exConstantPID = 0.0;
    public static double veConstantPID = 0.0;
    boolean exAddPower = false;
    boolean veAddPowerDown = false;
    boolean veAddPowerUp = false;
    boolean veHangPower = false;
    boolean outputWristSpecimen = false;
    public static double tuningPos1 = 0.5;
    public static double tuningPos2 = 0.5;
    double colorThreshold = 300;
    boolean blueAlliance;

    public Limelight3A limelight;
    public static double impulsePeriod = 15;
    double impulseFactor = 1;
    public double tx, ty, ta, tv; //vision variables
    public double targetAlignThresholdX = 4.0;
    public double targetAlignThresholdY = 4.0;
    public static double LLcannyUpper = 65;
    public static double LLcannyLower = 90;
    public static double LLkPx = 0.07;
    public static double LLkPy = -0.09;
    public static double LLkIx = 0;
    public static double LLkIy = 0;
    public static double LLkDx = 0;
    public static double LLkDy = 0;
    public static double LLIpersistence = 0.95;
    double LLIx = 0;
    double LLIy = 0;
    public static double limelightPow = 0.33;
    public static double angle = 0;
    public static double angleLerp = 0.05;
    double prevErrorX = -999;
    double prevErrorY = -999;
    public static double limelightScalarX = 0.05;
    public static double limelightScalarY = -0.07;

    public static double strafeCorrection = 1.6;

    List<List<Double>> corners;
    @Override
    public void init(){

        stateTimer.reset();
        slidesTimer.reset();

        bothHubs = hardwareMap.getAll(LynxModule.class);
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        slidesPidExtendo = new SlidesPID();
        slidesPidVertical = new SlidesPID();

        rf = hardwareMap.get(DcMotor.class, "motorRF");
        lf = hardwareMap.get(DcMotor.class, "motorLF");
        rb = hardwareMap.get(DcMotor.class, "motorRB");
        lb = hardwareMap.get(DcMotor.class, "motorLB");

        extendo = hardwareMap.get(DcMotor.class, "extendo");
        ls = hardwareMap.get(DcMotor.class, "ls");
        rs = hardwareMap.get(DcMotor.class, "rs");
        ms = hardwareMap.get(DcMotor.class, "ms");

        ls.setDirection(DcMotorSimple.Direction.REVERSE);
        rs.setDirection(DcMotorSimple.Direction.REVERSE);

        rs.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ls.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ms.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        alpha = hardwareMap.get(Servo.class, "alpha");
        beta = hardwareMap.get(Servo.class, "beta");
        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
        intakeWrist = hardwareMap.get(Servo.class, "intakeWrist");
        outputClaw = hardwareMap.get(ServoImplEx.class, "outputClaw");
        deltaRight = hardwareMap.get(Servo.class, "deltaRight");
        deltaLeft = hardwareMap.get(Servo.class, "deltaLeft");
        outputWrist = hardwareMap.get(Servo.class, "outputWrist");
        springToggle = hardwareMap.get(Servo.class, "springToggle");

        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extendoSlidesLimit = hardwareMap.get(TouchSensor.class, "extendoSlidesLimit");
        carabinerLimit = hardwareMap.get(TouchSensor.class, "carabinerLimit");
        ultrasonicSensorChamber = hardwareMap.get(AnalogInput.class, "chamberSensor");
        intakeColor = hardwareMap.get(RevColorSensorV3.class, "colorsensor");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");


        limelight.start();
        outputClaw.setDirection(Servo.Direction.REVERSE);
        beta.setDirection(Servo.Direction.REVERSE);

        /*ls.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rs.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ms.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/

        turtle = false;
        carabinerPressed = false;

        specimenOutputState = false;
        ozoneOutputState = false;
        sampleOutputState = false;

        // Bahand Debugging Output 5/20
        telemetry.addData("deltaLeft", "deltaLeft position: " + deltaLeft.getPosition());
        telemetry.addData("deltaRight", "deltaRight position: " + deltaRight.getPosition());

        telemetry.addData("ms position", "ms position: " + ms.getCurrentPosition());
        telemetry.addData("extendo position", "extendo position: " + extendo.getCurrentPosition());

        for(LynxModule hub : bothHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void start(){
        extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ms.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rs.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ls.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



    }

    @Override
    final public void loop(){

        for (LynxModule hub : bothHubs) {
            hub.clearBulkCache();
        }

        //bulk read
        rf.getCurrentPosition();
        lf.getCurrentPosition();
        rb.getCurrentPosition();
        lb.getCurrentPosition();
        ls.getCurrentPosition();
        rs.getCurrentPosition();
        ms.getCurrentPosition();
        extendo.getCurrentPosition();
        extendoSlidesLimit.isPressed();
        carabinerLimit.isPressed();

        timeGap = slidesTimer.milliseconds();
        slidesTimer.reset();

        loopCounter++;
        loopCumulativeTime += timeGap;

        if (loopCumulativeTime >= 1000) {
            telemetry.addData("Time per Loop", "Time per Loop: " + loopCumulativeTime/loopCounter);
            loopCounter = 0.0;
            loopCumulativeTime = 0.0;
        }

        extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rs.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ls.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ms.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        carabinerPressed = carabinerLimit.isPressed();

        if(slidePositionTargetEx < -5){
            slidePositionTargetEx = -5;
        }

        if(slidePositionTargetEx > slideMaxEx){
            slidePositionTargetEx = slideMaxEx;
        }

        if(slidePositionTargetVe < -100){
            slidePositionTargetVe = -100;
        }

        if(slidePositionTargetVe > slideMaxVe){
            slidePositionTargetVe = slideMaxVe;
        }

        if(extendoSlidesLimit.isPressed()){
            extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            exAddPower = false;
            slideMinEx = 0.0;
        }

        calibrateOutput(); // calibrate output servos

        childLoop();
        /*

        telemetry.addData("spring toggle pos", "spring toggle pos: " + springToggle.getPosition());
        telemetry.addData("battery voltage", "battery voltage: " + voltageSensor.getVoltage());
        telemetry.addData("intakeWrist position", "intake wrist position: " + intakeWrist.getPosition());

        telemetry.addData("alpha servo position", "alpha servo posotion: " + alpha.getPosition());
        telemetry.addData("beta servo position", "beta servo position: " + beta.getPosition());

        telemetry.addData("intakeClawPos", "intake claw position: " + intakeClaw.getPosition());
        telemetry.addData("outputWristPos", "output wrist position: " + outputWrist.getPosition());
        telemetry.addData("outputClawPos", "out claw position: " + outputClaw.getPosition());

        telemetry.addData("deltaLeft", "deltaLeft position: " + deltaLeft.getPosition());
        telemetry.addData("deltaRight", "deltaRight position: " + deltaRight.getPosition());
        telemetry.addData("ms position", "ms position: " + ms.getCurrentPosition());

        //telemetry.addData("Error Extendo", "Error Extendo: " + (slidePositionTargetEx - extendo.getCurrentPosition()));
        telemetry.addData("extendo position", "extendo position: " + extendo.getCurrentPosition());
        telemetry.addData("extendo power", "extendo power: " + extendo.getPower());

        telemetry.addData("time per loop", timeGap);
        telemetry.update();

        packet.put("slides target extendo", slidePositionTargetEx);
        packet.put("slides position extendo", extendo.getCurrentPosition());
        packet.put("slides power extendo", extendo.getPower());
        packet.put("extendo kp", "extendo kp" + SlidesPID.KpEx);
        packet.put("extendo kd", "extendo kd" + SlidesPID.KdEx);
        packet.put("extendo ki", "extendo ki" + SlidesPID.KiEx);

        packet.put("slides target vertical", slidePositionTargetVe);
        packet.put("slides position vertical", (ms.getCurrentPosition()));
        packet.put("slides power extendo", ms.getPower());
        packet.put("vertical kp", "extendo kp" + SlidesPID.KpVe);
        packet.put("vertical kd", "extendo kd" + SlidesPID.KdVe);
        packet.put("vertical ki", "extendo ki" + SlidesPID.KiVe);
        packet.put("tx", tx);
        packet.put("ty", ty);
        dashboard.sendTelemetryPacket(packet);

         */
    }

    public void calibrateOutput() {
        double leftPos = deltaLeft.getPosition();
        double rightPos = deltaRight.getPosition();
        boolean swapped = true;
        if (gamepad1.dpad_up) {
            leftPos += .001;
            rightPos -= .001;
        }
        if (gamepad1.dpad_down) {
            leftPos -= .001;
            rightPos += .001;
        }
        if (gamepad1.dpad_left) {
            leftPos += .001;
            rightPos += .001;
        }
        if (gamepad1.dpad_right) {
            leftPos -= .001;
            rightPos -= .001;
        }
        if (swapped) {
            deltaLeft.setPosition(rightPos);
            deltaRight.setPosition(leftPos);
        } else if (!swapped) {
            deltaLeft.setPosition(leftPos);
            deltaRight.setPosition(rightPos);
        }

        if (gamepad1.left_bumper) {
            deltaLeftTransferPos = deltaLeft.getPosition();
            deltaRightTransferPos = deltaRight.getPosition();

            deltaLeftPreTransfer = deltaLeftTransferPos + .14;
            deltaRightPreTransfer = deltaRightTransferPos - .15;

            deltaLeftSamplePos = deltaLeftTransferPos + .23;
            deltaRightSamplePos = deltaRightTransferPos - .23;

            deltaLeftSpecimenPos = deltaLeftTransferPos + .11;
            deltaRightSpecimenPos = deltaRightTransferPos - .40;
        }
    }

    public void setWristPosition() {
        if(corners != null) {
            limelight.pipelineSwitch(4);

            int expansion = 5;
            //x, y, w, h, cl, cu
            double[] inputs = {Math.max(0, corners.get(0).get(0) - expansion),
                    Math.max(0, corners.get(0).get(1) - expansion),
                    Math.min(corners.get(1).get(0) - corners.get(0).get(0) + 2 * expansion, 960-Math.max(0, corners.get(0).get(0) - expansion)),
                    Math.min(corners.get(3).get(1) - corners.get(0).get(1) + 2 * expansion, 720-Math.max(0, corners.get(0).get(1) - expansion)),
                    LLcannyLower, LLcannyUpper}; // canny threshold bounds
            limelight.updatePythonInputs(inputs);
            LLResult pyResult = limelight.getLatestResult();

            double[] pythonOutputs = pyResult.getPythonOutput();
            if (pythonOutputs != null && pythonOutputs.length > 0) {
                angle = angle * angleLerp + pythonOutputs[0] * (1 - angleLerp);
                intakeWrist.setPosition(0.06202 * Math.tan(-0.01456 * angle + -499.47093) + 0.22691); // experimentally determined formula
            }
        } else {
            getLimelightPower();
        }
    }

    public boolean getLimelightPower() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            tv = result.getTyNC();
            if (tv != 0) tv = 1;
            else tv = 0;

            tx = result.getTx(); // Horizontal offset
            ty = result.getTy(); // Vertical offset
            ta = result.getTa(); // Target area

            List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
            if (detections != null) {
                for (LLResultTypes.DetectorResult detection : detections) {
                    String className = detection.getClassName();

                    corners = detection.getTargetCorners();

                    double width = corners.get(1).get(0) - corners.get(0).get(0);
                    double height = corners.get(0).get(1) - corners.get(3).get(1);
                }
            }
        } else { //default to 0 to prevent unwanted movement
            telemetry.addData("Limelight", "No Targets");
            limelight.pipelineSwitch(blueAlliance ? 0 : 9);
            tx = 0;
            ty = 0;
            ta = 0;
            tv = 0;
        }

        telemetry.addData("TX", tx);
        telemetry.addData("TY", ty);
        telemetry.addData("TA", ta);
        telemetry.addData("TV", tv);

        if (Math.abs(tx) < targetAlignThresholdX && Math.abs(ty) < targetAlignThresholdY && (tx != 0 || ty != 0)) {
            prevErrorX = -999;
            return true;
        }

        telemetry.addData("x integral", LLIx);
        telemetry.addData("y integral", LLIy);

        double errorY = Math.pow(Math.abs(tx), limelightPow) * Math.signum(tx);
        LLIy = LLIpersistence * LLIy + errorY;
        double y = errorY * LLkPy + (errorY - prevErrorY) * (prevErrorX == -999 ? 0 : LLkDy) + LLIy * LLkIy; //verticals

        double errorX = Math.pow(Math.abs(ty), limelightPow) * Math.signum(ty);
        LLIx = LLIpersistence * LLIx + errorX;
        double x = errorX * LLkPx + (errorX - prevErrorX) * (prevErrorX == -999 ? 0 : LLkDx) + LLIx * LLkIx; //horizontals

        double preRF = strafeCorrection * (y + x);
        double preLF = strafeCorrection * (y - x);
        double preRB = -y + x;
        double preLB = -y - x;

        double max = Math.max(Math.max(Math.max(Math.max(preRF, preRB), preLB), preLF), 1);
        impulseFactor = (impulseFactor + 1/impulsePeriod) % 1;

        rf.setPower(impulseFactor * preRF / max);
        lf.setPower(impulseFactor * preLF / max);
        rb.setPower(impulseFactor * preRB / max);
        lb.setPower(impulseFactor * preLB / max);

        prevErrorX = errorX;
        prevErrorY = errorY;
        return false;
    }

    @Override
    public void stop(){
        rf.setPower(0);
        lf.setPower(0);
        rb.setPower(0);
        lb.setPower(0);
        extendo.setPower(0);
        ms.setPower(0);
        rs.setPower(0);
        ls.setPower(0);
    }

    public void childLoop(){

    }
}