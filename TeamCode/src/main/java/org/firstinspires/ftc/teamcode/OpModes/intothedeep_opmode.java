package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import java.util.List;

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

    DcMotor rf;
    DcMotor lf;
    DcMotor rb;
    DcMotor lb;
    DcMotor extendo;
    DcMotor ls;
    DcMotor rs;
    DcMotor ms;

    TouchSensor extendoSlidesLimit;
    TouchSensor verticalSlidesLimit;
    TouchSensor carabinerLimit;
    TouchSensor transferLimit;
    RevColorSensorV3 intakeColor;
    DistanceSensor chamberDS;
    DistanceSensor outputDS;

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
    public static double intakeClawOpenPos = 0.10;
    public static double intakeClawClosedPos = 0.45;
    public static double alphaTransferPos = 0.44;
    public static double betaTransferPos = 0.44;
    public static double alphaIntakePos = 0.81;
    public static double betaIntakePos = 0.81;
    public static double alphaLowerPos = 0.92;
    public static double betaLowerPos = 0.92;
    public static double intakeWristStraightPos = 0.52;
    public static double intakeWristRightLimit = 0.73;
    public static double intakeWristLeftLimit = 0.29;
    public static double intakeWristPos;

    //slides constants
    public static double slideMin = 0.0;
    public static double slideMaxEx = 450.0;
    public static double slideMaxVe = 1600.0;
    public static double slideSpecimenVe = 500.0;
    public static double slideHangVe = 900.0;
    public static double swingSizeEx = 120.0;
    public static double slidePositionTargetEx;
    public static double slidePositionTargetVe;
    double verticalPower;
    SlidesPID slidesPidExtendo;
    SlidesPID slidesPidVertical;

    //output constants
    public static double deltaRightPreTransfer = 0.59;
    public static double deltaLeftPreTransfer = 0.22;
    public static double deltaRightTransferPos = 0.66;
    public static double deltaLeftTransferPos = 0.15;
    public static double deltaRightSamplePos = 0.44;
    public static double deltaLeftSamplePos = 0.37;
    public static double deltaRightSpecimenPos = 0.28;
    public static double deltaLeftSpecimenPos = 0.27;
    public static double outputWristStraightPos = 0.90;
    public static double outputWristSwitchPos = 0.23;
    public static double outputWristSpecimenPos = 0.15;
    public static double outputClawClosedPos = 0.48;
    public static double outputClawOpenPos = 0.76;

    //hang constants
    public static double springToggleOnPos = 0.58;
    public static double springToggleOffPos = 0.94;

    //times
    final double INTAKELOWER_TIME = 0.10;
    final double INTAKECLAW_TIME = 0.35;
    final double OUTPUTCLAW_TIME = 0.20;
    final double OUTPUTARM_READY = 0.10;
    final double HANG1_UP_TIME = 2.50;
    final double HANG2_UP_TIME = 2.50;
    final double CARABINER_TIME = 1.00;

    //loop constants
    double timeGap;
    double loopCumulativeTime = 0.0;
    double loopCounter = 0.0;

    boolean turtle;
    boolean carabinerPressed;
    boolean transferPressed = false;
    boolean extendoPressed;
    boolean verticalPressed;
    boolean sampleOutputState;
    boolean specimenOutputState;
    boolean ozoneOutputState;
    double maxChamberDist = 150;
    double maxBucketDist = 100;
    double minSlideSafeDist = 400;
    public static double exConstantPID = 0.0;
    public static double veConstantPID = 0.0;
    boolean exAddPower = false;
    boolean veAddPowerDown = false;
    boolean veAddPowerUp = false;
    boolean veHangPower = false;
    boolean outputWristSpecimen = false;
    double tuningPos1 = 0.5;
    double tuningPos2 = 0.5;
    double colorThreshold = 300;
    boolean blueAlliance = true;

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
        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ls.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ms.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rs.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        extendoSlidesLimit = hardwareMap.get(TouchSensor.class, "extendoSlidesLimit");
        verticalSlidesLimit = hardwareMap.get(TouchSensor.class, "verticalSlidesLimit");
        carabinerLimit = hardwareMap.get(TouchSensor.class, "carabinerLimit");
        transferLimit = hardwareMap.get(TouchSensor.class, "transferLimit");
        intakeColor = hardwareMap.get(RevColorSensorV3.class, "intakeColor");
        outputDS = hardwareMap.get(DistanceSensor.class, "outputDS");
        chamberDS = hardwareMap.get(DistanceSensor.class, "chamberDS");

        outputClaw.setDirection(Servo.Direction.REVERSE);
        beta.setDirection(Servo.Direction.REVERSE);

        slidePositionTargetEx = 0.0;
        slidePositionTargetVe = 0.0;

        turtle = false;
        carabinerPressed = false;

        specimenOutputState = false;
        ozoneOutputState = false;
        sampleOutputState = false;

        for(LynxModule hub : bothHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void start(){
        slidePositionTargetEx = 0.0;
        slidePositionTargetVe = 0.0;
        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ls.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ms.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rs.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        verticalSlidesLimit.isPressed();
        carabinerLimit.isPressed();
        transferLimit.isPressed();

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
        extendo.setPower(2.4*slidesPidExtendo.calculatePowerExtendo(slidePositionTargetEx) + exConstantPID);
        slidesPidExtendo.updateEx(extendo.getCurrentPosition(), timeGap);

        rs.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ls.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ms.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalPower = 2.0*slidesPidVertical.calculatePowerVertical(slidePositionTargetVe) + veConstantPID;
        ls.setPower(verticalPower);
        ms.setPower(verticalPower);
        rs.setPower(verticalPower);
        slidesPidVertical.updateVe((ms.getCurrentPosition()), timeGap);

        if(transferLimit.isPressed()){
            transferPressed = true;
        }else{
            transferPressed = false;
        }

        if(carabinerLimit.isPressed()){
            carabinerPressed = true;
        }
        else{
            carabinerPressed = false;
        }

        if(extendoSlidesLimit.isPressed()){
            extendoPressed = true;
            extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        else{
            extendoPressed = false;
        }

        if(verticalSlidesLimit.isPressed()){
            verticalPressed = true;
            ms.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ls.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rs.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        else{
            verticalPressed = false;
        }

        childLoop();

        /*telemetry.addData("spring toggle", "spring toggle: " + springToggle.getPosition());*/
        //telemetry.addData("carabiner sensor", "carabiner: " + carabinerPressed);
        //telemetry.addData("transferPressed", "transfer pressed: " + transferPressed);
        telemetry.addData("battery voltage", "battery voltage: " + voltageSensor.getVoltage());
        telemetry.addData("red", intakeColor.red());
        telemetry.addData("blue", intakeColor.blue());
        telemetry.addData("green", intakeColor.green());
        telemetry.addData("intakeWrist position", "intake wrist position: " + intakeWrist.getPosition());

        telemetry.addData("alpha servo position", "alpha servo posotion: " + alpha.getPosition());
        telemetry.addData("beta servo position", "beta servo position: " + beta.getPosition());

        telemetry.addData("intakeClawPos", "intake claw position: " + intakeClaw.getPosition());
        telemetry.addData("outputWristPos", "output wrist position: " + outputWrist.getPosition());
        telemetry.addData("outputClawPos", "out claw position: " + outputClaw.getPosition());
        telemetry.addData("deltaLeft", "deltaLeft position: " + deltaLeft.getPosition());
        telemetry.addData("deltaRight", "deltaRight position: " + deltaRight.getPosition());

        telemetry.addData("ms position", "ms position: " + ms.getCurrentPosition());
        telemetry.addData("rs power", "rs power: " + rs.getPower());
        telemetry.addData("ls power", "ls power: " + ls.getPower());
        telemetry.addData("ms power", "ms power: " + ms.getPower());
        telemetry.addData("Error Vertical", "Error vertical: " + (slidePositionTargetVe - (ms.getCurrentPosition())));

        telemetry.addData("Error Extendo", "Error Extendo: " + (slidePositionTargetEx - extendo.getCurrentPosition()));
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
        dashboard.sendTelemetryPacket(packet);
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