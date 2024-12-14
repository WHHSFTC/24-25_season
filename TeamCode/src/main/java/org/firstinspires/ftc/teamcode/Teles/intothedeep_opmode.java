package org.firstinspires.ftc.teamcode.Teles;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
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
public class intothedeep_opmode extends OpMode{

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
    ColorSensor intakeColor;
    DistanceSensor leftDS;
    DistanceSensor rightDS;

    Servo alpha;
    Servo beta;
    Servo intakeClaw;
    Servo intakeWrist;
    Servo outputClaw;
    Servo deltaRight;
    Servo deltaLeft;
    Servo outputWrist;

    ElapsedTime stateTimer = new ElapsedTime();
    ElapsedTime slidesTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    //intake constants
    public static double intakeClawOpenPos = 0.10;
    public static double intakeClawClosedPos = 0.47;
    public static double alphaTransferPos = 0.44;
    public static double betaTransferPos = 0.44;
    public static double alphaIntakePos = 0.86;
    public static double betaIntakePos = 0.86;
    public static double alphaLowerPos = 0.92;
    public static double betaLowerPos = 0.92;
    public static double intakeWristStraightPos = 0.52;
    public static double intakeWristRightLimit = 0.73;
    public static double intakeWristLeftLimit = 0.29;
    public static double intakeWristPos;

    //slides constants
    public static double slideMin = 0.0;
    public static double slideMaxEx = 450.0;
    public static double slideMaxVe = 1100.0;
    public static double slideSpecimenVe = 530.0;
    public static double slideHang1Ve;
    public static double slideHang2Ve;
    double slidePositionTargetEx;
    double slidePositionTargetVe;
    double verticalPower;
    SlidesPID slidesPidExtendo;
    SlidesPID slidesPidVertical;

    //output constants
    public static double deltaRightPreTransfer = 0.59;
    public static double deltaLeftPreTransfer = 0.03;
    public static double deltaRightTransferPos = 0.61;
    public static double deltaLeftTransferPos = 0.02;
    public static double deltaRightSamplePos = 0.45;
    public static double deltaLeftSamplePos = 0.18;
    public static double deltaRightSpecimenPos = 0.23;
    public static double deltaLeftSpecimenPos = 0.0;
    public static double outputWristStraightPos = 0.90;
    public static double outputWristSwitchPos = 0.23;
    public static double outputWristSpecimenPos = 0.30;
    public static double outputClawClosedPos = 0.48;
    public static double outputClawOpenPos = 0.76;

    //times
    final double INTAKELOWER_TIME = 0.05;
    final double INTAKECLAW_TIME = 0.40;
    final double OUTPUTCLAW_TIME = 0.35;
    final double OUTPUTARM_READY = 0.20;
    final double HANG1_UP_TIME = 0.70;
    final double HANG2_UP_TIME = 1.00;

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
    double tuningPos1 = 0.5;
    double tuningPos2 = 0.5;

    @Override
    public void init(){

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
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
        outputClaw = hardwareMap.get(Servo.class, "outputClaw");
        deltaRight = hardwareMap.get(Servo.class, "deltaRight");
        deltaLeft = hardwareMap.get(Servo.class, "deltaLeft");
        outputWrist = hardwareMap.get(Servo.class, "outputWrist");

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
        /*leftDS = hardwareMap.get(DistanceSensor.class, "leftDS");
        rightDS = hardwareMap.get(DistanceSensor.class, "rightDS");*/

        deltaLeft.setDirection(Servo.Direction.REVERSE);
        deltaLeft.setDirection(Servo.Direction.REVERSE);
        outputClaw.setDirection(Servo.Direction.REVERSE);

        slidePositionTargetEx = 0.0;
        slidePositionTargetVe = 0.0;

        outputClaw.setPosition(outputClawClosedPos);
    }

    //@Override
    public void start(){
        extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rs.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ls.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ms.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        deltaLeft.setPosition(0.73);
        deltaRight.setPosition(0.97);
        outputClaw.setPosition(outputClawClosedPos);
        outputWrist.setPosition(1.0);
        intakeClaw.setPosition(0.63);
        intakeWrist.setPosition(0.50); //fully straight on
        alpha.setPosition(0.76);
        beta.setPosition(0.24);
    }

    @Override
    final public void loop(){
        timeGap = slidesTimer.milliseconds();
        slidesTimer.reset();

        loopCounter++;
        loopCumulativeTime += timeGap;

        if (loopCumulativeTime >= 1000) {
            telemetry.addData("Time per Loop", "Time per Loop: " + loopCumulativeTime/loopCounter);
            loopCounter = 0.0;
            loopCumulativeTime = 0.0;
        }

        //extendo digital
        extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendo.setPower(2.1*slidesPidExtendo.calculatePowerExtendo(slidePositionTargetEx));
        slidesPidExtendo.updateEx(extendo.getCurrentPosition(), timeGap);

        if (slidePositionTargetEx < slideMin) {
            slidePositionTargetEx = slideMin;
        }
        if (slidePositionTargetEx > slideMaxEx) {
            slidePositionTargetEx = slideMaxEx;
        }

        //vertical digital
        rs.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ls.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ms.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalPower = 1.9*slidesPidVertical.calculatePowerVertical(slidePositionTargetVe);
        ls.setPower(verticalPower);
        ms.setPower(verticalPower);
        rs.setPower(verticalPower);
        slidesPidVertical.updateVe(((double)ms.getCurrentPosition())/3, timeGap);

        if (slidePositionTargetVe < slideMin) {
            slidePositionTargetVe = slideMin;
        }
        if (slidePositionTargetVe > slideMaxVe) {
            slidePositionTargetVe = slideMaxVe;
        }

        //drivetrain
        double y = -gamepad1.left_stick_x; //verticals
        double x = -gamepad1.left_stick_y; //horizontal
        double r = -gamepad1.right_stick_x; //pivot and rotation
        double scalar = 0.5;

        if(gamepad1.left_bumper){
            scalar = 0.3;
        }

        double preRF = r*scalar + y*scalar + x*scalar;
        double preLF = r*scalar + y*scalar - x*scalar;
        double preRB = r*scalar -y*scalar + x*scalar;
        double preLB = r*scalar -y*scalar -x*scalar;

        double max = Math.max(Math.max(Math.max(Math.max(preRF, preRB), preLB), preLF), 1);

        rf.setPower(preRF/max);
        lf.setPower(preLF/max);
        rb.setPower(preRB/max);
        lb.setPower(preLB/max);

        gamepad1prev.copy(gamepad1);
        gamepad2prev.copy(gamepad2);

        telemetry.addData("carabiner pressed", "carabiner pressed: " + carabinerPressed);
        telemetry.addData("intakeWrist position", "intake wrist position: " + intakeWrist.getPosition());

        telemetry.addData("alpha servo position", "alpha servo posotion: " + alpha.getPosition());
        telemetry.addData("beta servo position", "beta servo position: " + beta.getPosition());

        telemetry.addData("intakeClawPos", "intake claw position: " + intakeClaw.getPosition());
        telemetry.addData("outputWristPos", "output wrist position: " + outputWrist.getPosition());
        telemetry.addData("deltaLeft", "deltaLeft position: " + deltaLeft.getPosition());
        telemetry.addData("deltaRight", "deltaRight position: " + deltaRight.getPosition());

        telemetry.addData("ms position", "ms position: " + ms.getCurrentPosition());
        telemetry.addData("rs power", "rs power: " + rs.getPower());
        telemetry.addData("ls power", "ls power: " + ls.getPower());
        telemetry.addData("ms power", "ms power: " + ms.getPower());
        telemetry.addData("Error Vertical", "Error vertical: " + (slidePositionTargetVe - (ls.getCurrentPosition() + rs.getCurrentPosition() + ms.getCurrentPosition())/3));

        telemetry.addData("Error Extendo", "Error Extendo: " + (slidePositionTargetEx - extendo.getCurrentPosition()));
        telemetry.addData("extendo position", "extendo position: " + extendo.getCurrentPosition());
        telemetry.addData("extendo power", "extendo power: " + extendo.getPower());
        telemetry.addData("extendo slides pressed", "extendo slides pressed: " + extendoPressed);

        telemetry.addData("gampead 2 left stick analog value", "gamepad 2 left stick analog value: " + gamepad2.left_stick_y);
        telemetry.addData("gampead 2 right stick analog value", "gamepad 2 right stick analog value: " + gamepad2.right_stick_y);
        telemetry.update();

        packet.put("slides target extendo", slidePositionTargetEx);
        packet.put("slides position extendo", extendo.getCurrentPosition());
        packet.put("slides power extendo", extendo.getPower());
        packet.put("extendo kp", "extendo kp" + SlidesPID.KpEx);
        packet.put("extendo kd", "extendo kd" + SlidesPID.KdEx);
        packet.put("extendo ki", "extendo ki" + SlidesPID.KiEx);

        packet.put("slides target vertical", slidePositionTargetVe);
        packet.put("slides position vertical", ((ls.getCurrentPosition()) + ms.getCurrentPosition() + rs.getCurrentPosition())/3);
        packet.put("slides power extendo", ls.getPower());
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
    }
}