package org.firstinspires.ftc.teamcode.Teles;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;

//@Disabled
@Config
@TeleOp
public class intothedeep_tele extends OpMode{

    FtcDashboard dashboard;
    static TelemetryPacket packet;

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
    TouchSensor carabiner;
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


    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime totalRunTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    double timePerLoop;
    double timeGap;
    double loopCumulativeTime = 0.0;
    double loopCounter = 0.0;
    double alphaPos;
    double betaPos;
    double prevAlphaPos;
    double prevBetaPos;
    public static double lerpProportion = 0.60;

    double rdsReading;
    double ldsReading;

    double verticalPower = 0.0;
    public static double slideTargetGainEx = 70.0;
    public static double slideTargetGainVe = 50.0;
    public static double slideMin = 0.0;
    public static double slideMaxEx = 450.0;
    public static double slideMaxVe = 500.0;
    public static double slideSpecimenVe = 250.0;
    public static double slidePositionTargetEx;
    public static double slidePositionTargetVe = 0.0;
    public static double variable = 0.0;


    public static double outputClawClosedPosition = 0.30;
    public static double outputClawOpenPosition = 0.50;
    public static double deltaLeftPosition = 0.73;
    public static double deltaRightPosition = 0.97;
    public static double intakeWristPos = 0.5;
    public static double intakeClawPos;

    Gamepad gamepad1prev = new Gamepad();
    Gamepad gamepad2prev = new Gamepad();

    SlidesPID slidesPidExtendo;
    SlidesPID slidesPidVertical;

    boolean extendoSlidesPressed = true;
    boolean verticalSlidesPressed = true;
    boolean carabinerPressed = false;
    boolean intakeClawClosed;
    boolean outputClawClosed;
    boolean outputWristSwitch;

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
        //verticalSlidesLimit = hardwareMap.get(TouchSensor.class, "verticalSlidesLimit");
        carabiner = hardwareMap.get(TouchSensor.class, "carabiner");
        /*leftDS = hardwareMap.get(DistanceSensor.class, "leftDS");
        rightDS = hardwareMap.get(DistanceSensor.class, "rightDS");*/

        intakeClawClosed = false;
        outputWristSwitch = false;

        deltaLeft.setDirection(Servo.Direction.REVERSE);
        deltaLeft.setDirection(Servo.Direction.REVERSE);
        outputClaw.setDirection(Servo.Direction.REVERSE);

        slidePositionTargetEx = 0.0;
        slidePositionTargetVe = 0.0;

        outputClaw.setPosition(outputClawClosedPosition);
        outputClawClosed = true;
    }

    //@Override
    public void start(){
        extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rs.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ls.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ms.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        deltaLeft.setPosition(0.73);
        deltaRight.setPosition(0.97);
        outputClaw.setPosition(outputClawClosedPosition);
        outputWrist.setPosition(1.0);
        intakeClaw.setPosition(0.63);
        intakeWrist.setPosition(0.50); //fully straight on
        alpha.setPosition(0.76);
        beta.setPosition(0.24);
    }

    @Override
    final public void loop(){
        timeGap = timer.milliseconds();
        timer.reset();

        timePerLoop = timer.milliseconds();
        timer.reset();
        loopCounter++;
        loopCumulativeTime += timePerLoop;

        if (loopCumulativeTime >= 1000) {
            telemetry.addData("Time per Loop", "Time per Loop: " + loopCumulativeTime/loopCounter);
            loopCounter = 0.0;
            loopCumulativeTime = 0.0;
        }

        //rdsReading = rightDS.getDistance(DistanceUnit.MM);
        //ldsReading = leftDS.getDistance(DistanceUnit.MM);

        //extendo (analog)


        //extendo digital
        /*if(gamepad2.dpad_up){ //extendo max
            slidePositionTargetEx = slideMaxEx;
        }*/

        extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendo.setPower(1.3*slidesPidExtendo.calculatePowerExtendo(slidePositionTargetEx));
        slidesPidExtendo.updateEx(extendo.getCurrentPosition(), timeGap);

        if(gamepad2.dpad_down){ //extendo zero
            slidePositionTargetEx = slideMin;
        }

        if (slidePositionTargetEx < slideMin) {
            slidePositionTargetEx = slideMin;
        }
        if (slidePositionTargetEx > slideMaxEx) {
            slidePositionTargetEx = slideMaxEx;
        }

        if (extendoSlidesLimit.isPressed()) { //extendo limit switch
            if (!extendoSlidesPressed) {
                extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            extendoSlidesPressed = true;
        } else {
            extendoSlidesPressed = false;
        }

        //intake wrist
        if(gamepad2.left_trigger > 0.0){
            intakeWristPos -= 0.04;
            if(intakeWristPos < 0.02){
                intakeWristPos = 0.02;
            }
            intakeWrist.setPosition(intakeWristPos);
        }

        if(gamepad2.right_trigger > 0.0){
            intakeWristPos += 0.04;
            if(intakeWristPos > 0.68){
                intakeWristPos = 0.68;
            }
            intakeWrist.setPosition(intakeWristPos);
        }

        if(!gamepad2prev.x && gamepad2.x){
            intakeWrist.setPosition(0.50);
        }

        //transfer position intake
        if(!gamepad2prev.y && gamepad2.y){
            alpha.setPosition(0.76);
            beta.setPosition(0.24);
        }

        if(carabiner.isPressed()){
            carabinerPressed = true;
        }

        rs.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ls.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ms.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (Math.abs(gamepad2.left_stick_y) > 0.01) {
            slidePositionTargetVe -= slideTargetGainVe * gamepad2.left_stick_y;
            verticalPower = 1.9*slidesPidVertical.calculatePowerVertical(slidePositionTargetVe);
            ls.setPower(verticalPower);
            ms.setPower(verticalPower);
            rs.setPower(verticalPower);
            slidesPidVertical.updateVe((ms.getCurrentPosition()), timeGap);
        }
        //vertical analog
        if(carabinerPressed){
            slidePositionTargetVe -= slideTargetGainVe * gamepad2.right_stick_y;
            rs.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ls.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ms.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            verticalPower = 1.9*slidesPidVertical.calculatePowerVertical(slidePositionTargetVe);
            ls.setPower(verticalPower);
            ms.setPower(verticalPower);
            rs.setPower(verticalPower);
            slidesPidVertical.updateVe((ms.getCurrentPosition()), timeGap);
        }
        else {
            if (Math.abs(gamepad2.right_stick_y) > 0.01) {
                if (gamepad2.right_stick_y > 0.60) {
                    ls.setPower(-1.0);
                    ms.setPower(-1.0);
                    rs.setPower(-1.0);
                    slidePositionTargetVe = slideMin;
                } else if (gamepad2.right_stick_y < -0.60) {
                    ls.setPower(1.0);
                    rs.setPower(1.0);
                    ms.setPower(1.0);
                    slidePositionTargetVe = slideMaxVe;
                }
            }
        }

        //vertical digital
        if(gamepad1.dpad_up){ //vertical slides max
            slidePositionTargetVe = slideMaxVe;
            rs.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ls.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ms.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            verticalPower = 1.9*slidesPidVertical.calculatePowerVertical(slidePositionTargetVe);
            ls.setPower(verticalPower);
            ms.setPower(verticalPower);
            rs.setPower(verticalPower);
            slidesPidVertical.updateVe((ls.getCurrentPosition() + rs.getCurrentPosition() + ms.getCurrentPosition())/3, timeGap);
        }

        if(gamepad1.dpad_down){ //zero vertical slides
            slidePositionTargetVe = slideMin;
            rs.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ls.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ms.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            verticalPower = 1.9*slidesPidVertical.calculatePowerVertical(slidePositionTargetVe);
            ls.setPower(verticalPower);
            ms.setPower(verticalPower);
            rs.setPower(verticalPower);
            slidesPidVertical.updateVe((ls.getCurrentPosition() + rs.getCurrentPosition() + ms.getCurrentPosition())/3, timeGap);
        }

        if(gamepad1.dpad_right && !gamepad1prev.dpad_right){ //specimen height
            slidePositionTargetVe = slideSpecimenVe;
            rs.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ls.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ms.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            verticalPower = 1.9*slidesPidVertical.calculatePowerVertical(slidePositionTargetVe);
            ls.setPower(verticalPower);
            ms.setPower(verticalPower);
            rs.setPower(verticalPower);
            slidesPidVertical.updateVe((ms.getCurrentPosition()), timeGap);
        }

        /*rs.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ls.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ms.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalPower = 1.9*slidesPidVertical.calculatePowerVertical(slidePositionTargetVe);
        ls.setPower(verticalPower);
        ms.setPower(verticalPower);
        rs.setPower(verticalPower);
        slidesPidVertical.updateVe((ls.getCurrentPosition() + rs.getCurrentPosition() + ms.getCurrentPosition())/3, timeGap);*/

        if (slidePositionTargetVe < slideMin) {
            slidePositionTargetVe = slideMin;
        }
        if (slidePositionTargetVe > slideMaxVe) {
            slidePositionTargetVe = slideMaxVe;
        }


        /*if (verticalSlidesLimit.isPressed()) { //vertical slides limit switch
            if (!verticalSlidesPressed) {
                ms.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                ls.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rs.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            verticalSlidesPressed = true;
        } else {
            verticalSlidesPressed = false;
        }*/

        //intake
        /*if(Math.abs(gamepad2.right_stick_y) > 0.01 || Math.abs(gamepad2.right_stick_x) > 0.01) { //intake differential
            alphaPos = ((gamepad2.right_stick_x - Math.min(gamepad2.right_stick_y, 0.9))/4.0 + 0.5);
            betaPos = ((gamepad2.right_stick_x + Math.min(gamepad2.right_stick_y, 0.9))/4.0 + 0.5);

            alpha.setPosition(lerpProportion * alphaPos + (1-lerpProportion)*prevAlphaPos);
            beta.setPosition(lerpProportion * betaPos + (1-lerpProportion)*prevBetaPos);

            prevAlphaPos = alpha.getPosition();
            prevBetaPos = beta.getPosition();
        }*/

        if(!gamepad2prev.a && gamepad2.a){  //intake claw
            if(intakeClawClosed){
                intakeClaw.setPosition(0.63);
                intakeClawClosed = false;
            }
            else{
                intakeClaw.setPosition(0.95);
                intakeClawClosed = true;
            }
        }

        //output
        if(!gamepad1prev.b && gamepad1.b){  //transfer
            deltaLeft.setPosition(0.73);
            deltaRight.setPosition(0.97);
            outputWrist.setPosition(1.0);
            outputWristSwitch = false;
            outputClaw.setPosition(outputClawOpenPosition);
            outputClawClosed = false;
        }

        if(!gamepad1prev.y && gamepad1.y){ //basket
            deltaLeft.setPosition(0.60);
            deltaRight.setPosition(0.85);
            outputWrist.setPosition(0.0);
            outputWristSwitch = true;
        }

        if(!gamepad1prev.x && gamepad1.x){  //specimen
            deltaLeft.setPosition(0.43);
            deltaRight.setPosition(0.99);
            outputWrist.setPosition(0.5);
            outputWristSwitch = false;
        }

        if(!gamepad1prev.a && gamepad1.a){  //output claw
            if(outputClawClosed){
                outputClaw.setPosition(outputClawOpenPosition);
                outputClawClosed = false;
            }
            else{
                outputClaw.setPosition(outputClawClosedPosition);
                outputClawClosed = true;
            }
        }

        if(!gamepad1prev.right_bumper && gamepad1.right_bumper){  //output wrist
            if(outputWristSwitch){
                outputWrist.setPosition(1.0); //non switch
                outputWristSwitch = false;
            }
            else{
                outputWrist.setPosition(0.0); //switch
                outputWristSwitch = true;
            }
        }

        if(!gamepad1prev.dpad_left && gamepad1.dpad_left){  //transfer between claws
            deltaLeft.setPosition(0.69);
            deltaLeft.setPosition(0.65);
            outputClaw.setPosition(outputClawOpenPosition);
            outputClawClosed = false;
        }

        /*if(gamepad1.dpad_up){
            deltaLeftPosition += 0.01;
            if(deltaLeftPosition > 1.0){
                deltaLeftPosition = 1.0;
            }
        }

        if(gamepad1.dpad_down){
            deltaLeftPosition -= 0.01;
            if(deltaLeftPosition < 0.0){
                deltaLeftPosition = 0.0;
            }
        }

        if(gamepad1.dpad_right){
            deltaRightPosition += 0.01;
            if(deltaRightPosition > 1.0){
                deltaRightPosition = 1.0;
            }
        }

        if(gamepad1.dpad_left){
            deltaRightPosition -= 0.01;
            if(deltaRightPosition < 0.0){
                deltaRightPosition = 0.0;
            }
        }

        deltaLeft.setPosition(deltaLeftPosition);
        deltaRight.setPosition(deltaRightPosition);*/

        /*if(gamepad2.dpad_right){
            intakeWristPos += 0.01;
            if(intakeWristPos > 1.0){
                intakeWristPos = 1.0;
            }
        }

        if(gamepad2.dpad_left){
            intakeWristPos -= 0.01;
            if(intakeWristPos < 0.0){
                intakeWristPos = 0.0;
            }
        }


        intakeWrist.setPosition(intakeWristPos);*/

        /*if(gamepad2.dpad_right){
            intakeClawPos += 0.01;
            if(intakeClawPos > 1){
                intakeClawPos = 1;
            }
        }

        if(gamepad2.dpad_left){
            intakeClawPos -= 0.01;
            if(intakeClawPos < 0){
                intakeClawPos = 0.0;
            }
        }

        intakeClaw.setPosition(intakeClawPos);*/

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
        telemetry.addData("extendo slides pressed", "extendo slides pressed: " + extendoSlidesPressed);

        telemetry.addData("time per loop", timePerLoop);
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