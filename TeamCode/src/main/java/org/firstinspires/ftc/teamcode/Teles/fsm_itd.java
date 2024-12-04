package org.firstinspires.ftc.teamcode.Teles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@Config
@TeleOp
public class fsm_itd extends OpMode {

    public enum TeleState {
        ARM_INTAKE,
        INTAKECLAW_CLOSE,
        BRING_IN,
        OUTPUTCLAW_CLOSE,
        INTAKECLAW_OPEN,


        OUTPUT_BRANCH,
        OUTPUT_BASKET,
        OUTPUT_SPECIMEN,
        OUTPUT_OZONE,
        OUTPUTCLAW_OPEN,

        HANG_UP1,
        HANG_DOWN1,
        HANG_UP2,
        HANG_DOWN2,

        INIT
    }

    TeleState telestate = TeleState.INIT;

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
    public static double intakeClawOpenPos;
    public static double intakeClawClosedPos;
    public static double alphaTransferPos;
    public static double betaTransferPos;
    public static double alphaIntakePos;
    public static double betaIntakePos;
    public static double alphaLowerPos;
    public static double betaLowerPos;
    public static double intakeWristStraightPos;
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
    public static double deltaRightTransferPos;
    public static double deltaLeftTransferPos;
    public static double deltaRightSamplePos;
    public static double deltaLeftSamplePos;
    public static double deltaRightSpecimenPos;
    public static double deltaLeftSpecimenPos;
    public static double deltaRightOzonePos;
    public static double deltaLeftOzonePos;
    public static double outputWristStraightPos;
    public static double outputWristSwitchPos;
    public static double outputWristSpecimenPos;
    public static double outputClawClosedPos;
    public static double outputClawOpenPos;

    //times
    final double INTAKELOWER_TIME = 0.10;
    final double INTAKECLAW_TIME = 0.20;
    final double OUTPUTCLAW_TIME = 0.20;
    final double HANG1_UP_TIME = 0.70;
    final double HANG2_UP_TIME = 1.00;

    //loop constants
    double timeGap;
    double loopCumulativeTime = 0.0;
    double loopCounter = 0.0;

    boolean turtle;
    boolean carabinerPressed;

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

        leftDS = hardwareMap.get(DistanceSensor.class, "leftDS");
        rightDS = hardwareMap.get(DistanceSensor.class, "rightDS");

        deltaLeft.setDirection(Servo.Direction.REVERSE);
        deltaLeft.setDirection(Servo.Direction.REVERSE);
        outputClaw.setDirection(Servo.Direction.REVERSE);

        slidePositionTargetEx = 0.0;
        slidePositionTargetVe = 0.0;

        turtle = false;
        carabinerPressed = false;

        for(LynxModule hub : bothHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void start(){
        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ls.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ms.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rs.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ls.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ms.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rs.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop(){
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

        switch(telestate){
            case INIT:
                if(gamepad2.a && !gamepad2prev.a){
                    slidePositionTargetEx = slideMaxEx;
                    alpha.setPosition(alphaIntakePos);
                    beta.setPosition(betaIntakePos);
                    telestate = TeleState.ARM_INTAKE;
                }
                break;
            case ARM_INTAKE:
                if(gamepad2.b && !gamepad2prev.b){
                    alpha.setPosition(alphaLowerPos);
                    beta.setPosition(betaLowerPos);
                    telestate = TeleState.INTAKECLAW_CLOSE;
                    stateTimer.reset();
                }
                break;
            case INTAKECLAW_CLOSE:
                if(stateTimer.seconds() >= INTAKELOWER_TIME){
                    intakeClaw.setPosition(intakeClawClosedPos);
                    telestate = TeleState.BRING_IN;
                    stateTimer.reset();
                }
                break;
            case BRING_IN:
                if(stateTimer.seconds() >= INTAKECLAW_TIME){
                   slidePositionTargetEx = slideMin;
                   intakeWrist.setPosition(intakeWristStraightPos);
                   alpha.setPosition(alphaTransferPos);
                   beta.setPosition(betaTransferPos);
                   telestate = TeleState.OUTPUTCLAW_CLOSE;
                }
                break;
            case OUTPUTCLAW_CLOSE:
                if(Math.abs(extendo.getCurrentPosition() - slidePositionTargetEx) <= 15){  //TODO: add code to check if sensor is hit
                    outputClaw.setPosition(outputClawClosedPos);
                    telestate = TeleState.INTAKECLAW_OPEN;
                    stateTimer.reset();
                }
                break;
            case INTAKECLAW_OPEN:
                if(stateTimer.seconds() >= OUTPUTCLAW_TIME){
                    intakeClaw.setPosition(intakeClawOpenPos);
                    telestate = TeleState.OUTPUT_BRANCH;
                }
                break;
            case OUTPUT_BRANCH:
                if(gamepad1.x){
                    telestate = TeleState.OUTPUT_SPECIMEN;
                }
                if(gamepad1.y){
                    telestate = TeleState.OUTPUT_BASKET;
                }
                if(gamepad1.b){
                    telestate = TeleState.OUTPUT_OZONE;
                }
                break;
            case OUTPUT_BASKET:
                slidePositionTargetVe = slideMaxVe;
                deltaLeft.setPosition(deltaLeftSamplePos);
                deltaRight.setPosition(deltaRightSamplePos);
                outputWrist.setPosition(outputWristSwitchPos);
                telestate = TeleState.OUTPUTCLAW_OPEN;
                break;
            case OUTPUT_SPECIMEN:
                slidePositionTargetVe = slideSpecimenVe;
                deltaLeft.setPosition(deltaLeftSpecimenPos);
                deltaRight.setPosition(deltaRightSpecimenPos);
                outputWrist.setPosition(outputWristSpecimenPos);
                telestate = TeleState.OUTPUTCLAW_OPEN;
                break;
            case OUTPUT_OZONE:
                slidePositionTargetVe = slideMin;
                deltaLeft.setPosition(deltaLeftOzonePos);
                deltaRight.setPosition(deltaRightOzonePos);
                outputWrist.setPosition(outputWristStraightPos);
                telestate = TeleState.OUTPUTCLAW_OPEN;
                break;
            case OUTPUTCLAW_OPEN:
                if(gamepad1.y){
                    outputClaw.setPosition(outputClawOpenPos);
                    outputWrist.setPosition(outputWristStraightPos);
                    deltaLeft.setPosition(deltaLeftTransferPos);
                    deltaRight.setPosition(deltaRightTransferPos);
                    slidePositionTargetVe = slideMin;
                    telestate = TeleState.INIT;
                }
                break;
            case HANG_UP1:
                slidePositionTargetVe = slideHang1Ve;
                telestate = TeleState.HANG_DOWN1;
                stateTimer.reset();
                break;
            case HANG_DOWN1:
                if(stateTimer.seconds() >= HANG1_UP_TIME){
                    slidePositionTargetVe = 0;
                    if(carabinerPressed){
                        slidePositionTargetVe = 150;
                        carabinerPressed = false;
                        telestate = TeleState.HANG_UP2;
                    }
                }
            case HANG_UP2:
                if(gamepad1.dpad_right && !gamepad1prev.dpad_right){
                    slidePositionTargetVe = slideHang2Ve;
                    telestate = TeleState.HANG_DOWN2;
                    stateTimer.reset();
                    break;
                }
            case HANG_DOWN2:
                if(stateTimer.seconds() >= HANG2_UP_TIME){

                }
        }


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
        extendo.setPower(1.3*slidesPidExtendo.calculatePowerExtendo(slidePositionTargetEx));
        slidesPidExtendo.updateEx(extendo.getCurrentPosition(), timeGap);

        rs.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ls.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ms.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalPower = 1.9*slidesPidVertical.calculatePowerVertical(slidePositionTargetVe);
        ls.setPower(verticalPower);
        ms.setPower(verticalPower);
        rs.setPower(verticalPower);
        slidesPidVertical.updateVe((ms.getCurrentPosition()), timeGap);

        if(carabinerLimit.isPressed()){
            carabinerPressed = true;
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

        if(gamepad1.right_bumper && !gamepad1prev.right_bumper){
            outputWrist.setPosition(outputWristSwitchPos);
        }


        //hang
        if(gamepad1.dpad_up && !gamepad1prev.dpad_up){
            telestate = TeleState.HANG_UP1;
        }

        //tuning positions
        /*if(gamepad2.dpad_right){
            somePos += 0.01;
            if(somePos > 1.0){
                somePos = 1.0;
            }
        }

        if(gamepad2.dpad_left){
            somePos -= 0.01;
            if(somePos < 0.0){
                somePos = 0.0;
            }
        }
        someServo.setPosition(somePos);
        */

        //drivetrain
        double y = -gamepad1.left_stick_x; //verticals
        double x = -gamepad1.left_stick_y; //horizontal
        double r = -gamepad1.right_stick_x; //pivot and rotation
        double scalar = 0.5;

        if(gamepad1.left_bumper && !gamepad1prev.left_bumper){
            if(turtle){
                turtle = false;
            }
            else{
                turtle = true;
            }
        }

        if(turtle){
            scalar = 0.3;
        }
        else{
            scalar = 0.85;
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

        telemetry.addData("battery voltage", "battery voltage: " + voltageSensor.getVoltage());
        telemetry.addData("carabiner pressed", "carabiner pressed: " + carabinerLimit.isPressed());
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
        telemetry.addData("Error Vertical", "Error vertical: " + (slidePositionTargetVe - (ms.getCurrentPosition())));

        telemetry.addData("Error Extendo", "Error Extendo: " + (slidePositionTargetEx - extendo.getCurrentPosition()));
        telemetry.addData("extendo position", "extendo position: " + extendo.getCurrentPosition());
        telemetry.addData("extendo power", "extendo power: " + extendo.getPower());
        telemetry.addData("extendo slides pressed", "extendo slides pressed: " + extendoSlidesLimit.isPressed());

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
        rs.setPower(0);
        ms.setPower(0);
        ls.setPower(0);
    }
}
