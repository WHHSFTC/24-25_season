package org.firstinspires.ftc.teamcode.Teles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
        OUTPUTARM_TRANSFER,
        OUTPUTCLAW_OPEN,
        RESET,

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
    TouchSensor transferLimit;
    ColorSensor intakeColor;
    DistanceSensor leftDS;
    DistanceSensor rightDS;
    DistanceSensor outputDS;

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
    public static double slideMaxVe = 1950.0;
    public static double slideSpecimenVe = 530.0;
    public static double slideHang1Ve = 1400;
    public static double slideHang2Ve = 1950;
    double slidePositionTargetEx;
    double slidePositionTargetVe;
    double verticalPower;
    SlidesPID slidesPidExtendo;
    SlidesPID slidesPidVertical;

    //output constants
    public static double deltaRightPreTransfer = 0.58;
    public static double deltaLeftPreTransfer = 0.21;
    public static double deltaRightTransferPos = 0.63;
    public static double deltaLeftTransferPos = 0.15;
    public static double deltaRightSamplePos = 0.45;
    public static double deltaLeftSamplePos = 0.34;
    public static double deltaRightSpecimenPos = 0.35;
    public static double deltaLeftSpecimenPos = 0.20;
    public static double outputWristStraightPos = 0.90;
    public static double outputWristSwitchPos = 0.23;
    public static double outputWristSpecimenPos = 0.30;
    public static double outputClawClosedPos = 0.48;
    public static double outputClawOpenPos = 0.76;

    //times
    final double INTAKELOWER_TIME = 0.05;
    final double INTAKECLAW_TIME = 0.40;
    final double OUTPUTCLAW_TIME = 0.25;
    final double OUTPUTARM_READY = 0.20;
    final double HANG1_UP_TIME = 3.50;
    final double HANG2_UP_TIME = 3.50;

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
    public static double exConstantPID = 0.0;
    public static double veConstantPID = 0.0;
    boolean exAddPower = false;
    boolean veAddPowerDown = false;
    boolean veAddPowerUp = false;
    double tuningPos1 = 0.5;
    double tuningPos2 = 0.5;

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
        transferLimit = hardwareMap.get(TouchSensor.class, "transferLimit");
        intakeColor = hardwareMap.get(ColorSensor.class, "intakeColor");
        outputDS = hardwareMap.get(DistanceSensor.class, "outputDS");

        /*leftDS = hardwareMap.get(DistanceSensor.class, "leftDS");
        rightDS = hardwareMap.get(DistanceSensor.class, "rightDS");*/
        //deltaLeft.setDirection(Servo.Direction.REVERSE);
        //deltaRight.setDirection(Servo.Direction.REVERSE);

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

        intakeClaw.setPosition(intakeClawOpenPos);
        intakeWrist.setPosition(intakeWristStraightPos);
        alpha.setPosition(alphaTransferPos);
        beta.setPosition(betaTransferPos);
        outputClaw.setPosition(outputClawOpenPos);
        outputWrist.setPosition(outputWristStraightPos);
        deltaLeft.setPosition(deltaLeftPreTransfer);
        deltaRight.setPosition(deltaRightPreTransfer);
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
        verticalPower = 2.3*slidesPidVertical.calculatePowerVertical(slidePositionTargetVe) + veConstantPID;
        ls.setPower(verticalPower);
        ms.setPower(verticalPower);
        rs.setPower(verticalPower);
        slidesPidVertical.updateVe((ms.getCurrentPosition()), timeGap);

        if(exAddPower){
            exConstantPID = -0.3;
        }else{
            exConstantPID = 0.0;
        }

        if(veAddPowerDown){
            veConstantPID = -0.2;
        }else{
            veConstantPID = 0.0;
        }

        if(veAddPowerUp){
            veConstantPID = 0.2;
        }else{
            veConstantPID = 0.0;
        }

        switch(telestate){
            case INIT:
                veAddPowerDown = false;
                if(gamepad2.a && !gamepad2prev.a){
                    slidePositionTargetEx = slideMaxEx;
                    alpha.setPosition(alphaIntakePos);
                    beta.setPosition(betaIntakePos);
                    telestate = TeleState.ARM_INTAKE;
                    intakeClaw.setPosition(intakeClawOpenPos);
                    deltaLeft.setPosition(deltaLeftPreTransfer);
                    deltaRight.setPosition(deltaRightPreTransfer);
                }
                break;
            case ARM_INTAKE:
                if(gamepad2.b && !gamepad2prev.b){
                    alpha.setPosition(alphaLowerPos);
                    beta.setPosition(betaLowerPos);
                    telestate = TeleState.INTAKECLAW_CLOSE;
                    ozoneOutputState = true;
                    stateTimer.reset();
                }

                if(gamepad2.x && !gamepad2prev.x){
                    alpha.setPosition(alphaLowerPos);
                    beta.setPosition(betaLowerPos);
                    telestate = TeleState.INTAKECLAW_CLOSE;
                    specimenOutputState = true;
                    stateTimer.reset();
                }

                if(gamepad2.y && !gamepad2prev.y){
                    alpha.setPosition(alphaLowerPos);
                    beta.setPosition(betaLowerPos);
                    telestate = TeleState.INTAKECLAW_CLOSE;
                    sampleOutputState = true;
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
                    if(intakeColor.blue() > 100 || intakeColor.red() > 100 || intakeColor.green() > 100) {
                        exAddPower = true;
                        slidePositionTargetEx = slideMin;
                        intakeWrist.setPosition(intakeWristStraightPos);
                        alpha.setPosition(alphaTransferPos);
                        beta.setPosition(betaTransferPos);
                        telestate = TeleState.OUTPUTARM_TRANSFER;
                        stateTimer.reset();
                    }
                    else{
                        intakeClaw.setPosition(intakeClawOpenPos);
                        alpha.setPosition(alphaIntakePos);
                        beta.setPosition(betaIntakePos);
                        telestate = TeleState.ARM_INTAKE;
                    }
                }
                break;
            case OUTPUTARM_TRANSFER:
                if(transferPressed && extendoPressed){
                    exAddPower = false;
                    deltaLeft.setPosition(deltaLeftTransferPos);
                    deltaRight.setPosition(deltaRightTransferPos);
                    telestate = TeleState.OUTPUTCLAW_CLOSE;
                    stateTimer.reset();
                }
                break;
            case OUTPUTCLAW_CLOSE:
                if(stateTimer.seconds() >= OUTPUTARM_READY){
                    outputClaw.setPosition(outputClawClosedPos);
                    telestate = TeleState.INTAKECLAW_OPEN;
                    stateTimer.reset();
                }
                break;
            case INTAKECLAW_OPEN:
                if(stateTimer.seconds() >= OUTPUTCLAW_TIME){
                    intakeClaw.setPosition(intakeClawOpenPos);
                    telestate = TeleState.OUTPUT_BRANCH;
                    stateTimer.reset();
                }
                break;
            case OUTPUT_BRANCH:
                if(specimenOutputState && stateTimer.seconds() > INTAKECLAW_TIME){
                    telestate = TeleState.OUTPUT_SPECIMEN;
                }
                if(sampleOutputState && stateTimer.seconds() > INTAKECLAW_TIME){
                    telestate = TeleState.OUTPUT_BASKET;
                }
                if(ozoneOutputState && stateTimer.seconds() > INTAKECLAW_TIME){
                    telestate = TeleState.OUTPUT_OZONE;
                }
                break;
            case OUTPUT_BASKET:
                veAddPowerUp = true;
                slidePositionTargetVe = slideMaxVe;
                deltaLeft.setPosition(deltaLeftSamplePos);
                deltaRight.setPosition(deltaRightSamplePos);
                outputWrist.setPosition(outputWristSwitchPos);
                telestate = TeleState.OUTPUTCLAW_OPEN;
                break;
            case OUTPUT_SPECIMEN:
                veAddPowerUp = true;
                slidePositionTargetVe = slideSpecimenVe;
                deltaLeft.setPosition(deltaLeftSpecimenPos);
                deltaRight.setPosition(deltaRightSpecimenPos);
                outputWrist.setPosition(outputWristSpecimenPos);
                telestate = TeleState.OUTPUTCLAW_OPEN;
                break;
            case OUTPUT_OZONE:
                slidePositionTargetVe = slideMin;
                deltaLeft.setPosition(deltaLeftSpecimenPos);
                deltaRight.setPosition(deltaRightSpecimenPos);
                outputWrist.setPosition(outputWristStraightPos);
                telestate = TeleState.OUTPUTCLAW_OPEN;
                break;
            case OUTPUTCLAW_OPEN:
                veAddPowerUp = false;
                if(gamepad1.a && !gamepad1prev.a || outputDS.getDistance(DistanceUnit.MM) < 100){
                    outputClaw.setPosition(outputClawOpenPos);
                    telestate = TeleState.RESET;
                    stateTimer.reset();
                }
                break;
            case RESET:
                if(stateTimer.seconds() >= OUTPUTCLAW_TIME && outputDS.getDistance(DistanceUnit.MM) > 350){
                    outputWrist.setPosition(outputWristStraightPos);
                    deltaLeft.setPosition(deltaLeftPreTransfer);
                    deltaRight.setPosition(deltaRightPreTransfer);
                    veAddPowerDown = true;
                    slidePositionTargetVe = slideMin;
                    specimenOutputState = false;
                    ozoneOutputState = false;
                    sampleOutputState = false;
                    telestate = TeleState.INIT;
                }
                break;
            case HANG_UP1:
                veAddPowerUp = true;
                slidePositionTargetVe = slideHang1Ve;
                telestate = TeleState.HANG_DOWN1;
                stateTimer.reset();
                break;
            case HANG_DOWN1:
                if(stateTimer.seconds() >= HANG1_UP_TIME){
                    veAddPowerDown = true;
                    slidePositionTargetVe = slideMin;
                    if(carabinerPressed){
                        slidePositionTargetVe = 150;
                        carabinerPressed = false;
                        telestate = TeleState.HANG_UP2;
                    }
                }
            case HANG_UP2:
                if(gamepad1.dpad_right && !gamepad1prev.dpad_right){
                    veAddPowerUp = true;
                    slidePositionTargetVe = slideHang2Ve;
                    telestate = TeleState.HANG_DOWN2;
                    stateTimer.reset();
                    break;
                }
            case HANG_DOWN2:
                if(stateTimer.seconds() >= HANG2_UP_TIME){
                    veAddPowerDown = true;
                    slidePositionTargetVe = slideMin;
                    if(carabinerPressed){
                        slidePositionTargetVe = 150;
                    }
                }
        }

        if(transferLimit.isPressed()){
            transferPressed = true;
        }else{
            transferPressed = false;
        }

        if(carabinerLimit.isPressed()){
            carabinerPressed = true;
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

        //intake wrist
        if(gamepad2.left_trigger > 0.0){
            intakeWristPos -= 0.01;
            if(intakeWristPos < intakeWristLeftLimit){
                intakeWristPos = intakeWristLeftLimit;
            }
            intakeWrist.setPosition(intakeWristPos);
        }

        if(gamepad2.right_trigger > 0.0){
            intakeWristPos += 0.01;
            if(intakeWristPos > intakeWristRightLimit){
                intakeWristPos = intakeWristRightLimit;
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
            tuningPos1 += 0.01;
            if(tuningPos1 > 1.0){
                tuningPos1 = 1.0;
            }
        }

        if(gamepad2.dpad_left){
            tuningPos1 -= 0.01;
            if(tuningPos1 < 0.0){
                tuningPos1 = 0.0;
            }
        }

        if(gamepad2.dpad_up){
            tuningPos2 += 0.01;
            if(tuningPos2 > 1.0){
                tuningPos2 = 1.0;
            }
        }

        if(gamepad2.dpad_down){
            tuningPos2 -= 0.01;
            if(tuningPos2 < 0.0){
                tuningPos2 = 0.0;
            }
        }

        deltaLeft.setPosition(tuningPos1);
        deltaRight.setPosition(tuningPos2);*/

        //drivetrain
        double y = -gamepad1.left_stick_x; //verticals
        double x = -gamepad1.left_stick_y; //horizontal
        double r = -gamepad1.right_stick_x; //pivot and rotation
        double scalar = 0.85;

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

        telemetry.addData("intake color red", "red: " + intakeColor.red());
        telemetry.addData("intake color blue", "blue: " + intakeColor.blue());
        telemetry.addData("intake color green", "green: " + intakeColor.green());
        telemetry.addData("transferPressed", "transfer pressed: " + transferPressed);
        //telemetry.addData("outputDS", "outputDS: " + outputDS.getDistance(DistanceUnit.MM));
        //telemetry.addData("color sensor output blue", "cs blue: " + intakeColor.blue());
        //telemetry.addData("color sensor output red", "cs red: " + intakeColor.red());
        //telemetry.addData("color sensor output red", "cs green: " + intakeColor.green());
        telemetry.addData("battery voltage", "battery voltage: " + voltageSensor.getVoltage());
        telemetry.addData("tele state", "tele state: " + telestate);
        //telemetry.addData("carabiner pressed", "carabiner pressed: " + carabinerLimit.isPressed());
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
        //telemetry.addData("extendo slides pressed", "extendo slides pressed: " + extendoSlidesLimit.isPressed());

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
