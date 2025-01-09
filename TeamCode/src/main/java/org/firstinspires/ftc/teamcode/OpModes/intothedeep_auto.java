package org.firstinspires.ftc.teamcode.OpModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

import java.util.List;

@Config
@Autonomous

public class intothedeep_auto extends OpMode {
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

    //intake constants
    public static double intakeClawOpenPos = 0.10;
    public static double intakeClawClosedPos = 0.45;
    public static double alphaTransferPos = 0.52;
    public static double betaTransferPos = 0.52;
    public static double alphaIntakePos = 0.92;
    public static double betaIntakePos = 0.92;
    public static double alphaLowerPos = 1.0;
    public static double betaLowerPos = 1.0;
    public static double intakeWristStraightPos = 0.52;
    public static double intakeWristRightLimit = 0.73;
    public static double intakeWristLeftLimit = 0.29;
    public static double intakeWristPos;

    //slides constants
    public static double slideMinEx = -20.0;
    public static double slideMinVe = -5.0;
    public static double slideMaxEx = 450.0;
    public static double slideMaxVe = 1830.0;
    public static double slideSpecimenVe = 400.0;
    public static double slideHangVe = 1500.0;
    public static double swingSizeEx = 120.0;
    public static double slidePositionTargetEx;
    public static double slidePositionTargetVe;
    double verticalPower;
    SlidesPID slidesPidExtendo;
    SlidesPID slidesPidVertical;

    //output constants
    public static double deltaRightPreTransfer = 0.58;
    public static double deltaLeftPreTransfer = 0.23;
    public static double deltaRightTransferPos = 0.64;
    public static double deltaLeftTransferPos = 0.17;
    public static double deltaRightSamplePos = 0.44;
    public static double deltaLeftSamplePos = 0.37;
    public static double deltaRightSpecimenPos = 0.28;
    public static double deltaLeftSpecimenPos = 0.27;
    public static double outputWristStraightPos = 0.90;
    public static double outputWristSwitchPos = 0.23;
    public static double outputWristSpecimenPos = 0.15;
    public static double outputClawClosedPos = 0.48;
    public static double outputClawOpenPos = 0.76;

    public static double springToggleOnPos = 0.58;
    public static double springToggleOffPos = 0.94;


    public static double exConstantPID = 0.0;
    public static double veConstantPID = 0.0;

    @Override
    public void init(){
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

        extendoSlidesLimit = hardwareMap.get(TouchSensor.class, "extendoSlidesLimit");
        verticalSlidesLimit = hardwareMap.get(TouchSensor.class, "verticalSlidesLimit");
        carabinerLimit = hardwareMap.get(TouchSensor.class, "carabinerLimit");
        transferLimit = hardwareMap.get(TouchSensor.class, "transferLimit");
        //intakeColor = hardwareMap.get(RevColorSensorV3.class, "intakeColor");
        outputDS = hardwareMap.get(DistanceSensor.class, "outputDS");
        chamberDS = hardwareMap.get(DistanceSensor.class, "chamberDS");

        outputClaw.setDirection(Servo.Direction.REVERSE);
        beta.setDirection(Servo.Direction.REVERSE);

        rs.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ls.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ms.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
        transferLimit.isPressed();

        childLoop();
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

    public class VertSlides implements Action {
        double slidePosTargetVe;
        double vertPower;
        ElapsedTime timer = new ElapsedTime();
        double timeSlides;
        boolean veAddPowerDown;
        public VertSlides(double slideVePos, boolean addPower){
            slidePosTargetVe = slideVePos;
            veAddPowerDown = addPower;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if(veAddPowerDown){
                veConstantPID = -0.4;
            }else{
                veConstantPID = 0.0;
            }

            timeSlides = timer.seconds();
            ms.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rs.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ls.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slidesPidVertical.updateVeAuto((ms.getCurrentPosition()), timeSlides);
            vertPower = 2.0*slidesPidVertical.autoCalculatePowerVertical(slidePosTargetVe) + veConstantPID;
            ls.setPower(vertPower);
            ms.setPower(vertPower);
            rs.setPower(vertPower);

            double veDifference = Math.abs(slidePosTargetVe - ms.getCurrentPosition());
            boolean veThreshold = veDifference > 8;

            /*packet.put("vetarget", slidePosTargetVe);
            packet.put("veactual", ms.getCurrentPosition());
            packet.put("difference", veDifference);
            packet.put("threshold",veThreshold);
            packet.put("timegap", timer);
            packet.put("vepower", ms.getPower());
            dashboard.sendTelemetryPacket(packet);*/

            timer.reset();

            if(veThreshold){
                return true;
            }
            else{
                return false;
            }
        }
    }

    public Action vertslides(double slideVePos, boolean addPower) {
        return new VertSlides(slideVePos, addPower);
    }

    public class ExtendoSlides implements Action {
        double slidePosTargetEx;
        ElapsedTime timer = new ElapsedTime();
        double timeSlides;
        double extendPower;
        boolean exAddPower;
        public ExtendoSlides(double slideExPos, boolean addPower){
            slidePosTargetEx = slideExPos;
            exAddPower = addPower;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if(exAddPower){
                exConstantPID = -0.5;
            }else{
                exConstantPID = 0.0;
            }

            timeSlides = timer.seconds();
            extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slidesPidExtendo.updateExAuto(extendo.getCurrentPosition(), timeSlides);
            extendPower = 2.4*slidesPidExtendo.autoCalculatePowerExtendo(slidePosTargetEx) + exConstantPID;
            extendo.setPower(extendPower);

            double exDifference = Math.abs(slidePosTargetEx - extendo.getCurrentPosition());
            boolean exThreshold = exDifference > 8;

            packet.put("extarget", slidePosTargetEx);
            packet.put("exactual", extendo.getCurrentPosition());
            packet.put("exdifference", exDifference);
            packet.put("exthreshold",exThreshold);
            packet.put("timegap", timeSlides);
            packet.put("expower", extendo.getPower());
            dashboard.sendTelemetryPacket(packet);

            timer.reset();

            if(exThreshold){
                return true;
            }
            else{
                return false;
            }
        }
    }

    public Action extendslides(double slideExPos, boolean addPower) {
        return new ExtendoSlides(slideExPos, addPower);
    }


    public class OutputArm implements Action {
        double dlPos;
        double drPos;
        public OutputArm(double deltaLeftPos, double deltaRightPos){
            dlPos = deltaLeftPos;
            drPos = deltaRightPos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            deltaLeft.setPosition(dlPos);
            deltaRight.setPosition(drPos);
            return false;
        }
    }

    public Action outputarm(double deltaLeftPos, double deltaRightPos) {
        return new OutputArm(deltaLeftPos, deltaRightPos);
    }


    public class OutputWrist implements Action {
        double oWristPos;
        public OutputWrist(double outputwristpos){
            oWristPos = outputwristpos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            outputWrist.setPosition(oWristPos);
            return false;
        }
    }

    public Action outputwrist(double outputwristpos) {
        return new OutputWrist(outputwristpos);
    }


    public class OutputClaw implements Action {
        double oClawPos;
        public OutputClaw(double outputclawpos){
            oClawPos = outputclawpos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            outputClaw.setPosition(oClawPos);
            return false;
        }
    }

    public Action outputclaw(double outputclawpos) {
        return new OutputClaw(outputclawpos);
    }


    public class SpringToggle implements Action {
        double sTogglePos;
        public SpringToggle(double springtogglepos){
            sTogglePos = springtogglepos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            springToggle.setPosition(sTogglePos);
            return false;
        }
    }

    public Action springtoggle(double springtogglepos) {
        return new SpringToggle(springtogglepos);
    }

    public class IntakeClaw implements Action {
        double iClawPos;
        ElapsedTime intakeClawTimer = new ElapsedTime();
        public IntakeClaw(double intakeclawpos){
            iClawPos = intakeclawpos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intakeClaw.setPosition(iClawPos);
            if(intakeClawTimer.seconds() > 0.35){
                intakeClawTimer.reset();
                return false;
            }
            else{
                return true;
            }
        }
    }

    public Action intakeclaw(double intakeclawpos) {
        return new IntakeClaw(intakeclawpos);
    }


    public class IntakeWrist implements Action {
        double iWristPos;
        public IntakeWrist(double intakewristpos){
            iWristPos = intakewristpos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intakeWrist.setPosition(iWristPos);
            return false;
        }
    }

    public Action intakewrist(double intakewristpos) {
        return new IntakeWrist(intakewristpos);
    }


    public class IntakeArm implements Action {
        double aPos;
        double bPos;
        public IntakeArm(double alphapos, double betapos){
            aPos = alphapos;
            bPos = betapos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            alpha.setPosition(aPos);
            beta.setPosition(bPos);
            return false;
        }
    }

    public Action intakearm(double alphapos, double betapos) {
        return new IntakeArm(alphapos, betapos);
    }


    public Action transfer(){
        return new InstantAction(()->{
                deltaLeft.setPosition(deltaLeftTransferPos);
                deltaRight.setPosition(deltaRightTransferPos);
                new SleepAction(0.2);
                outputClaw.setPosition(outputClawClosedPos);
                new SleepAction(0.4);
                intakeClaw.setPosition(intakeClawOpenPos);
                new SleepAction(0.5);
                deltaLeft.setPosition(deltaLeftSpecimenPos);
                deltaRight.setPosition(deltaRightSpecimenPos);
                outputWrist.setPosition(outputWristSpecimenPos);
        }
        );
    }

    public Action reset(){
        return new InstantAction(()->{
            deltaLeft.setPosition(deltaLeftPreTransfer);
            deltaRight.setPosition(deltaRightPreTransfer);
            outputWrist.setPosition(outputWristStraightPos);
            vertslides(slideMinVe, true);
            extendslides(slideMaxEx, false);
        });
    }

    public Action intake(){
        return new ParallelAction(
            intakearm(alphaLowerPos, betaLowerPos),
            new SleepAction(0.5),
            intakeclaw(intakeClawClosedPos),
            new SleepAction(0.5),
            intakearm(alphaTransferPos, betaTransferPos)
        );
    }
}
