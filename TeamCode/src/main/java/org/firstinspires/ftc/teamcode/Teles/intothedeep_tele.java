package org.firstinspires.ftc.teamcode.Teles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp
public class intothedeep_tele extends intothedeep_opmode {

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
        OUTPUTCLAWSPECIMEN_OPEN,
        SPECIMEN_DELAY,
        RESET_SPECIMEN,
        RESET,

        HANG_UP1,
        HANG_DOWN1,
        HANG_UP2,
        HANG_DOWN2,
        CARABINER,

        INIT
    }

    TeleState telestate = TeleState.INIT;

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
    public void childLoop(){

        if(exAddPower){
            exConstantPID = -0.4;
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

        if(veHangPower){
            veConstantPID = -1.0;
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
                    if(intakeColor.blue() > 300 || intakeColor.red() > 300 || intakeColor.green() > 300) {
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
                    if(intakeColor.blue() > 300 || intakeColor.red() > 300 || intakeColor.green() > 300){
                        exAddPower = false;
                        deltaLeft.setPosition(deltaLeftTransferPos);
                        deltaRight.setPosition(deltaRightTransferPos);
                        telestate = TeleState.OUTPUTCLAW_CLOSE;
                        stateTimer.reset();
                        }
                    else{
                        slidePositionTargetEx = slideMaxEx;
                        alpha.setPosition(alphaIntakePos);
                        beta.setPosition(betaIntakePos);
                        intakeClaw.setPosition(intakeClawOpenPos);
                        intakeWristPos = intakeWristStraightPos;
                        intakeWrist.setPosition(intakeWristStraightPos);
                        telestate = TeleState.ARM_INTAKE;
                        }
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
                if(specimenOutputState && stateTimer.seconds() > 0.10){
                    telestate = TeleState.OUTPUT_SPECIMEN;
                }
                if(sampleOutputState && stateTimer.seconds() > 0.10){
                    telestate = TeleState.OUTPUT_BASKET;
                }
                if(ozoneOutputState && stateTimer.seconds() > 0.10){
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
                veAddPowerUp = false;
                slidePositionTargetVe = slideSpecimenVe;
                deltaLeft.setPosition(deltaLeftSpecimenPos);
                deltaRight.setPosition(deltaRightSpecimenPos);
                outputWrist.setPosition(outputWristSpecimenPos);
                outputWristSpecimen = true;
                stateTimer.reset();
                telestate = TeleState.SPECIMEN_DELAY;
                break;
            case SPECIMEN_DELAY:
                if(stateTimer.seconds() >= 0.1){
                slidePositionTargetEx = slideMaxEx;
                alpha.setPosition(alphaIntakePos);
                beta.setPosition(betaIntakePos);
                telestate = TeleState.OUTPUTCLAWSPECIMEN_OPEN;
                }
                break;
            case OUTPUT_OZONE:
                slidePositionTargetVe = slideMin;
                deltaLeft.setPosition(deltaLeftSpecimenPos);
                deltaRight.setPosition(deltaRightSpecimenPos);
                outputWrist.setPosition(outputWristStraightPos);
                telestate = TeleState.OUTPUTCLAW_OPEN;
                break;
            case OUTPUTCLAWSPECIMEN_OPEN:
                veAddPowerUp = false;
                if(gamepad1.a && !gamepad1prev.a){
                    outputClaw.setPosition(outputClawOpenPos);
                    telestate = TeleState.RESET_SPECIMEN;
                    stateTimer.reset();
                }
                break;
            case OUTPUTCLAW_OPEN:
                veAddPowerUp = false;
                if(gamepad1.a && !gamepad1prev.a || outputDS.getDistance(DistanceUnit.MM) < 100){
                    outputClaw.setPosition(outputClawOpenPos);
                    telestate = TeleState.RESET;
                    stateTimer.reset();
                }
                break;
            case RESET_SPECIMEN:
                if(stateTimer.seconds() >= 0.35 && outputDS.getDistance(DistanceUnit.MM) > 350){
                    outputWrist.setPosition(outputWristStraightPos);
                    deltaLeft.setPosition(deltaLeftPreTransfer);
                    deltaRight.setPosition(deltaRightPreTransfer);
                    veAddPowerDown = true;
                    slidePositionTargetVe = slideMin;
                    specimenOutputState = false;
                    ozoneOutputState = false;
                    sampleOutputState = false;
                    telestate = TeleState.BRING_IN;
                }
                break;
            case RESET:
                if(stateTimer.seconds() >= 0.35 && outputDS.getDistance(DistanceUnit.MM) > 350){
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
                    veHangPower = true;
                    slidePositionTargetVe = slideMin;
                    if(carabinerPressed){
                        telestate = TeleState.CARABINER;
                        stateTimer.reset();
                    }
                }
                break;
            case CARABINER:
                if(!carabinerPressed){
                    veHangPower = true;
                    slidePositionTargetVe = slideMin;
                    if(stateTimer.seconds() >= CARABINER_TIME){
                        veHangPower = false;
                        telestate = TeleState.HANG_UP2;
                    }
                }
                break;
            case HANG_UP2:
                if(gamepad1.dpad_right && !gamepad1prev.dpad_right){
                    veAddPowerUp = true;
                    slidePositionTargetVe = slideHang2Ve;
                    telestate = TeleState.HANG_DOWN2;
                    stateTimer.reset();
                }
                break;
            case HANG_DOWN2:
                if(stateTimer.seconds() >= HANG2_UP_TIME){
                    veAddPowerDown = true;
                    slidePositionTargetVe = slideMin;
                    if(carabinerPressed){
                        telestate = TeleState.CARABINER;
                        stateTimer.reset();
                    }
                }
                break;
        }

        //intake wrist
        if(gamepad2.left_trigger > 0.0){
            intakeWristPos -= 0.03;
            if(intakeWristPos < intakeWristLeftLimit){
                intakeWristPos = intakeWristLeftLimit;
            }
            intakeWrist.setPosition(intakeWristPos);
        }

        if(gamepad2.right_trigger > 0.0){
            intakeWristPos += 0.03;
            if(intakeWristPos > intakeWristRightLimit){
                intakeWristPos = intakeWristRightLimit;
            }
            intakeWrist.setPosition(intakeWristPos);
        }

        if(gamepad1.right_bumper && !gamepad1prev.right_bumper){
            if(outputWristSpecimen){
                outputWrist.setPosition(outputWristSwitchPos);
                slidePositionTargetVe += 100;
                outputWristSpecimen = false;
            }else{
                outputWrist.setPosition(outputWristSpecimenPos);
                slidePositionTargetVe -= 100;
                outputWristSpecimen = true;
            }
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
        double scalar = 0.95;

        if(gamepad1.left_bumper && !gamepad1prev.left_bumper){
            if(turtle){
                turtle = false;
            }
            else{
                turtle = true;
            }
        }

        if(turtle){
            scalar = 0.5;
        }
        else{
            scalar = 0.95;
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

        telemetry.addData("tele state", "tele state: " + telestate);
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
