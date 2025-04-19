package OpModes;

import static com.qualcomm.hardware.lynx.LynxModule.DebugGroup.I2C;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

@Config
@TeleOp

public class intothedeep_tele_red extends intothedeep_opmode {

    public enum TeleState {
        //intake
        ARM_INTAKE,
        INTAKECLAW_CLOSE,
        BRING_IN,
        OUTPUTCLAW_CLOSE,
        INTAKECLAW_OPEN,
        SIDE_INTAKE,

        //sample
        OUTPUT_BRANCH,
        OUTPUT_BASKET,
        OUTPUTARM_TRANSFER,
        OUTPUTCLAW_OPEN,
        RESET,

        //ozone
        OZONE_RETRACT,
        OZONE_EXTEND,
        DROP_OZONE,
        OZONE_RETRACTTWO,

        //specimen
        OUTPUT_SPECIMEN,
        OUTPUTCLAWSPECIMEN_OPEN,
        SPECIMEN_DELAY,
        SPECIMEN_OUTPUT,
        RESET_SPECIMEN,

        //hang
        HANG_UP1,
        HANG_DOWN1,
        HANG_UP2,
        HANG_UP2_EXTEND,
        HANG_DOWN2,
        CARABINER,
        CARABINER2,

        INIT
    }

    intothedeep_tele_blue.TeleState telestate = intothedeep_tele_blue.TeleState.INIT;

    double slideTargetGainEx = 110;
    double slideTargetGainVe = 30;
    double exPowerRaw = 0;
    double exPowerScaled = 0;

    @Override
    public void init(){
        super.init();
        springToggle.setPosition(springToggleOffPos);
    }

    @Override
    public void start(){
        super.start();
        intakeClaw.setPosition(intakeClawOpenPosTele);
        intakeWrist.setPosition(intakeWristStraightPos);
        alpha.setPosition(alphaTransferPos);
        beta.setPosition(betaTransferPos);
        outputClaw.setPosition(outputClawOpenPos);
        outputWrist.setPosition(outputWristStraightPos);
        deltaLeft.setPosition(deltaLeftPreTransfer);
        deltaRight.setPosition(deltaRightPreTransfer);
        springToggle.setPosition(springToggleOffPos);
        limelight.pipelineSwitch(9);
        blueAlliance = false;

        slidePositionTargetVe = 0.0;
        slidePositionTargetEx = 0.0;

        intakeWristPos = intakeWristStraightPos;
    }

    public void checkForA() {
        if(gamepad2.a && !gamepad2prev.a){
            intakeWristPos = intakeWristStraightPos;
            slidePositionTargetEx = slideMaxEx;
            alpha.setPosition(alphaIntakePos);
            beta.setPosition(betaIntakePos);
            telestate = intothedeep_tele_blue.TeleState.ARM_INTAKE;
            intakeClaw.setPosition(intakeClawOpenPosTele);
            deltaLeft.setPosition(deltaLeftPreTransfer);
            deltaRight.setPosition(deltaRightPreTransfer);
            ozoneOutputState = false;
            specimenOutputState = false;
            sampleOutputState = false;
        }
    }

    public boolean isCorrectColor(ColorSensor sensorOutput) {
        if(specimenOutputState || intakeClaw.getPosition() == intakeClawOpenPos || (gamepad2.dpad_left && gamepad2prev.dpad_left)){
            return true;
        }
        else{
            if(blueAlliance) {
                if(specimenOutputState || ozoneOutputState){
                    return sensorOutput.blue() > 0.5 * colorThreshold && sensorOutput.red() < colorThreshold * 1.7 &&
                            sensorOutput.green() < colorThreshold * 1.7;
                }
                if(sampleOutputState){
                    return (sensorOutput.blue() > colorThreshold || sensorOutput.green() > colorThreshold)
                            && (sensorOutput.red() < sensorOutput.green() || sensorOutput.red() < sensorOutput.blue());
                }
            }
            else {
                if (specimenOutputState || ozoneOutputState) {
                    return sensorOutput.blue() < 0.5 * colorThreshold
                            && sensorOutput.red() > colorThreshold * 1.7 && sensorOutput.green() < colorThreshold * 1.7;
                }
                if(sampleOutputState){
                    return (sensorOutput.red() > colorThreshold || sensorOutput.green() > colorThreshold)
                            && sensorOutput.blue() < sensorOutput.green() || sensorOutput.blue() < sensorOutput.red();
                }
            }
            return true;
        }
    }

    @Override
    public void childLoop(){
        super.childLoop();

        if(exAddPower){
            exConstantPID = -1.0;
        }else{
            exConstantPID = 0.0;
        }

        if(veAddPowerDown){
            veConstantPID = -0.8;
        }else if(veHangPower){
            veConstantPID = -2.0;
        }
        else if(veAddPowerUp){
            veConstantPID = 2.0;
        }else{
            veConstantPID = 0.0;
        }

        if(gamepad2.dpad_down && gamepad2prev.dpad_down){
            extendo.setPower(-1.0);
        }
        else{
            exPowerRaw = slidesPidExtendo.calculatePowerExtendo(slidePositionTargetEx);
            exPowerScaled = 2.4*slidesPidExtendo.calculatePowerExtendo(slidePositionTargetEx);
            extendo.setPower(2.4*slidesPidExtendo.calculatePowerExtendo(slidePositionTargetEx) + exConstantPID);
            slidesPidExtendo.updateEx(extendo.getCurrentPosition(), timeGap);
        }

        verticalPower = 2.0*slidesPidVertical.calculatePowerVertical(slidePositionTargetVe) + veConstantPID;
        ls.setPower(verticalPower);
        ms.setPower(verticalPower);
        rs.setPower(verticalPower);
        slidesPidVertical.updateVe((ms.getCurrentPosition()), timeGap);

        switch(telestate){
            case INIT:
                veAddPowerDown = false;
                veHangPower = false;
                veAddPowerUp = false;

                intakeWristPos = intakeWristStraightPos;
                intakeClaw.setPosition(intakeClawOpenPosTele);
                if(ms.getCurrentPosition() < -10) springToggle.setPosition(springToggleOffPos);
                exAddPower = false;
                checkForA();
                break;
            case ARM_INTAKE:

                deltaLeft.setPosition(deltaLeftPreTransfer);
                deltaRight.setPosition(deltaRightPreTransfer);

                intakeWristPos = Math.min(Math.max(
                        intakeWristPos + (gamepad2.left_trigger - gamepad2.right_trigger) * 0.02,
                        intakeWristRightLimit), intakeWristLeftLimit);

                intakeWrist.setPosition(intakeWristPos);

                if(gamepad2.b && !gamepad2prev.b){
                    alpha.setPosition(0.93);
                    beta.setPosition(0.93);
                    telestate = intothedeep_tele_blue.TeleState.INTAKECLAW_CLOSE;
                    ozoneOutputState = true;
                    specimenOutputState = false;
                    sampleOutputState = false;
                    stateTimer.reset();
                }

                if(gamepad2.x && !gamepad2prev.x){
                    alpha.setPosition(0.93);
                    beta.setPosition(0.93);
                    telestate = intothedeep_tele_blue.TeleState.INTAKECLAW_CLOSE;
                    specimenOutputState = true;
                    sampleOutputState = false;
                    ozoneOutputState = false;
                    stateTimer.reset();
                }

                if(gamepad2.right_bumper){
                    alpha.setPosition(0.93);
                    beta.setPosition(0.93);
                    telestate = intothedeep_tele_blue.TeleState.INTAKECLAW_CLOSE;
                    sampleOutputState = true;
                    sideIntakeState = true;
                    specimenOutputState = false;
                    ozoneOutputState = false;
                    stateTimer.reset();
                }

                if(gamepad2.y && !gamepad2prev.y){
                    alpha.setPosition(0.93);
                    beta.setPosition(0.93);
                    telestate = intothedeep_tele_blue.TeleState.INTAKECLAW_CLOSE;
                    sampleOutputState = true;
                    specimenOutputState = false;
                    ozoneOutputState = false;
                    stateTimer.reset();
                }
                break;
            case INTAKECLAW_CLOSE:
                if(stateTimer.seconds() >= INTAKELOWER_TIME){
                    intakeClaw.setPosition(intakeClawClosedPos);
                    telestate = intothedeep_tele_blue.TeleState.BRING_IN;
                    stateTimer.reset();
                }
                break;
            case BRING_IN:
                veAddPowerDown = false;
                if(stateTimer.seconds() >= INTAKECLAW_TIME){
                    if(isCorrectColor(intakeColor)) {
                        if(ozoneOutputState){
                            intakeWrist.setPosition(intakeWristStraightPos);
                            alpha.setPosition(alphaTransferPos);
                            beta.setPosition(betaTransferPos);
                            telestate = intothedeep_tele_blue.TeleState.OZONE_RETRACT;
                            stateTimer.reset();
                        }
                        else if(sideIntakeState){
                            exAddPower = true;
                            slidePositionTargetEx = slideMinEx;
                            exConstantPID = -1.5;
                            intakeWrist.setPosition(intakeWristStraightPos);
                            alpha.setPosition(0.70);
                            beta.setPosition(0.70);
                            telestate = intothedeep_tele_blue.TeleState.SIDE_INTAKE;
                            stateTimer.reset();
                        }
                        else{
                            exAddPower = true;
                            slidePositionTargetEx = slideMinEx;
                            exConstantPID = -1.5;
                            intakeWrist.setPosition(intakeWristStraightPos);
                            alpha.setPosition(alphaTransferPos);
                            beta.setPosition(betaTransferPos);
                            telestate = intothedeep_tele_blue.TeleState.OUTPUTARM_TRANSFER;
                            stateTimer.reset();
                        }
                    }
                    else{
                        intakeClaw.setPosition(intakeClawOpenPosTele);
                        alpha.setPosition(alphaIntakePos);
                        beta.setPosition(betaIntakePos);
                        telestate = intothedeep_tele_blue.TeleState.ARM_INTAKE;
                    }
                }
                break;
            case SIDE_INTAKE:
                checkForA();
                if(extendoSlidesLimit.isPressed() && alpha.getPosition() < alphaTransferPos + 0.01){
                    alpha.setPosition(alphaTransferPos);
                    beta.setPosition(betaTransferPos);
                    telestate = intothedeep_tele_blue.TeleState.OUTPUTARM_TRANSFER;
                    stateTimer.reset();
                }
            case OUTPUTARM_TRANSFER:
                checkForA();
                if(extendoSlidesLimit.isPressed() && alpha.getPosition() < alphaTransferPos + 0.01){
                    if(isCorrectColor(intakeColor)){
                        exAddPower = true;
                        exConstantPID = -1.5;
                        deltaLeft.setPosition(deltaLeftTransferPos);
                        deltaRight.setPosition(deltaRightTransferPos);
                        intakeClaw.setPosition(intakeClawBarelyClosedPos); //intakeclawclosedpos
                        telestate = intothedeep_tele_blue.TeleState.OUTPUTCLAW_CLOSE;
                        stateTimer.reset();
                    }
                    else{
                        exAddPower = false;
                        slidePositionTargetEx = slideMaxEx;
                        alpha.setPosition(alphaIntakePos);
                        beta.setPosition(betaIntakePos);
                        intakeClaw.setPosition(intakeClawOpenPosTele);
                        intakeWristPos = intakeWristStraightPos;
                        intakeWrist.setPosition(intakeWristStraightPos);
                        telestate = intothedeep_tele_blue.TeleState.ARM_INTAKE;
                    }
                }
                break;
            case OUTPUTCLAW_CLOSE:
                if(stateTimer.seconds() >= OUTPUTARM_READY){
                    exAddPower = true;
                    exConstantPID = -1.5;
                    outputClaw.setPosition(outputClawClosedPos);
                    telestate = intothedeep_tele_blue.TeleState.INTAKECLAW_OPEN;
                    stateTimer.reset();
                }
                break;
            case INTAKECLAW_OPEN:
                if(stateTimer.seconds() >= OUTPUTCLAW_TIME){
                    exAddPower = true;
                    exConstantPID = -1.5;
                    intakeClaw.setPosition(intakeClawOpenPosTele);
                    telestate = intothedeep_tele_blue.TeleState.OUTPUT_BRANCH;
                    stateTimer.reset();
                }
                break;
            case OUTPUT_BRANCH:
                if(specimenOutputState && stateTimer.seconds() > 0.15){
                    telestate = intothedeep_tele_blue.TeleState.OUTPUT_SPECIMEN;
                }
                if((sampleOutputState || sideIntakeState) && stateTimer.seconds() > 0.15){
                    telestate = intothedeep_tele_blue.TeleState.OUTPUT_BASKET;
                    sideIntakeState = false;
                }
                if(ozoneOutputState && stateTimer.seconds() > 0.15){
                    telestate = intothedeep_tele_blue.TeleState.OZONE_RETRACT;
                }
                break;
            case OUTPUT_BASKET:
                exAddPower = false;
                exConstantPID = 0.0;
                slidePositionTargetVe = slideMaxVe;
                deltaLeft.setPosition(deltaLeftSamplePos);
                deltaRight.setPosition(deltaRightSamplePos);
                outputWrist.setPosition(outputWristSwitchPos);
                telestate = intothedeep_tele_blue.TeleState.OUTPUTCLAW_OPEN;
                break;
            case OUTPUT_SPECIMEN:
                veAddPowerDown = false;
                exAddPower = false;
                exConstantPID = 0.0;
                slidePositionTargetVe = 450;
                deltaLeft.setPosition(deltaLeftSpecimenPos);
                deltaRight.setPosition(deltaRightSpecimenPos);
                outputWrist.setPosition(outputWristSpecimenPos);
                outputWristSpecimen = true;
                stateTimer.reset();
                telestate = intothedeep_tele_blue.TeleState.SPECIMEN_DELAY;
                break;
            case SPECIMEN_DELAY:
                if(stateTimer.seconds() >= 0.2){
                    slidePositionTargetEx = slideMaxEx;
                    alpha.setPosition(alphaIntakePos);
                    beta.setPosition(betaIntakePos);
                    telestate = intothedeep_tele_blue.TeleState.SPECIMEN_OUTPUT;
                }
                break;
            case SPECIMEN_OUTPUT:
                if((gamepad2.x && !gamepad2prev.x)){
                    outputWrist.setPosition(0.15);
                    slidePositionTargetVe += 425;
                    outputClaw.setPwmRange(new PwmControl.PwmRange(2000,2500));
                    telestate = intothedeep_tele_blue.TeleState.OUTPUTCLAWSPECIMEN_OPEN;
                }
                break;
            case OZONE_RETRACT:
                exAddPower = true;
                slidePositionTargetEx = slideMinEx;
                intakeWrist.setPosition(intakeWristStraightPos);
                telestate = intothedeep_tele_blue.TeleState.OZONE_EXTEND;
                break;
            case OZONE_EXTEND:
                if(!gamepad2.b && gamepad2prev.b){
                    exAddPower = false;
                    exConstantPID = 0.0;
                    slidePositionTargetEx = slideMaxEx;
                    alpha.setPosition(alphaIntakePos);
                    beta.setPosition(betaIntakePos);
                    telestate = intothedeep_tele_blue.TeleState.DROP_OZONE;
                    stateTimer.reset();
                }
                break;
            case DROP_OZONE:
                if(extendo.getCurrentPosition() > 300){
                    intakeClaw.setPosition(intakeClawOpenPosTele);
                    telestate = intothedeep_tele_blue.TeleState.OZONE_RETRACTTWO;
                    stateTimer.reset();
                }
                break;
            case OZONE_RETRACTTWO:
                if(stateTimer.seconds() >= INTAKECLAW_TIME){
                    exAddPower = true;
                    slidePositionTargetEx = slideMinEx;
                    intakeWrist.setPosition(intakeWristStraightPos);
                    alpha.setPosition(alphaTransferPos);
                    beta.setPosition(betaTransferPos);
                    telestate = intothedeep_tele_blue.TeleState.INIT;
                }
                break;
            case OUTPUTCLAWSPECIMEN_OPEN:
                if(gamepad1.a && !gamepad1prev.a){
                    outputClaw.setPwmRange(new PwmControl.PwmRange(500,2500));
                    outputClaw.setPosition(outputClawOpenPos);
                    telestate = intothedeep_tele_blue.TeleState.RESET_SPECIMEN;
                    stateTimer.reset();
                }
                break;
            case OUTPUTCLAW_OPEN:
                if((gamepad1.a && !gamepad1prev.a) /*|| (outputMM < maxBucketDist)*/){
                    outputClaw.setPosition(outputClawOpenPos);
                    telestate = intothedeep_tele_blue.TeleState.RESET;
                    stateTimer.reset();
                }
                break;
            case RESET_SPECIMEN:
                if(stateTimer.seconds() >= 0.20){
                    outputWrist.setPosition(outputWristStraightPos);
                    deltaLeft.setPosition(deltaLeftPreTransfer);
                    deltaRight.setPosition(deltaRightPreTransfer);
                    veAddPowerDown = true;
                    slidePositionTargetVe = slideMinVe;
                    specimenOutputState = false;
                    ozoneOutputState = false;
                    sampleOutputState = false;
                    telestate = intothedeep_tele_blue.TeleState.ARM_INTAKE;
                }
                break;
            case RESET:
                if(stateTimer.seconds() >= 0.7 /*&& outputMM > minSlideSafeDist*/){
                    outputWrist.setPosition(outputWristStraightPos);
                    deltaLeft.setPosition(deltaLeftPreTransfer);
                    deltaRight.setPosition(deltaRightPreTransfer);
                    outputClaw.setPosition(outputClawOpenPos);
                    veAddPowerDown = true;
                    slidePositionTargetVe = slideMinVe;
                    specimenOutputState = false;
                    ozoneOutputState = false;
                    sampleOutputState = false;
                    telestate = intothedeep_tele_blue.TeleState.INIT;
                }
                break;
            case HANG_UP1:
                springToggle.setPosition(springToggleOnPos);
                if(stateTimer.seconds() > 0.7) {
                    alpha.setPosition(alphaTransferPos);
                    beta.setPosition(betaTransferPos);
                    veAddPowerUp = true;
                    slidePositionTargetVe = slideHangVe;
                    slidePositionTargetEx = slideMinEx;
                    telestate = intothedeep_tele_blue.TeleState.HANG_DOWN1;
                    stateTimer.reset();
                }
                break;
            case HANG_DOWN1:
                if(veHangPower || ms.getCurrentPosition() > 650.0 && !gamepad2.right_bumper && stateTimer.seconds() > 0.8){
                    veAddPowerUp = false;
                    veConstantPID = -2.0;
                    veHangPower = true;
                    slidePositionTargetVe = -400;
                    slidePositionTargetEx = slideMinEx;
                    exAddPower = true;
                    if(carabinerPressed || ms.getCurrentPosition() < 15.0){
                        telestate = intothedeep_tele_blue.TeleState.CARABINER;
                        stateTimer.reset();
                    }
                }
                break;
            case CARABINER:
                if(!carabinerPressed){
                    slidePositionTargetVe = -400;
                    veHangPower = true;
                    veConstantPID = -2.0;
                    telestate = intothedeep_tele_blue.TeleState.HANG_UP2;
                    stateTimer.reset();
                }
                else {
                    veHangPower = true;
                    slidePositionTargetVe = -400;
                    slidePositionTargetEx = slideMinEx;
                    exAddPower = true;
                }
                break;
            case HANG_UP2:
                if(stateTimer.seconds() > 0.25){
                    veHangPower = false;
                    exAddPower = false;
                    slidePositionTargetVe = 3600;
                    veAddPowerUp = true;
                    veConstantPID = 2.0;
                    if(ms.getCurrentPosition() > 700 && ms.getCurrentPosition() < 1760){
                        exAddPower = false;
                        slidePositionTargetEx = swingSizeEx;
                        alpha.setPosition(0.75);
                        beta.setPosition(0.75);
                        veAddPowerUp = true;
                        veConstantPID = 2.0;
                    }

                    if(ms.getCurrentPosition() > 1780){
                        slidePositionTargetEx = slideMinEx;
                        exAddPower = true;
                        alpha.setPosition(alphaTransferPos);
                        beta.setPosition(betaTransferPos);
                        veAddPowerUp = true;
                        veConstantPID = 2.0;
                        telestate = intothedeep_tele_blue.TeleState.HANG_DOWN2;
                        stateTimer.reset();
                    }
                }
                break;
            case HANG_DOWN2:
                if(gamepad1.dpad_left && gamepad1prev.dpad_left){
                    veHangPower = true;
                    veAddPowerUp = false;
                    veConstantPID = -2.0;
                    slidePositionTargetVe = -400.0;
                }
                if(carabinerPressed){
                    stateTimer.reset();
                    telestate = intothedeep_tele_blue.TeleState.CARABINER2;
                }
                break;

            case CARABINER2:
                if(!carabinerPressed){
                    veConstantPID = -2.0;
                    veHangPower = true;
                    exAddPower = false;
                    slidePositionTargetVe = -400;
                }
                break;
        }

        //hang
        if(gamepad2.right_bumper && !gamepad2prev.right_bumper){
            telestate = intothedeep_tele_blue.TeleState.HANG_UP1;
            stateTimer.reset();
        }

        //failsafes
        if(gamepad2.dpad_right){
            telestate = intothedeep_tele_blue.TeleState.RESET;
        }

        if(Math.abs(gamepad2.left_stick_y) > 0.3){
            exAddPower = gamepad2.left_stick_y > 0;
            slidePositionTargetEx -= slideTargetGainEx * gamepad2.left_stick_y;
        }

        if(Math.abs(gamepad2.right_stick_y) > 0.3){
            veAddPowerDown = gamepad2.right_stick_y > 0;
            slidePositionTargetVe -= slideTargetGainVe * gamepad2.right_stick_y;
        }

        if(gamepad1.dpad_right && !gamepad1prev.dpad_right){
            telestate = intothedeep_tele_blue.TeleState.OUTPUTCLAWSPECIMEN_OPEN;
        }



        //tuning positions
        /*if(gamepad2.dpad_right && !gamepad2prev.dpad_right){
            tuningPos1 += 0.01;
            if(tuningPos1 > 1.0){
                tuningPos1 = 1.0;
            }
        }

        if(gamepad2.dpad_left && !gamepad2prev.dpad_left){
            tuningPos1 -= 0.01;
            if(tuningPos1 < 0.0){
                tuningPos1 = 0.0;
            }
        }*/

        /*if(gamepad2.dpad_up && !gamepad2prev.dpad_up){
            tuningPos2 += 0.01;
            if(tuningPos2 > 1.0){
                tuningPos2 = 1.0;
            }
        }

        if(gamepad2.dpad_down && !gamepad2prev.dpad_down){
            tuningPos2 -= 0.01;
            if(tuningPos2 < 0.0){
                tuningPos2 = 0.0;
            }
        }*/

        //outputClaw.setPosition(tuningPos1);

        if(gamepad1.right_bumper && gamepad1prev.right_bumper){
            intakeClaw.setPosition(intakeClawClosedPos);
        }

        if(gamepad2.left_bumper) {
            setWristPosition();
        }

        if(gamepad1.right_bumper && gamepad1prev.right_bumper){
            getLimelightPower();
        }else{
            //drivetrain
            double y = -gamepad1.left_stick_x; //verticals
            double x = -gamepad1.left_stick_y; //horizontal
            double r = -gamepad1.right_stick_x; //pivot and rotation
            double scalar = 0.95;

            /*if(gamepad1.left_bumper && !gamepad1prev.left_bumper){
                if(turtle){
                    turtle = false;
                }
                else{
                    turtle = true;
                }
            }*/

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
        }

        gamepad1prev.copy(gamepad1);
        gamepad2prev.copy(gamepad2);

        telemetry.addData("ex raw power", exPowerRaw);
        telemetry.addData("ex scaled power", exPowerScaled);
        telemetry.addData("ve constant pid", veConstantPID);
        telemetry.addData("hang boolean power", veHangPower);
        telemetry.addData("vertical power", verticalPower);
        telemetry.addData("tele state", "tele state: " + telestate);
        telemetry.addData("correct color", isCorrectColor(intakeColor));
        telemetry.addData("limelight status", limelight.getStatus());
    }
}