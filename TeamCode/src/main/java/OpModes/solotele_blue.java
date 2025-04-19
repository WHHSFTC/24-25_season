package OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.PwmControl;

@Config
@TeleOp
public class solotele_blue extends intothedeep_opmode{

    public enum TeleState {
        //intake
        ARM_INTAKE,
        INTAKECLAW_CLOSE,
        BRING_IN,
        OUTPUTCLAW_CLOSE,
        INTAKECLAW_OPEN,

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

    TeleState telestate = TeleState.INIT;
    RevColorSensorV3 intakeColor;

    double slideTargetGainEx = 110;
    double slideTargetGainVe = 30;

    @Override
    public void init(){
        super.init();

        intakeColor = hardwareMap.get(RevColorSensorV3.class, "colorsensor");
    }

    @Override
    public void start(){
        super.start();
        intakeClaw.setPosition(intakeClawOpenPos);
        intakeWrist.setPosition(intakeWristStraightPos);
        alpha.setPosition(alphaTransferPos);
        beta.setPosition(betaTransferPos);
        outputClaw.setPosition(outputClawOpenPos);
        outputWrist.setPosition(outputWristStraightPos);
        deltaLeft.setPosition(deltaLeftPreTransfer);
        deltaRight.setPosition(deltaRightPreTransfer);
        springToggle.setPosition(springToggleOffPos);
        blueAlliance = true;
    }

    public boolean isCorrectColor(ColorSensor sensorOutput) {
        if(intakeClaw.getPosition() < intakeClawOpenPos + 0.02 || (gamepad2.dpad_left && gamepad2prev.dpad_left)){
            return true;
        }
        else{
            if(blueAlliance) {
                if(specimenOutputState || ozoneOutputState){
                    return sensorOutput.blue() > 0.6 * colorThreshold && sensorOutput.red() < colorThreshold * 1.5 &&
                            sensorOutput.green() < colorThreshold * 1.5;
                }
                if(sampleOutputState){
                    return (sensorOutput.blue() > colorThreshold || sensorOutput.green() > colorThreshold)
                            && (sensorOutput.red() < sensorOutput.green() || sensorOutput.red() < sensorOutput.blue());
                }
            }
            else {
                if (specimenOutputState || ozoneOutputState) {
                    return sensorOutput.blue() < 0.6 * colorThreshold
                            && sensorOutput.red() > colorThreshold * 1.5 && sensorOutput.green() < colorThreshold * 1.5;
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

        if(!gamepad1.dpad_left){
            verticalPower = 2.0*slidesPidVertical.calculatePowerVertical(slidePositionTargetVe) + veConstantPID;
            ls.setPower(verticalPower);
            ms.setPower(verticalPower);
            rs.setPower(verticalPower);
            slidesPidVertical.updateVe((ms.getCurrentPosition()), timeGap);
        }

        switch(telestate){
            case INIT:
                veAddPowerDown = false;
                exAddPower = false;

                if((gamepad1.x && !gamepad1prev.x) || (gamepad1.circle && !gamepad1prev.circle)){
                    intakeWristPos = intakeWristStraightPos;
                    slidePositionTargetEx = slideMaxEx;
                    alpha.setPosition(alphaIntakePos);
                    beta.setPosition(betaIntakePos);
                    telestate = TeleState.ARM_INTAKE;
                    intakeClaw.setPosition(intakeClawOpenPos);
                    deltaLeft.setPosition(deltaLeftPreTransfer);
                    deltaRight.setPosition(deltaRightPreTransfer);
                    ozoneOutputState = false;
                    specimenOutputState = false;
                    sampleOutputState = false;
                }

                if(gamepad1.right_bumper && !gamepad1prev.right_bumper){
                    telestate = TeleState.HANG_UP1;
                    stateTimer.reset();
                }

                break;
            case ARM_INTAKE:
                if(gamepad1.circle && !gamepad1prev.circle){
                    alpha.setPosition(alphaLowerPos);
                    beta.setPosition(betaLowerPos);
                    telestate = TeleState.INTAKECLAW_CLOSE;
                    ozoneOutputState = false;
                    specimenOutputState = true;
                    sampleOutputState = false;
                    stateTimer.reset();
                }

                if(gamepad1.x && !gamepad1prev.x){
                    alpha.setPosition(alphaLowerPos);
                    beta.setPosition(betaLowerPos);
                    telestate = TeleState.INTAKECLAW_CLOSE;
                    sampleOutputState = true;
                    specimenOutputState = false;
                    ozoneOutputState = false;
                    stateTimer.reset();
                }

                intakeWristPos = Math.min(Math.max(
                        intakeWristPos + (gamepad1.left_trigger - gamepad1.right_trigger) * 0.02,
                        intakeWristRightLimit), intakeWristLeftLimit);

                intakeWrist.setPosition(intakeWristPos);

                break;
            case INTAKECLAW_CLOSE:
                if(stateTimer.seconds() >= INTAKELOWER_TIME){
                    intakeClaw.setPosition(intakeClawClosedPos);
                    telestate = TeleState.BRING_IN;
                    stateTimer.reset();
                }
                break;
            case BRING_IN:
                veAddPowerDown = false;
                if(stateTimer.seconds() >= INTAKECLAW_TIME){
                    if(isCorrectColor(intakeColor)) {
                        exAddPower = true;
                        slidePositionTargetEx = slideMinEx;
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
                if (extendoSlidesLimit.isPressed() && gamepad1.circle) {
                    specimenOutputState = false;
                    ozoneOutputState = true;
                    telestate = TeleState.OZONE_EXTEND;
                    stateTimer.reset();
                } else {
                    if(!isCorrectColor(intakeColor)){
                        exAddPower = false;
                        slidePositionTargetEx = slideMaxEx;
                        alpha.setPosition(alphaIntakePos);
                        beta.setPosition(betaIntakePos);
                        intakeClaw.setPosition(intakeClawOpenPos);
                        intakeWristPos = intakeWristStraightPos;
                        intakeWrist.setPosition(intakeWristStraightPos);
                        telestate = TeleState.ARM_INTAKE;
                    } else if(extendoSlidesLimit.isPressed()) {
                        exAddPower = false;
                        deltaLeft.setPosition(deltaLeftTransferPos);
                        deltaRight.setPosition(deltaRightTransferPos);
                        telestate = TeleState.OUTPUTCLAW_CLOSE;
                        stateTimer.reset();
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
                if(specimenOutputState && stateTimer.seconds() > 0.15){
                    telestate = TeleState.OUTPUT_SPECIMEN;
                }
                if(sampleOutputState && stateTimer.seconds() > 0.15){
                    telestate = TeleState.OUTPUT_BASKET;
                }
                if(ozoneOutputState && stateTimer.seconds() > 0.15){
                    telestate = TeleState.OZONE_RETRACT;
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
                veAddPowerDown = false;
                slidePositionTargetVe = 650;
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
                    telestate = TeleState.SPECIMEN_OUTPUT;
                }
                break;
            case SPECIMEN_OUTPUT:
                if(300 > 200/*chamberDS.getDistance(DistanceUnit.MM) < maxChamberDist*/){ //TODO: replace with matbotix
                    outputWrist.setPosition(0.10);
                    slidePositionTargetVe += 200;
                    gamepad1.rumble(500);
                    outputClaw.setPwmRange(new PwmControl.PwmRange(500,1000));
                    telestate = TeleState.OUTPUTCLAWSPECIMEN_OPEN;
                }
                break;
            case OZONE_RETRACT: // this state may be obsolete :(
                if(stateTimer.seconds() >= INTAKECLAW_TIME){
                    exAddPower = true;
                    slidePositionTargetEx = slideMinEx;
                    intakeWrist.setPosition(intakeWristStraightPos);
                    telestate = TeleState.OZONE_EXTEND;
                }
                break;
            case OZONE_EXTEND:
                if(!gamepad1.circle){
                    exAddPower = false;
                    slidePositionTargetEx = slideMaxEx;
                    alpha.setPosition(alphaIntakePos);
                    beta.setPosition(betaIntakePos);
                    telestate = TeleState.DROP_OZONE;
                    stateTimer.reset();
                }
                break;
            case DROP_OZONE:
                if(extendo.getCurrentPosition() > 350){
                    intakeClaw.setPosition(intakeClawOpenPos);
                    telestate = TeleState.OZONE_RETRACTTWO;
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
                    telestate = TeleState.INIT;
                }
                break;
            case OUTPUTCLAWSPECIMEN_OPEN:
                if(stateTimer.seconds() > 0.5){
                    outputClaw.setPwmRange(new PwmControl.PwmRange(500,2500));
                    outputClaw.setPosition(outputClawOpenPos);
                    telestate = TeleState.RESET_SPECIMEN;
                    stateTimer.reset();
                }
                break;
            case OUTPUTCLAW_OPEN:
                if(gamepad2.a && !gamepad2prev.a /*|| outputDS.getDistance(DistanceUnit.MM) < maxBucketDist*/){ // TODO: replace with maxbotix
                    outputClaw.setPosition(outputClawOpenPos);
                    gamepad1.rumble(500);
                    telestate = TeleState.RESET;
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
                    telestate = TeleState.ARM_INTAKE;
                }
                break;
            case RESET:
                if(stateTimer.seconds() >= 1.2 /*&& outputDS.getDistance(DistanceUnit.MM) > minSlideSafeDist*/){
                    outputWrist.setPosition(outputWristStraightPos);
                    deltaLeft.setPosition(deltaLeftPreTransfer);
                    deltaRight.setPosition(deltaRightPreTransfer);
                    veAddPowerDown = true;
                    slidePositionTargetVe = slideMinVe;
                    specimenOutputState = false;
                    ozoneOutputState = false;
                    sampleOutputState = false;
                    slidePositionTargetEx = slideMinEx;
                    intakeClaw.setPosition(intakeClawOpenPos);
                    alpha.setPosition(alphaTransferPos);
                    beta.setPosition(betaTransferPos);
                    springToggle.setPosition(springToggleOffPos);
                    intakeWrist.setPosition(intakeWristStraightPos);
                    outputClaw.setPosition(outputClawOpenPos);

                    telestate = TeleState.INIT;
                }
                break;
            case HANG_UP1:
                springToggle.setPosition(springToggleOnPos);
                if(stateTimer.seconds() > 0.4) {
                    alpha.setPosition(alphaTransferPos);
                    beta.setPosition(betaTransferPos);
                    slidePositionTargetVe = slideHangVe;
                    slidePositionTargetEx = slideMinEx;
                    telestate = TeleState.HANG_DOWN1;
                    stateTimer.reset();
                }
                break;
            case HANG_DOWN1:
                if(veHangPower || ms.getCurrentPosition() > 750.0 && !gamepad1.right_bumper && stateTimer.seconds() > 0.8){
                    veHangPower = true;
                    slidePositionTargetVe = -400;
                    slidePositionTargetEx = slideMinEx;
                    exAddPower = true;
                    if(carabinerPressed || ms.getCurrentPosition() < 25.0){
                        telestate = TeleState.CARABINER;
                        stateTimer.reset();
                    }
                }
                break;
            case CARABINER:
                if(!carabinerPressed){
                    slidePositionTargetVe = -400;
                    veHangPower = true;
                    telestate = TeleState.HANG_UP2;
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
                    slidePositionTargetVe = 3200;
                    veAddPowerUp = true;
                    if(ms.getCurrentPosition() > 700 && ms.getCurrentPosition() < 1760){
                        exAddPower = false;
                        slidePositionTargetEx = swingSizeEx;
                        alpha.setPosition(0.75);
                        beta.setPosition(0.75);
                    }

                    if(ms.getCurrentPosition() > 1770){
                        slidePositionTargetEx = slideMinEx;
                        exAddPower = true;
                        alpha.setPosition(alphaTransferPos);
                        beta.setPosition(betaTransferPos);
                        veAddPowerUp = false;
                        telestate = TeleState.HANG_DOWN2;
                        stateTimer.reset();
                    }
                }
                break;
            case HANG_DOWN2:
                if(gamepad1.right_bumper){
                    rs.setPower(-1.0);
                    ls.setPower(-1.0);
                    ms.setPower(-1.0);
                    slidePositionTargetVe = 0.0;
                } else {
                    veHangPower = false;
                    exAddPower = false;
                    slidePositionTargetVe = 3200;
                    veAddPowerUp = true;
                }
                if(carabinerPressed){
                    stateTimer.reset();
                    telestate = TeleState.CARABINER2;
                }
                break;
            case CARABINER2:
                if(!carabinerPressed){
                    veHangPower = true;
                    exAddPower = false;
                    slidePositionTargetVe = -400;
                }
                break;
        }

        if((gamepad1.right_bumper && !gamepad1prev.right_bumper)){
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

        //failsafes
        if(gamepad1.left_bumper && !gamepad1prev.left_bumper){
            telestate = TeleState.RESET;
        }

        if(gamepad2.dpad_down && gamepad2prev.dpad_down){
            extendo.setPower(-1.0);
        }
        else{
            extendo.setPower(2.4*slidesPidExtendo.calculatePowerExtendo(slidePositionTargetEx) + exConstantPID);
            slidesPidExtendo.updateEx(extendo.getCurrentPosition(), timeGap);
        }

        if(Math.abs(gamepad2.left_stick_y) > 0.3){
            if(gamepad2.left_stick_y > 0){
                exAddPower = true;
            }else{
                exAddPower = false;
            }
            slidePositionTargetEx -= slideTargetGainEx * gamepad2.left_stick_y;
        }

        if(Math.abs(gamepad2.right_stick_y) > 0.3){
            if(gamepad2.right_stick_y > 0){
                veAddPowerDown = true;
            }else{
                veAddPowerDown = false;
            }
            slidePositionTargetVe -= slideTargetGainVe * gamepad2.right_stick_y;
        }

        if(gamepad1.dpad_right && !gamepad1prev.dpad_right){
            telestate = TeleState.OUTPUTCLAWSPECIMEN_OPEN;
        }



        //tuning positions
        if(gamepad2.dpad_right && !gamepad2prev.dpad_right){
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
        }

        if(gamepad2.dpad_up && !gamepad2prev.dpad_up){
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
        }


        deltaLeft.setPosition(tuningPos1);
        deltaRight.setPosition(tuningPos2);

        //drivetrain
        double y = -gamepad1.left_stick_x; //verticals
        double x = -gamepad1.left_stick_y; //horizontal
        double r = -gamepad1.right_stick_x; //pivot and rotation
        double scalarT = 0.95;
        double scalarR = 0.95;
        boolean invertTurtle = false;

        scalarT = gamepad1.left_stick_button^invertTurtle ? 0.25 : 0.95; // turtle
        scalarR = gamepad1.right_stick_button^invertTurtle ? 0.25 : 0.95; // turtle

        double preRF = r*scalarR + y*scalarT + x*scalarT;
        double preLF = r*scalarR + y*scalarT - x*scalarT;
        double preRB = r*scalarR - y*scalarT + x*scalarT;
        double preLB = r*scalarR - y*scalarT - x*scalarT;

        double max = Math.max(Math.max(Math.max(Math.max(preRF, preRB), preLB), preLF), 1);

        rf.setPower(preRF/max);
        lf.setPower(preLF/max);
        rb.setPower(preRB/max);
        lb.setPower(preLB/max);

        gamepad1prev.copy(gamepad1);
        gamepad2prev.copy(gamepad2);

        telemetry.addData("tele state", "tele state: " + telestate);
        telemetry.addData("correct color", isCorrectColor(intakeColor));
    }
}
