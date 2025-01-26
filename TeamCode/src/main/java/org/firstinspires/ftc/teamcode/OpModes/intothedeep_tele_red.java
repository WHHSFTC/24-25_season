package org.firstinspires.ftc.teamcode.OpModes;

import static com.qualcomm.hardware.lynx.LynxModule.DebugGroup.I2C;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
        blueAlliance = false;
    }

    public boolean isCorrectColor(ColorSensor sensorOutput) {
        if(gamepad2.dpad_left && gamepad2prev.dpad_left){
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

        if(gamepad2.dpad_down && gamepad2prev.dpad_down){
            extendo.setPower(-1.0);
        }else{
            extendo.setPower(2.4*slidesPidExtendo.calculatePowerExtendo(slidePositionTargetEx) + exConstantPID);
            slidesPidExtendo.updateEx(extendo.getCurrentPosition(), timeGap);
        }


        verticalPower = 2.0*slidesPidVertical.calculatePowerVertical(slidePositionTargetVe) + veConstantPID;
        ls.setPower(verticalPower);
        ms.setPower(verticalPower);
        rs.setPower(verticalPower);
        slidesPidVertical.updateVe((ms.getCurrentPosition()), timeGap);

        if(exAddPower){
            exConstantPID = -0.7;
        }else{
            exConstantPID = 0.0;
        }

        if(veAddPowerDown){
            veConstantPID = -0.4;
        }else{
            veConstantPID = 0.0;
        }

        if(veHangPower){
            veConstantPID = 0.5;
        }

        switch(telestate){
            case INIT:
                veAddPowerDown = false;
                exAddPower = false;
                if(gamepad2.a && !gamepad2prev.a){
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
                break;
            case ARM_INTAKE:
                if(gamepad2.b && !gamepad2prev.b){
                    alpha.setPosition(alphaLowerPos);
                    beta.setPosition(betaLowerPos);
                    telestate = TeleState.INTAKECLAW_CLOSE;
                    ozoneOutputState = true;
                    specimenOutputState = false;
                    sampleOutputState = false;
                    stateTimer.reset();
                }

                if(gamepad2.x && !gamepad2prev.x){
                    alpha.setPosition(alphaLowerPos);
                    beta.setPosition(betaLowerPos);
                    telestate = TeleState.INTAKECLAW_CLOSE;
                    specimenOutputState = true;
                    sampleOutputState = false;
                    ozoneOutputState = false;
                    stateTimer.reset();
                }

                if(gamepad2.y && !gamepad2prev.y){
                    alpha.setPosition(alphaLowerPos);
                    beta.setPosition(betaLowerPos);
                    telestate = TeleState.INTAKECLAW_CLOSE;
                    sampleOutputState = true;
                    specimenOutputState = false;
                    ozoneOutputState = false;
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
                veAddPowerDown = false;
                if(stateTimer.seconds() >= INTAKECLAW_TIME){
                    if(isCorrectColor(intakeColor)) {
                        if(ozoneOutputState){
                            intakeWrist.setPosition(intakeWristStraightPos);
                            alpha.setPosition(alphaTransferPos);
                            beta.setPosition(betaTransferPos);
                            telestate = TeleState.OZONE_RETRACT;
                            stateTimer.reset();
                        }else{
                            exAddPower = true;
                            slidePositionTargetEx = slideMinEx;
                            intakeWrist.setPosition(intakeWristStraightPos);
                            alpha.setPosition(alphaTransferPos);
                            beta.setPosition(betaTransferPos);
                            telestate = TeleState.OUTPUTARM_TRANSFER;
                            stateTimer.reset();
                        }
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
                if(extendoSlidesLimit.isPressed()){
                    if(isCorrectColor(intakeColor)){
                        exAddPower = false;
                        deltaLeft.setPosition(deltaLeftTransferPos);
                        deltaRight.setPosition(deltaRightTransferPos);
                        telestate = TeleState.OUTPUTCLAW_CLOSE;
                        stateTimer.reset();
                    }
                    else{
                        exAddPower = false;
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
                if(chamberDS.getDistance(DistanceUnit.MM) < maxChamberDist){
                    outputWrist.setPosition(0.10);
                    slidePositionTargetVe += 200;
                    outputClaw.setPwmRange(new PwmControl.PwmRange(500,1000));
                    telestate = TeleState.OUTPUTCLAWSPECIMEN_OPEN;
                }
                break;
            case OZONE_RETRACT:
                if(stateTimer.seconds() >= INTAKECLAW_TIME){
                    exAddPower = true;
                    slidePositionTargetEx = slideMinEx;
                    intakeWrist.setPosition(intakeWristStraightPos);
                    telestate = TeleState.OZONE_EXTEND;
                }
                break;
            case OZONE_EXTEND:
                if(!gamepad2.b && gamepad2prev.b){
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
                if(gamepad1.a && !gamepad1prev.a){
                    outputClaw.setPwmRange(new PwmControl.PwmRange(500,2500));
                    outputClaw.setPosition(outputClawOpenPos);
                    telestate = TeleState.RESET_SPECIMEN;
                    stateTimer.reset();
                }
                break;
            case OUTPUTCLAW_OPEN:
                if(gamepad1.a && !gamepad1prev.a /*|| outputDS.getDistance(DistanceUnit.MM) < maxBucketDist*/){
                    outputClaw.setPosition(outputClawOpenPos);
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
                    telestate = TeleState.INIT;
                }
                break;
            case HANG_UP1:
                springToggle.setPosition(springToggleOffPos);
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
                if(veHangPower || ms.getCurrentPosition() > 750.0 && !gamepad1.dpad_up && stateTimer.seconds() > 0.8){
                    veHangPower = true;
                    slidePositionTargetVe = 0;
                    slidePositionTargetEx = slideMinEx;
                    exAddPower = true;
                    if(carabinerPressed || verticalSlidesLimit.isPressed()){
                        telestate = TeleState.CARABINER;
                        stateTimer.reset();
                    }
                }
                break;
            case CARABINER:
                if(!carabinerPressed){
                    veHangPower = false;
                    exAddPower = false;
                    slidePositionTargetEx = swingSizeEx;
                    alpha.setPosition(alphaLowerPos);
                    beta.setPosition(betaLowerPos);
                    telestate = TeleState.HANG_UP2;
                    stateTimer.reset();
                }
                break;
            case HANG_UP2:
                if(extendo.getCurrentPosition() > swingSizeEx - 15 && stateTimer.seconds() > 1.00){
                    exAddPower = false;
                    slidePositionTargetVe = slideMaxVe;
                    telestate = TeleState.HANG_DOWN2;
                    stateTimer.reset();
                }
                break;
            case HANG_DOWN2:
                if(ms.getCurrentPosition() > slideMaxVe - 30){
                    slidePositionTargetEx = slideMinEx;
                    exAddPower = true;
                    alpha.setPosition(alphaTransferPos);
                    beta.setPosition(betaTransferPos);
                }
                if(extendo.getCurrentPosition() < slideMinEx + 15){
                    veHangPower = true;
                    slidePositionTargetVe = slideMinVe;
                }
                if(carabinerPressed){
                    stateTimer.reset();
                    telestate = TeleState.CARABINER2;
                }
                break;
            case CARABINER2:
                if(!carabinerPressed){
                    if(stateTimer.seconds() > 0.50){
                        veHangPower = false;
                        exAddPower = false;
                        slidePositionTargetVe = 400;
                    }
                }
        }

        //intake wrist
        if(gamepad2.left_trigger > 0.0){
            intakeWristPos += 0.01;
            if(intakeWristPos < intakeWristLeftLimit){
                intakeWristPos = intakeWristLeftLimit;
            }
            intakeWrist.setPosition(intakeWristPos);
        }

        if(gamepad2.right_trigger > 0.0){
            intakeWristPos -= 0.01;
            if(intakeWristPos > intakeWristRightLimit){
                intakeWristPos = intakeWristRightLimit;
            }
            intakeWrist.setPosition(intakeWristPos);
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

        //hang
        if(gamepad1.dpad_up && !gamepad1prev.dpad_up){
            telestate = TeleState.HANG_UP1;
            stateTimer.reset();
        }

        //failsafes
        if(gamepad2.dpad_right && !gamepad2prev.dpad_right){
            telestate = TeleState.INIT;
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


        intakeWrist.setPosition(tuningPos1);*/

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
        telemetry.addData("correct color", isCorrectColor(intakeColor));
    }
}
