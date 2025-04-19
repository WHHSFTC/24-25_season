package OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.pedropathing.localization.Pose;
@Config
@Autonomous (preselectTeleOp = "intothedeep_tele_blue")
public class fivespec_blue extends intothedeep_auto{
//
    private final Pose startPos = new Pose(7.1791, 55.0, Math.toRadians(180));
    private final Pose controlSample1Pos = new Pose(17.32, 31.79, Math.toRadians(180));
    private final Pose controlSample2Pos = new Pose(18.98, 38.90, Math.toRadians(180));
    private final Pose controlSample3Pos = new Pose(30.37, 40.57, Math.toRadians(180));
    private final Pose controlSample4Pos = new Pose(55.04, 33.21, Math.toRadians(180));
    private final Pose sample1Pos = new Pose(57, 27.65, Math.toRadians(180));
    private final Pose pushSample1 = new Pose(25.0, 27.65, Math.toRadians(180));
    private final Pose sample2ControlPos1 = new Pose(61.92, 27.52, Math.toRadians(180));
    private final Pose sample2ControlPos2 = new Pose(66.90, 21.35, Math.toRadians(180));
    private final Pose sample2ControlPos3 = new Pose(68.32, 21.83, Math.toRadians(180));
    private final Pose sample2Pos = new Pose(56.7, 18.6, Math.toRadians(180));
    private final Pose pushSample2 = new Pose(24.0, 18.6, Math.toRadians(180));
    private final Pose sample3ControlPos1 = new Pose(67.14, 18.27, Math.toRadians(180));
    private final Pose sample3ControlPos2 = new Pose(59.78, 14.47, Math.toRadians(180));
    private final Pose sample3ControlPos3 = new Pose(66.42, 13.05, Math.toRadians(180));
    private final Pose sample3Pos = new Pose(56.7, 13.0, Math.toRadians(180));
    private final Pose pushSample3 = new Pose(24.5, 13.0, Math.toRadians(180));
    private final Pose scoreSpec1Control1 = new Pose(20.64, 38.91, Math.toRadians(225));
    private final Pose scoreSpec1Control2 = new Pose(15.42, 53.14, Math.toRadians(225));
    private final Pose scoreSpec1Control3 = new Pose(19.93, 65.48, Math.toRadians(225));
    private final Pose intakePos = new Pose(25.0, 54.0, Math.toRadians(225));
    private final Pose scoreSpec1Pos = new Pose(34.0, 71.0, Math.toRadians(225));
    private final Pose scoreSpec2Pos = new Pose(34.5, 70, Math.toRadians(225));
    private final Pose scoreSpec3Pos = new Pose(34.5, 69, Math.toRadians(225));
    private final Pose scoreSpec4Pos = new Pose(34.5, 68, Math.toRadians(225));
    private final Pose scoreSpec5Pos = new Pose(34.5, 67, Math.toRadians(225));
    private final Pose parkPos = new Pose(27.5, 52, Math.toRadians(225));

    private Path scorePreload, park;
    private PathChain transition, pushOne, getToTwo,
            pushTwo, getToThree, pushThree,
            intakeSpec1, intakeSpec2, intakeSpec3,
            intakeSpec4, scoreSpec1, scoreSpec2, scoreSpec3,
            scoreSpec4, scoreSpec5, intakeSpec5;

    public void buildPaths(){
        transition = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                        new Point(startPos),
                        new Point(controlSample1Pos),
                        new Point(controlSample2Pos),
                        new Point(controlSample3Pos),
                        new Point(controlSample4Pos),
                        new Point(sample1Pos)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        pushOne = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                        new Point(sample1Pos),
                        new Point(pushSample1)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(9.0)
                .build();

        getToTwo = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(pushSample1),
                                new Point(sample2ControlPos1),
                                new Point(sample2ControlPos2),
                                new Point(sample2ControlPos3),
                                new Point(sample2Pos)))
                .setZeroPowerAccelerationMultiplier(7.0)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        pushTwo = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(sample2Pos),
                        new Point(pushSample2)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(9.0)
                .build();

        getToThree = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(pushSample2),
                        new Point(sample3ControlPos1),
                        new Point(sample3ControlPos2),
                        new Point(sample3ControlPos3),
                        new Point(sample3Pos)))
                .setZeroPowerAccelerationMultiplier(7.0)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        pushThree = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(sample3Pos),
                        new Point(pushSample3)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(9.0)
                .build();

        scoreSpec1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(pushSample3),
                        new Point(scoreSpec1Control1),
                        new Point(scoreSpec1Control2),
                        new Point(scoreSpec1Control3),
                        new Point(scoreSpec1Pos)))
                .setLinearHeadingInterpolation(pushSample3.getHeading(), scoreSpec1Pos.getHeading())
                .setZeroPowerAccelerationMultiplier(9.0)
                .build();

        intakeSpec2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(scoreSpec1Pos),
                        new Point(intakePos)))
                .setConstantHeadingInterpolation(Math.toRadians(225))
                .build();

        scoreSpec2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(intakePos),
                        new Point(scoreSpec2Pos)))
                .setConstantHeadingInterpolation(Math.toRadians(225))
                .build();

        intakeSpec3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(scoreSpec2Pos),
                        new Point(intakePos)))
                .setConstantHeadingInterpolation(Math.toRadians(225))
                .build();

        scoreSpec3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(intakePos),
                        new Point(scoreSpec3Pos)))
                .setConstantHeadingInterpolation(Math.toRadians(225))
                .build();

        intakeSpec4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(scoreSpec3Pos),
                        new Point(intakePos)))
                .setConstantHeadingInterpolation(Math.toRadians(225))
                .build();

        scoreSpec4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(intakePos),
                        new Point(scoreSpec4Pos)))
                .setConstantHeadingInterpolation(Math.toRadians(225))
                .build();

        intakeSpec5 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(scoreSpec4Pos),
                        new Point(intakePos)))
                .setConstantHeadingInterpolation(Math.toRadians(225))
                .build();

        scoreSpec5 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(intakePos),
                        new Point(scoreSpec5Pos)))
                .setConstantHeadingInterpolation(Math.toRadians(225))
                .build();

        park = new Path(new BezierLine(
                new Point(scoreSpec5Pos),
                new Point(intakePos)));
        park.setConstantHeadingInterpolation(Math.toRadians(240));
    }

    @Override
    public void autonomousPathUpdate(){
        switch (pathState){
            case 0:
                follower.followPath(transition);
                setPathState(1);
                break;

            case 1:
                if(follower.getPose().getY() < 26){
                    follower.followPath(pushOne);
                    setPathState(2);
                }
                break;

            case 2:
                if(follower.getPose().getX() < 26){
                    follower.setMaxPower(0.9);
                    follower.setCentripetalScaling(0.012);
                    follower.followPath(getToTwo);
                    setPathState(3);
                }
                break;

            case 3:
                if(follower.getPose().getY() < 19.2){
                    follower.followPath(pushTwo);
                    setPathState(4);
                }
                break;

            case 4:
                if(follower.getPose().getX() < 25){
                    follower.setCentripetalScaling(0.0018);
                    follower.setMaxPower(1.0);
                    follower.followPath(getToThree);
                    setPathState(5);
                }
                break;

            case 5:
                if(follower.getPose().getY() < 13.6){
                    alpha.setPosition(alphaIntakePos);
                    beta.setPosition(betaIntakePos);
                    intakeClaw.setPosition(intakeClawOpenPos);
                    follower.followPath(pushThree);
                    slidePositionTargetEx = slideMinEx;
                    setPathState(6);
                }
                break;

            case 6:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.5){
                    alpha.setPosition(alphaLowerPos);
                    beta.setPosition(betaLowerPos);
                    intakeWrist.setPosition(intakeWristStraightPos);
                    setPathState(7);
                }
                break;

            case 7:
                if(pathTimer.getElapsedTimeSeconds() > INTAKELOWER_TIME){
                    intakeClaw.setPosition(intakeClawClosedPos);
                    setPathState(8);
                }
                break;

            case 8:
                if(pathTimer.getElapsedTimeSeconds() > INTAKECLAW_TIME){
                    alpha.setPosition(alphaTransferPos);
                    beta.setPosition(betaTransferPos);
                    exConstantPID = -1.5;
                    slidePositionTargetEx = slideMinEx;
                    setPathState(9);
                }
                break;

            case 9:
                if(extendoSlidesLimit.isPressed() || pathTimer.getElapsedTimeSeconds() > 0.6){
                    exConstantPID = -1.5;
                    slidePositionTargetEx = slideMinEx;
                    deltaLeft.setPosition(deltaLeftTransferPos);
                    deltaRight.setPosition(deltaRightTransferPos);
                    follower.followPath(scoreSpec1);
                    setPathState(10);
                }
                break;

            case 10:
                if(pathTimer.getElapsedTimeSeconds() > OUTPUTARM_READY - 0.04){
                    outputClaw.setPosition(outputClawClosedPos);
                    exConstantPID = -1.5;
                    slidePositionTargetEx = slideMinEx;
                    setPathState(11);
                }
                break;

            case 11:
                if(pathTimer.getElapsedTimeSeconds() > OUTPUTCLAW_TIME - 0.10){
                    intakeClaw.setPosition(intakeClawOpenPos);
                    exConstantPID = -1.5;
                    slidePositionTargetEx = slideMinEx;
                    setPathState(12);
                }
                break;

            case 12:
                if(pathTimer.getElapsedTimeSeconds() > 0.10){
                    exAddPower = false;
                    exConstantPID = 0.0;
                    slidePositionTargetVe = 365;
                    deltaLeft.setPosition(deltaLeftSpecimenPos);
                    deltaRight.setPosition(deltaRightSpecimenPos);
                    outputWrist.setPosition(outputWristSpecimenPos);
                    setPathState(13);
                }
                break;

            case 13:
                if(follower.getPose().getX() > 33.5){
                    slidePositionTargetVe = 875;
                    outputWrist.setPosition(0.15);
                    setPathState(14);
                }
                break;

            case 14:
                if(ms.getCurrentPosition() > 650){
                    follower.setMaxPower(0.7);
                    follower.followPath(intakeSpec2);
                    setPathState(15);
                }
                break;

            case 15:
                if(pathTimer.getElapsedTimeSeconds() > 0.9){
                    slidePositionTargetVe = slideMinVe;
                    deltaLeft.setPosition(deltaLeftPreTransfer);
                    deltaRight.setPosition(deltaRightPreTransfer);
                    outputWrist.setPosition(outputWristStraightPos);
                    slidePositionTargetEx = slideMaxEx;
                    alpha.setPosition(alphaIntakePos);
                    beta.setPosition(betaIntakePos);
                }

                if(ms.getCurrentPosition() < 50){
                    outputClaw.setPosition(outputClawOpenPos);
                    setPathState(16);
                }
                break;

            case 16:
                if(!follower.isBusy()){
                    alpha.setPosition(alphaLowerPos);
                    beta.setPosition(betaLowerPos);
                    setPathState(17);
                }
                break;

            case 17:
                if(pathTimer.getElapsedTimeSeconds() >= INTAKELOWER_TIME){
                    intakeClaw.setPosition(intakeClawClosedPos);
                    outputClaw.setPosition(outputClawOpenPos);
                    setPathState(18);
                }
                break;

            case 18:
                if(pathTimer.getElapsedTimeSeconds() >= INTAKECLAW_TIME){
                    alpha.setPosition(alphaTransferPos);
                    beta.setPosition(betaTransferPos);
                    intakeWrist.setPosition(intakeWristStraightPos);
                    slidePositionTargetEx = slideMinEx;
                    follower.followPath(scoreSpec2);
                    exAddPower = true;
                    exConstantPID = -1.5;

                    setPathState(19);
                }
                break;

            case 19:
                if(extendoSlidesLimit.isPressed()){
                    exAddPower = true;
                    exConstantPID = -1.5;
                    deltaLeft.setPosition(deltaLeftTransferPos);
                    deltaRight.setPosition(deltaRightTransferPos);
                    setPathState(20);
                }
                break;

            case 20:
                if(pathTimer.getElapsedTimeSeconds() >= OUTPUTARM_READY - 0.04){
                    exAddPower = true;
                    exConstantPID = -1.5;
                    outputClaw.setPosition(outputClawClosedPos);
                    setPathState(21);
                }
                break;

            case 21:
                if(pathTimer.getElapsedTimeSeconds() >= OUTPUTCLAW_TIME - 0.10){
                    intakeClaw.setPosition(intakeClawOpenPos);
                    exAddPower = true;
                    exConstantPID = -1.5;
                    follower.setMaxPower(0.70);
                    setPathState(22);
                }
                break;

            case 22:
                if(pathTimer.getElapsedTimeSeconds() > 0.15){
                    exAddPower = false;
                    exConstantPID = 0.0;
                    slidePositionTargetVe = 365;
                    deltaLeft.setPosition(deltaLeftSpecimenPos);
                    deltaRight.setPosition(deltaRightSpecimenPos);
                    outputWrist.setPosition(outputWristSpecimenPos);
                    setPathState(23);
                }
                break;

            case 23:
                if(follower.getPose().getX() > 34.4){
                    slidePositionTargetVe = 875;
                    outputWrist.setPosition(0.15);
                    setPathState(24);
                }
                break;

            case 24:
                if(ms.getCurrentPosition() > 650){
                    follower.followPath(intakeSpec3);
                    setPathState(25);
                }
                break;

            case 25:
                if(pathTimer.getElapsedTimeSeconds() > 0.9){
                    slidePositionTargetVe = slideMinVe;
                    deltaLeft.setPosition(deltaLeftPreTransfer);
                    deltaRight.setPosition(deltaRightPreTransfer);
                    outputWrist.setPosition(outputWristStraightPos);
                    slidePositionTargetEx = slideMaxEx;
                    alpha.setPosition(alphaIntakePos);
                    beta.setPosition(betaIntakePos);
                }

                if(ms.getCurrentPosition() < 50){
                    outputClaw.setPosition(outputClawOpenPos);
                    setPathState(26);
                }
                break;

            case 26:
                if(!follower.isBusy()){
                    alpha.setPosition(alphaLowerPos);
                    beta.setPosition(betaLowerPos);
                    setPathState(27);
                }
                break;

            case 27:
                if(pathTimer.getElapsedTimeSeconds() >= INTAKELOWER_TIME){
                    intakeClaw.setPosition(intakeClawClosedPos);
                    outputClaw.setPosition(outputClawOpenPos);
                    setPathState(28);
                }
                break;

            case 28:
                if(pathTimer.getElapsedTimeSeconds() >= INTAKECLAW_TIME){
                    alpha.setPosition(alphaTransferPos);
                    beta.setPosition(betaTransferPos);
                    intakeWrist.setPosition(intakeWristStraightPos);
                    slidePositionTargetEx = slideMinEx;
                    exAddPower = true;
                    exConstantPID = -1.5;

                    follower.setMaxPower(0.7);
                    follower.followPath(scoreSpec3);
                    setPathState(29);
                }
                break;

            case 29:
                if(extendoSlidesLimit.isPressed()){
                    exAddPower = true;
                    exConstantPID = -1.5;
                    deltaLeft.setPosition(deltaLeftTransferPos);
                    deltaRight.setPosition(deltaRightTransferPos);
                    setPathState(30);
                }
                break;

            case 30:
                if(pathTimer.getElapsedTimeSeconds() >= OUTPUTARM_READY - 0.04){
                    exAddPower = true;
                    exConstantPID = -1.5;
                    outputClaw.setPosition(outputClawClosedPos);
                    setPathState(31);
                }
                break;

            case 31:
                if(pathTimer.getElapsedTimeSeconds() >= OUTPUTCLAW_TIME - 0.10){
                    exAddPower = true;
                    exConstantPID = -1.5;
                    intakeClaw.setPosition(intakeClawOpenPos);
                    setPathState(32);
                }
                break;

            case 32:
                if(pathTimer.getElapsedTimeSeconds() > 0.15){
                    exAddPower = false;
                    exConstantPID = 0.0;
                    slidePositionTargetVe = 365;
                    deltaLeft.setPosition(deltaLeftSpecimenPos);
                    deltaRight.setPosition(deltaRightSpecimenPos);
                    outputWrist.setPosition(outputWristSpecimenPos);
                    setPathState(33);
                }
                break;

            case 33:
                if(follower.getPose().getX() > 34.4){
                    slidePositionTargetVe = 875;
                    outputWrist.setPosition(0.15);
                    setPathState(34);
                }
                break;

            case 34:
                if(ms.getCurrentPosition() > 650){
                    follower.setMaxPower(0.70);
                    follower.followPath(intakeSpec4);
                    setPathState(35);
                }
                break;
            case 35:
                if(pathTimer.getElapsedTimeSeconds() > 0.9){
                    slidePositionTargetVe = slideMinVe;
                    deltaLeft.setPosition(deltaLeftPreTransfer);
                    deltaRight.setPosition(deltaRightPreTransfer);
                    outputWrist.setPosition(outputWristStraightPos);
                    slidePositionTargetEx = slideMaxEx;
                    alpha.setPosition(alphaIntakePos);
                    beta.setPosition(betaIntakePos);
                }

                if(ms.getCurrentPosition() < 50){
                    outputClaw.setPosition(outputClawOpenPos);
                    setPathState(36);
                }
                break;

            case 36:
                if(!follower.isBusy()){
                    alpha.setPosition(alphaLowerPos);
                    beta.setPosition(betaLowerPos);
                    setPathState(37);
                }
                break;

            case 37:
                if(pathTimer.getElapsedTimeSeconds() >= INTAKELOWER_TIME){
                    intakeClaw.setPosition(intakeClawClosedPos);
                    outputClaw.setPosition(outputClawOpenPos);
                    setPathState(38);
                }
                break;

            case 38:
                if(pathTimer.getElapsedTimeSeconds() >= INTAKECLAW_TIME){
                    alpha.setPosition(alphaTransferPos);
                    beta.setPosition(betaTransferPos);
                    intakeWrist.setPosition(intakeWristStraightPos);
                    slidePositionTargetEx = slideMinEx;
                    follower.setMaxPower(0.70);
                    follower.followPath(scoreSpec4);
                    exAddPower = true;
                    exConstantPID = -1.5;
                    setPathState(39);
                }
                break;

            case 39:
                if(extendoSlidesLimit.isPressed()){
                    exAddPower = true;
                    exConstantPID = -1.5;
                    deltaLeft.setPosition(deltaLeftTransferPos);
                    deltaRight.setPosition(deltaRightTransferPos);
                    setPathState(40);
                }
                break;

            case 40:
                if(pathTimer.getElapsedTimeSeconds() >= OUTPUTARM_READY - 0.04){
                    exAddPower = true;
                    exConstantPID = -1.5;
                    outputClaw.setPosition(outputClawClosedPos);
                    setPathState(41);
                }
                break;

            case 41:
                if(pathTimer.getElapsedTimeSeconds() >= OUTPUTCLAW_TIME - 0.10){
                    exAddPower = true;
                    exConstantPID = -1.5;
                    intakeClaw.setPosition(intakeClawOpenPos);
                    setPathState(42);
                }
                break;

            case 42:
                if(pathTimer.getElapsedTimeSeconds() > 0.15){
                    exAddPower = false;
                    exConstantPID = 0.0;
                    slidePositionTargetVe = 365;
                    deltaLeft.setPosition(deltaLeftSpecimenPos);
                    deltaRight.setPosition(deltaRightSpecimenPos);
                    outputWrist.setPosition(outputWristSpecimenPos);
                    setPathState(43);
                }
                break;

            case 43:
                if(follower.getPose().getX() > 34.4){
                    slidePositionTargetVe = 875;
                    outputWrist.setPosition(0.15);
                    setPathState(44);
                }
                break;

            case 44:
                if(ms.getCurrentPosition() > 650){
                    follower.followPath(intakeSpec5);
                    setPathState(45);
                }
                break;
            case 45:
                if(pathTimer.getElapsedTimeSeconds() > 0.9){
                    slidePositionTargetVe = slideMinVe;
                    deltaLeft.setPosition(deltaLeftPreTransfer);
                    deltaRight.setPosition(deltaRightPreTransfer);
                    outputWrist.setPosition(outputWristStraightPos);
                    slidePositionTargetEx = slideMaxEx;
                    alpha.setPosition(alphaIntakePos);
                    beta.setPosition(betaIntakePos);
                }

                if(ms.getCurrentPosition() < 50){
                    outputClaw.setPosition(outputClawOpenPos);
                    setPathState(46);
                }
                break;

            case 46:
                if(!follower.isBusy()){
                    alpha.setPosition(alphaLowerPos);
                    beta.setPosition(betaLowerPos);
                    setPathState(47);
                }
                break;

            case 47:
                if(pathTimer.getElapsedTimeSeconds() >= INTAKELOWER_TIME){
                    intakeClaw.setPosition(intakeClawClosedPos);
                    outputClaw.setPosition(outputClawOpenPos);
                    setPathState(48);
                }
                break;

            case 48:
                if(pathTimer.getElapsedTimeSeconds() >= INTAKECLAW_TIME){
                    alpha.setPosition(alphaTransferPos);
                    beta.setPosition(betaTransferPos);
                    intakeWrist.setPosition(intakeWristStraightPos);
                    slidePositionTargetEx = slideMinEx;
                    exAddPower = true;
                    exConstantPID = -1.5;

                    follower.setMaxPower(0.7);
                    follower.followPath(scoreSpec5);
                    setPathState(49);
                }
                break;

            case 49:
                if(extendoSlidesLimit.isPressed()){
                    exAddPower = true;
                    exConstantPID = -1.5;
                    deltaLeft.setPosition(deltaLeftTransferPos);
                    deltaRight.setPosition(deltaRightTransferPos);
                    setPathState(50);
                }
                break;

            case 50:
                if(pathTimer.getElapsedTimeSeconds() >= OUTPUTARM_READY - 0.04){
                    exAddPower = true;
                    exConstantPID = -1.5;
                    outputClaw.setPosition(outputClawClosedPos);
                    setPathState(51);
                }
                break;

            case 51:
                if(pathTimer.getElapsedTimeSeconds() >= OUTPUTCLAW_TIME - 0.10){
                    exAddPower = true;
                    exConstantPID = -1.5;
                    intakeClaw.setPosition(intakeClawOpenPos);
                    setPathState(52);
                }
                break;

            case 52:
                if(pathTimer.getElapsedTimeSeconds() > 0.15){
                    exAddPower = false;
                    exConstantPID = 0.0;
                    slidePositionTargetVe = 365;
                    deltaLeft.setPosition(deltaLeftSpecimenPos);
                    deltaRight.setPosition(deltaRightSpecimenPos);
                    outputWrist.setPosition(outputWristSpecimenPos);
                    setPathState(53);
                }
                break;

            case 53:
                if(follower.getPose().getX() > 34.4){
                    slidePositionTargetVe = 875;
                    outputWrist.setPosition(0.15);
                    setPathState(54);
                }
                break;

            case 54:
                if(ms.getCurrentPosition() > 650){
                    follower.setMaxPower(0.70);
                    follower.followPath(park);
                    slidePositionTargetEx = slideMaxEx;
                    alpha.setPosition(alphaIntakePos);
                    beta.setPosition(betaIntakePos);
                    setPathState(-1);
                }
                break;

            /*case 0:
                follower.followPath(scorePreload);
                slidePositionTargetVe = 500;
                setPathState(1);
                break;

            case 1:
                if(!follower.isBusy() && ms.getCurrentPosition() > 400){
                    slidePositionTargetVe = 700;
                    outputWrist.setPosition(0.10);
                    setPathState(2);
                }
                break;

            case 2:
                if(ms.getCurrentPosition() > 650){
                    follower.followPath(grabSpec1);
                    setPathState(3);
                }
            case 3:
                if(follower.getPose().getX() < 30){
                    outputClaw.setPosition(outputClawOpenPos);
                }

                if(pathTimer.getElapsedTimeSeconds() > 0.4){
                    slidePositionTargetVe = slideMinVe;
                    deltaLeft.setPosition(deltaLeftPreTransfer);
                    deltaRight.setPosition(deltaRightPreTransfer);
                    outputWrist.setPosition(outputWristStraightPos);
                }

                if(ms.getCurrentPosition() < 30){
                    setPathState(4);
                }
                break;

            case 4:
                if(!follower.isBusy()){
                    slidePositionTargetEx = slideMaxEx;
                    intakeWrist.setPosition(0.32);
                    alpha.setPosition(alphaIntakePos);
                    beta.setPosition(betaIntakePos);
                    setPathState(5);
                }
                break;

            case 5:
                if(pathTimer.getElapsedTimeSeconds() > 0.4){
                    alpha.setPosition(alphaLowerPos);
                    beta.setPosition(betaLowerPos);
                    setPathState(6);
                }

            case 6:
                if(pathTimer.getElapsedTimeSeconds() >= 0.5){
                    intakeClaw.setPosition(intakeClawClosedPos);
                    setPathState(7);
                }

                break;

            case 7:
                if(pathTimer.getElapsedTimeSeconds() >= INTAKECLAW_TIME){
                    alpha.setPosition(alphaIntakePos);
                    beta.setPosition(betaIntakePos);
                    follower.followPath(turnSpec1);
                    setPathState(8);
                }
                break;

            case 8:
                if(!follower.isBusy()){
                    intakeClaw.setPosition(intakeClawOpenPos);
                    setPathState(9);
                }
                break;

            case 9:
                if(pathTimer.getElapsedTimeSeconds() > 0.4){
                    slidePositionTargetEx = slideMinEx;
                    exAddPower = true;
                    setPathState(10);
                }
                break;

            case 10:
                if(extendo.getCurrentPosition() < 30){
                    follower.followPath(grabSpec2);
                    setPathState(11);
                }
                break;

            case 11:
                if(!follower.isBusy()){
                    slidePositionTargetEx = slideMaxEx;
                    setPathState(12);
                }
            case 12:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5){
                    alpha.setPosition(alphaLowerPos);
                    beta.setPosition(betaLowerPos);
                    setPathState(13);
                }
                break;

            case 13:
                if(pathTimer.getElapsedTimeSeconds() >= 0.5){
                    intakeClaw.setPosition(intakeClawClosedPos);
                    setPathState(14);
                }
                break;

            case 14:
                if(pathTimer.getElapsedTimeSeconds() >= INTAKECLAW_TIME){
                    alpha.setPosition(alphaIntakePos);
                    beta.setPosition(betaIntakePos);
                    follower.followPath(turnSpec2);
                    setPathState(15);
                }
                break;

            case 15:
                if(!follower.isBusy()){
                    intakeClaw.setPosition(intakeClawOpenPos);
                    setPathState(16);
                }
                break;

            case 16:
                if(pathTimer.getElapsedTimeSeconds() > 0.3){
                    slidePositionTargetEx = slideMinEx;
                    intakeWrist.setPosition(intakeWristStraightPos);
                    exAddPower = true;
                    follower.followPath(intakeSpec1);
                    setPathState(17);
                }
                break;

            case 17:
                if(!follower.isBusy() && extendo.getCurrentPosition() < 60){
                    exAddPower = false;
                    slidePositionTargetEx = slideMaxEx;
                    alpha.setPosition(alphaIntakePos);
                    beta.setPosition(betaIntakePos);
                    setPathState(18);
                }
                break;

            case 18:
                if(pathTimer.getElapsedTimeSeconds() > 0.5){
                    alpha.setPosition(alphaLowerPos);
                    beta.setPosition(betaLowerPos);
                    setPathState(19);
                }
                break;

            case 19:
                if(pathTimer.getElapsedTimeSeconds() >= 0.5){
                    intakeClaw.setPosition(intakeClawClosedPos);
                    setPathState(20);
                }
                break;

            case 20:
                if(pathTimer.getElapsedTimeSeconds() >= INTAKECLAW_TIME){
                    alpha.setPosition(alphaTransferPos);
                    beta.setPosition(betaTransferPos);
                    intakeWrist.setPosition(intakeWristStraightPos);
                    slidePositionTargetEx = slideMinEx;
                    exAddPower = true;
                    follower.followPath(scoreSpec1);
                    setPathState(21);
                }
                break;

            case 21:
                if(extendoSlidesLimit.isPressed()){
                    exAddPower = false;
                    deltaLeft.setPosition(deltaLeftTransferPos);
                    deltaRight.setPosition(deltaRightTransferPos);
                    setPathState(22);
                }
                break;

            case 22:
                if(pathTimer.getElapsedTimeSeconds() >= OUTPUTARM_READY){
                    outputClaw.setPosition(outputClawClosedPos);
                    setPathState(23);
                }
                break;

            case 23:
                if(pathTimer.getElapsedTimeSeconds() >= OUTPUTCLAW_TIME){
                    intakeClaw.setPosition(intakeClawOpenPos);
                    setPathState(24);
                }
                break;

            case 24:
                slidePositionTargetVe = 300;
                deltaLeft.setPosition(deltaLeftSpecimenPos);
                deltaRight.setPosition(deltaRightSpecimenPos);
                outputWrist.setPosition(outputWristSpecimenPos);
                setPathState(25);
                break;

            case 25:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.0){
                    slidePositionTargetVe = 700;
                    outputWrist.setPosition(0.10);
                    setPathState(26);
                }
                break;

            case 26:
                if(ms.getCurrentPosition() > 650){
                    follower.followPath(intakeSpec2);
                    setPathState(27);
                }
                break;

            case 27:
                if(follower.getPose().getX() < 30){
                    outputClaw.setPosition(outputClawOpenPos);
                }

                if(pathTimer.getElapsedTimeSeconds() > 0.2){
                    slidePositionTargetVe = slideMinVe;
                    deltaLeft.setPosition(deltaLeftPreTransfer);
                    deltaRight.setPosition(deltaRightPreTransfer);
                    outputWrist.setPosition(outputWristStraightPos);
                    slidePositionTargetEx = slideMaxEx;
                    alpha.setPosition(alphaIntakePos);
                    beta.setPosition(betaIntakePos);
                }

                if(extendo.getCurrentPosition() > 300){
                    setPathState(28);
                }
                break;

            //spec2
            case 28:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.3){
                    alpha.setPosition(alphaLowerPos);
                    beta.setPosition(betaLowerPos);
                    setPathState(29);
                }
                break;

            case 29:
                if(pathTimer.getElapsedTimeSeconds() >= 0.4){
                    intakeClaw.setPosition(intakeClawClosedPos);
                    setPathState(30);
                }
                break;

            case 30:
                if(pathTimer.getElapsedTimeSeconds() >= 0.4){
                    alpha.setPosition(alphaTransferPos);
                    beta.setPosition(betaTransferPos);
                    intakeWrist.setPosition(intakeWristStraightPos);
                    slidePositionTargetEx = slideMinEx;
                    exAddPower = true;
                    follower.followPath(scoreSpec2);
                    setPathState(31);
                }
                break;

            case 31:
                if(extendoSlidesLimit.isPressed()){
                    exAddPower = false;
                    deltaLeft.setPosition(deltaLeftTransferPos);
                    deltaRight.setPosition(deltaRightTransferPos);
                    setPathState(32);
                }
                break;

            case 32:
                if(pathTimer.getElapsedTimeSeconds() >= OUTPUTARM_READY){
                    outputClaw.setPosition(outputClawClosedPos);
                    setPathState(33);
                }
                break;

            case 33:
                if(pathTimer.getElapsedTimeSeconds() >= OUTPUTCLAW_TIME){
                    intakeClaw.setPosition(intakeClawOpenPos);
                    setPathState(34);
                }
                break;

            case 34:
                slidePositionTargetVe = 300;
                deltaLeft.setPosition(deltaLeftSpecimenPos);
                deltaRight.setPosition(deltaRightSpecimenPos);
                outputWrist.setPosition(outputWristSpecimenPos);
                setPathState(35);
                break;

            case 35:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.0){
                    slidePositionTargetVe = 700;
                    outputWrist.setPosition(0.10);
                    setPathState(36);
                }
                break;

            case 36:
                if(ms.getCurrentPosition() > 650){
                    follower.followPath(intakeSpec3);
                    setPathState(37);
                }
                break;

            case 37:
                if(follower.getPose().getX() < 30){
                    outputClaw.setPosition(outputClawOpenPos);
                }

                if(pathTimer.getElapsedTimeSeconds() > 0.4){
                    slidePositionTargetVe = slideMinVe;
                    deltaLeft.setPosition(deltaLeftPreTransfer);
                    deltaRight.setPosition(deltaRightPreTransfer);
                    outputWrist.setPosition(outputWristStraightPos);
                    slidePositionTargetEx = slideMaxEx;
                    alpha.setPosition(alphaIntakePos);
                    beta.setPosition(betaIntakePos);
                }

                if(extendo.getCurrentPosition() > 300){
                    setPathState(38);
                }
                break;

            case 38:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.3){
                    alpha.setPosition(alphaLowerPos);
                    beta.setPosition(betaLowerPos);
                    setPathState(39);
                }
                break;

            case 39:
                if(pathTimer.getElapsedTimeSeconds() >= 0.4){
                    intakeClaw.setPosition(intakeClawClosedPos);
                    setPathState(40);
                }
                break;

            case 40:
                if(pathTimer.getElapsedTimeSeconds() >= 0.4){
                    alpha.setPosition(alphaTransferPos);
                    beta.setPosition(betaTransferPos);
                    intakeWrist.setPosition(intakeWristStraightPos);
                    slidePositionTargetEx = slideMinEx;
                    exAddPower = true;
                    follower.followPath(scoreSpec3);
                    setPathState(41);
                }
                break;

            case 41:
                if(extendoSlidesLimit.isPressed()){
                    exAddPower = false;
                    deltaLeft.setPosition(deltaLeftTransferPos);
                    deltaRight.setPosition(deltaRightTransferPos);
                    setPathState(42);
                }
                break;

            case 42:
                if(pathTimer.getElapsedTimeSeconds() >= OUTPUTARM_READY){
                    outputClaw.setPosition(outputClawClosedPos);
                    setPathState(43);
                }
                break;

            case 43:
                if(pathTimer.getElapsedTimeSeconds() >= OUTPUTCLAW_TIME){
                    intakeClaw.setPosition(intakeClawOpenPos);
                    setPathState(44);
                }
                break;

            case 44:
                slidePositionTargetVe = 300;
                deltaLeft.setPosition(deltaLeftSpecimenPos);
                deltaRight.setPosition(deltaRightSpecimenPos);
                outputWrist.setPosition(outputWristSpecimenPos);
                setPathState(45);
                break;

            case 45:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.0){
                    slidePositionTargetVe = 700;
                    outputWrist.setPosition(0.10);
                    setPathState(46);
                }
                break;

            case 46:
                if(ms.getCurrentPosition() > 650){
                    follower.followPath(park);
                    setPathState(47);
                }
                break;
            case 47:
                if(follower.getPose().getX() < 30){
                    outputClaw.setPosition(outputClawOpenPos);
                }

                if(pathTimer.getElapsedTimeSeconds() > 0.4){
                    slidePositionTargetVe = slideMinVe;
                    deltaLeft.setPosition(deltaLeftPreTransfer);
                    deltaRight.setPosition(deltaRightPreTransfer);
                    outputWrist.setPosition(outputWristStraightPos);
                    slidePositionTargetEx = slideMaxEx;
                    alpha.setPosition(alphaIntakePos);
                    beta.setPosition(betaIntakePos);
                }

                if(extendo.getCurrentPosition() > 300){
                    setPathState(-1);
                }
                break;*/
        }

    }

    @Override
    public void init(){
        super.init();
        follower.setStartingPose(startPos);
        buildPaths();
        deltaLeft.setPosition(deltaLeftPreTransfer);
        deltaRight.setPosition(deltaRightPreTransfer);
        outputWrist.setPosition(outputWristStraightPos);
        outputClaw.setPosition(outputClawOpenPos);

        intakeClaw.setPosition(intakeClawOpenPos);
        intakeWrist.setPosition(intakeWristStraightPos);
        alpha.setPosition(alphaTransferPos);
        beta.setPosition(betaTransferPos);
    }
}
