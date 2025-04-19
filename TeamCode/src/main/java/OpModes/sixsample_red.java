package OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous (preselectTeleOp = "intothedeep_tele_red")
public class sixsample_red extends intothedeep_auto{
    private final Pose startPos = new Pose(7.21, 103.1791, Math.toRadians(270));
    private final Pose scorePos = new Pose(13.4, 127.5, Math.toRadians(315)); //13.5, 126.3
    private final Pose preloadControlPos = new Pose(27.52, 115.77, Math.toRadians(315));
    private final Pose sample1Pos = new Pose(17.3, 124.5, Math.toRadians(350));
    private final Pose sample2Pos = new Pose(16.2, 127.5, Math.toRadians(5));
    private final Pose sample3Pos = new Pose(17.5, 131.2, Math.toRadians(18));
    private final Pose parkControlPos = new Pose(55, 135, Math.toRadians(90));
    private final Pose subControlPos1 = new Pose(59.55, 120.28, Math.toRadians(290));
    private final Pose subControlPos2 = new Pose(59.55, 120.28, Math.toRadians(315));
    private final Pose subPos = new Pose(60, 100, Math.toRadians(270));
    private final Pose parkPos = new Pose(60, 90, Math.toRadians(90));

    private Path scorePreload, park;
    private PathChain grabSample1, grabSample2, grabSample3, sub1, sub2, scoreSample1, scoreSample2, scoreSample3, scoreSub1, scoreSub2;


    public void buildPaths(){
        scorePreload = new Path(new BezierCurve(new Point(startPos), new Point(preloadControlPos), new Point(scorePos)));
        scorePreload.setLinearHeadingInterpolation(startPos.getHeading(), scorePos.getHeading());

        grabSample1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePos), new Point(sample1Pos)))
                .setLinearHeadingInterpolation(scorePos.getHeading(), sample1Pos.getHeading())
                .build();

        scoreSample1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample1Pos), new Point(scorePos)))
                .setLinearHeadingInterpolation(sample1Pos.getHeading(), scorePos.getHeading())
                .build();

        grabSample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePos), new Point(sample2Pos)))
                .setLinearHeadingInterpolation(scorePos.getHeading(), sample2Pos.getHeading())
                .build();

        scoreSample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample2Pos), new Point(scorePos)))
                .setLinearHeadingInterpolation(sample2Pos.getHeading(), scorePos.getHeading())
                .build();

        grabSample3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePos), new Point(sample3Pos)))
                .setLinearHeadingInterpolation(scorePos.getHeading(), sample3Pos.getHeading())
                .build();

        scoreSample3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample3Pos), new Point(scorePos)))
                .setLinearHeadingInterpolation(sample3Pos.getHeading(), scorePos.getHeading())
                .build();

        sub1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePos), new Point(subControlPos1), new Point(subPos)))
                .setLinearHeadingInterpolation(scorePos.getHeading(), subPos.getHeading())
                .build();

        sub2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePos), new Point(subControlPos1), new Point(subPos)))
                .setLinearHeadingInterpolation(scorePos.getHeading(), subPos.getHeading())
                .build();

        scoreSub1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(subPos), new Point(scorePos)))
                .setLinearHeadingInterpolation(subPos.getHeading(), scorePos.getHeading(), 0.0)
                .build();

        scoreSub2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(subPos), new Point(subControlPos2), new Point(scorePos)))
                .setLinearHeadingInterpolation(subPos.getHeading(), scorePos.getHeading())
                .build();

        park = new Path(new BezierCurve(new Point(scorePos), new Point(parkControlPos), new Point(parkPos)));
        park.setLinearHeadingInterpolation(scorePos.getHeading(), parkPos.getHeading());
    }

    @Override
    public void autonomousPathUpdate(){
        switch (pathState){
            case 0:
                follower.setMaxPower(0.85);
                follower.followPath(scorePreload);
                slidePositionTargetVe = slideMaxVe + 30;
                outputWrist.setPosition(outputWristSwitchPos);
                deltaRight.setPosition(deltaRightSamplePos);
                deltaLeft.setPosition(deltaLeftSamplePos);
                setPathState(1);
                break;

            case 1:
                if(pathTimer.getElapsedTimeSeconds() >= 1.9 && ms.getCurrentPosition() > 1750){
                    outputClaw.setPosition(outputClawOpenPos);
                    slidePositionTargetEx = slideMaxEx;
                    alpha.setPosition(alphaIntakePos);
                    beta.setPosition(betaIntakePos);
                    setPathState(2);
                }
                break;

            case 2:
                if(pathTimer.getElapsedTimeSeconds() > 0.2){
                    follower.setMaxPower(0.5);
                    follower.followPath(grabSample1);
                    setPathState(3);
                }
                break;

            case 3:
                if(extendo.getCurrentPosition() > 300 && !follower.isBusy()){
                    slidePositionTargetVe = slideMinVe;
                    deltaRight.setPosition(deltaRightPreTransfer);
                    deltaLeft.setPosition(deltaLeftPreTransfer);
                    outputWrist.setPosition(outputWristStraightPos);

                    alpha.setPosition(alphaLowerPos);
                    beta.setPosition(betaLowerPos);
                    setPathState(4);
                }
                break;

            case 4:
                if(pathTimer.getElapsedTimeSeconds() >= INTAKELOWER_TIME - 0.05){
                    intakeClaw.setPosition(intakeClawClosedPos);
                    setPathState(5);
                }
                break;

            case 5:
                if(pathTimer.getElapsedTimeSeconds() >= 0.50){
                    follower.followPath(scoreSample1);
                    alpha.setPosition(alphaTransferPos);
                    beta.setPosition(betaTransferPos);
                    intakeWrist.setPosition(intakeWristStraightPos);
                    slidePositionTargetEx = slideMinEx;
                    exAddPower = true;
                    exConstantPID = -1.5;
                    setPathState(6);
                }
                break;

            case 6:
                if(extendoSlidesLimit.isPressed() && ms.getCurrentPosition() < 30){
                    exAddPower = false;
                    deltaLeft.setPosition(deltaLeftTransferPos);
                    deltaRight.setPosition(deltaRightTransferPos);
                    exAddPower = true;
                    exConstantPID = -1.5;
                    setPathState(7);
                }
                break;

            case 7:
                if(pathTimer.getElapsedTimeSeconds() >= OUTPUTARM_READY){
                    outputClaw.setPosition(outputClawClosedPos);
                    exAddPower = true;
                    exConstantPID = -1.5;
                    setPathState(8);
                }
                break;

            case 8:
                if(pathTimer.getElapsedTimeSeconds() >= OUTPUTCLAW_TIME + 0.10){
                    intakeClaw.setPosition(intakeClawOpenPos);
                    exAddPower = true;
                    exConstantPID = -1.5;
                    setPathState(9);
                }
                break;

            case 9:
                if(pathTimer.getElapsedTimeSeconds() > 0.2){
                    slidePositionTargetVe = slideMaxVe;
                    exAddPower = false;
                    exConstantPID = 0.0;
                    outputWrist.setPosition(outputWristSwitchPos);
                    setPathState(10);
                }
                break;

            case 10:
                if(ms.getCurrentPosition() > 1400){
                    deltaLeft.setPosition(deltaLeftSamplePos);
                    deltaRight.setPosition(deltaRightSamplePos);
                    setPathState(11);
                }
                break;

            case 11:
                if(!follower.isBusy() && ms.getCurrentPosition() > 1750 && pathTimer.getElapsedTimeSeconds() > 1){
                    outputClaw.setPosition(outputClawOpenPos);
                    slidePositionTargetEx = slideMaxEx;
                    alpha.setPosition(alphaIntakePos);
                    beta.setPosition(betaIntakePos);
                    setPathState(12);
                }
                break;


            case 12:
                if(pathTimer.getElapsedTimeSeconds() > 0.3){
                    follower.setMaxPower(0.5);
                    follower.followPath(grabSample2);
                    setPathState(13);
                }
                break;


            case 13:
                if(extendo.getCurrentPosition() > 300 && !follower.isBusy()){
                    slidePositionTargetVe = slideMinVe;
                    deltaRight.setPosition(deltaRightPreTransfer);
                    deltaLeft.setPosition(deltaLeftPreTransfer);
                    outputWrist.setPosition(outputWristStraightPos);
                    alpha.setPosition(alphaLowerPos);
                    beta.setPosition(betaLowerPos);
                    setPathState(14);
                }
                break;

            case 14:
                if(pathTimer.getElapsedTimeSeconds() >= INTAKELOWER_TIME - 0.05){
                    intakeClaw.setPosition(intakeClawClosedPos);
                    setPathState(15);
                }
                break;

            case 15:
                if(pathTimer.getElapsedTimeSeconds() >= 0.50){
                    alpha.setPosition(alphaTransferPos);
                    beta.setPosition(betaTransferPos);
                    intakeWrist.setPosition(intakeWristStraightPos);
                    slidePositionTargetEx = slideMinEx;
                    exAddPower = true;
                    exConstantPID = -1.5;
                    follower.followPath(scoreSample2);
                    setPathState(16);
                }
                break;

            case 16:
                if(extendoSlidesLimit.isPressed() && ms.getCurrentPosition() < 30){
                    exAddPower = false;
                    deltaLeft.setPosition(deltaLeftTransferPos);
                    deltaRight.setPosition(deltaRightTransferPos);
                    exAddPower = true;
                    exConstantPID = -1.5;
                    setPathState(17);
                }
                break;

            case 17:
                if(pathTimer.getElapsedTimeSeconds() >= OUTPUTARM_READY){
                    outputClaw.setPosition(outputClawClosedPos);
                    exAddPower = true;
                    exConstantPID = -1.5;
                    setPathState(18);
                }
                break;

            case 18:
                if(pathTimer.getElapsedTimeSeconds() >= OUTPUTCLAW_TIME + 0.10){
                    intakeClaw.setPosition(intakeClawOpenPos);
                    exAddPower = true;
                    exConstantPID = -1.5;
                    setPathState(19);
                }
                break;

            case 19:
                if(pathTimer.getElapsedTimeSeconds() > 0.2){
                    slidePositionTargetVe = slideMaxVe;
                    outputWrist.setPosition(outputWristSwitchPos);
                    exAddPower = false;
                    exConstantPID = 0.0;
                    setPathState(20);
                }
                break;

            case 20:
                if(ms.getCurrentPosition() > 1400){
                    deltaLeft.setPosition(deltaLeftSamplePos);
                    deltaRight.setPosition(deltaRightSamplePos);
                    setPathState(21);
                }
                break;
            case 21:
                if(!follower.isBusy() && ms.getCurrentPosition() > 1750 && pathTimer.getElapsedTimeSeconds() > 1){
                    outputClaw.setPosition(outputClawOpenPos);
                    slidePositionTargetEx = slideMaxEx;
                    alpha.setPosition(alphaIntakePos);
                    beta.setPosition(betaIntakePos);
                    setPathState(22);
                }
                break;

            case 22:
                if(pathTimer.getElapsedTimeSeconds() > 0.3){
                    follower.setMaxPower(0.7);
                    follower.followPath(grabSample3);
                    setPathState(23);
                }
                break;

            case 23:
                if(extendo.getCurrentPosition() > 300 && !follower.isBusy()){
                    slidePositionTargetVe = slideMinVe;
                    deltaRight.setPosition(deltaRightPreTransfer);
                    deltaLeft.setPosition(deltaLeftPreTransfer);
                    outputWrist.setPosition(outputWristStraightPos);

                    alpha.setPosition(alphaLowerPos);
                    beta.setPosition(betaLowerPos);
                    setPathState(24);
                }
                break;

            case 24:
                if(pathTimer.getElapsedTimeSeconds() >= INTAKELOWER_TIME - 0.05){
                    intakeClaw.setPosition(intakeClawClosedPos);
                    setPathState(25);
                }
                break;

            case 25:
                if(pathTimer.getElapsedTimeSeconds() >= 0.50){
                    alpha.setPosition(alphaTransferPos);
                    beta.setPosition(betaTransferPos);
                    intakeWrist.setPosition(intakeWristStraightPos);
                    slidePositionTargetEx = slideMinEx;
                    exAddPower = true;
                    exConstantPID = -1.5;
                    follower.followPath(scoreSample3);
                    setPathState(26);
                }
                break;

            case 26:
                if(extendoSlidesLimit.isPressed() && ms.getCurrentPosition() < 30){
                    exAddPower = false;
                    deltaLeft.setPosition(deltaLeftTransferPos);
                    deltaRight.setPosition(deltaRightTransferPos);
                    exAddPower = true;
                    exConstantPID = -1.5;
                    setPathState(27);
                }
                break;

            case 27:
                if(pathTimer.getElapsedTimeSeconds() >= OUTPUTARM_READY){
                    outputClaw.setPosition(outputClawClosedPos);
                    exAddPower = true;
                    exConstantPID = -1.5;
                    setPathState(28);
                }
                break;

            case 28:
                if(pathTimer.getElapsedTimeSeconds() >= OUTPUTCLAW_TIME + 0.10){
                    intakeClaw.setPosition(intakeClawOpenPos);
                    exAddPower = true;
                    exConstantPID = -1.5;
                    setPathState(29);
                }
                break;

            case 29:
                if(pathTimer.getElapsedTimeSeconds() > 0.2){
                    slidePositionTargetVe = slideMaxVe;
                    outputWrist.setPosition(outputWristSwitchPos);
                    exAddPower = false;
                    exConstantPID = 0.0;
                    setPathState(30);
                }
                break;

            case 30:
                if(ms.getCurrentPosition() > 1400){
                    deltaLeft.setPosition(deltaLeftSamplePos);
                    deltaRight.setPosition(deltaRightSamplePos);
                    setPathState(31);
                }
                break;

            case 31:
                if(!follower.isBusy() && ms.getCurrentPosition() > 1750 && pathTimer.getElapsedTimeSeconds() > 1){
                    outputClaw.setPosition(outputClawOpenPos);
                    setPathState(32);
                }
                break;

            case 32:
                follower.followPath(park);
                setPathState(33);
                break;

            case 33:
                if(pathTimer.getElapsedTimeSeconds() >= 0.6){
                    deltaLeft.setPosition(deltaLeftPreTransfer);
                    deltaRight.setPosition(deltaRightPreTransfer);
                    outputWrist.setPosition(outputWristStraightPos);
                    slidePositionTargetVe = 1000;
                    setPathState(-1);
                }
                break;
            /*case 33:
                if(pathTimer.getElapsedTimeSeconds() >= 0.6){
                    deltaLeft.setPosition(deltaLeftPreTransfer);
                    deltaRight.setPosition(deltaRightPreTransfer);
                    outputWrist.setPosition(outputWristStraightPos);
                    slidePositionTargetVe = 0.0;
                    setPathState(34);
                }
                break;

            case 34:
                if(pathTimer.getElapsedTimeSeconds() >= 1.0){
                    slidePositionTargetEx = slideMaxEx;
                    alpha.setPosition(alphaIntakePos);
                    beta.setPosition(betaIntakePos);
                    intakeWrist.setPosition(intakeWristStraightPos);
                    setPathState(35);
                }
                break;
            case 35:
                if(!follower.isBusy()){
                    while(true) {
                        if(getLimelightPower()) break;
                    }
                    setPathState(36);
                }
                break;
            case 36:
                follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading()));
                    alpha.setPosition(alphaLowerPos);
                    beta.setPosition(betaLowerPos);
                    setPathState(37);
                break;
            case 37:
                if(pathTimer.getElapsedTimeSeconds() > INTAKELOWER_TIME){
                    intakeClaw.setPosition(intakeClawClosedPos);
                    setPathState(38);
                }
                break;
            case 38:
                if(pathTimer.getElapsedTimeSeconds() > 0.50){
                    alpha.setPosition(alphaTransferPos);
                    beta.setPosition(betaTransferPos);
                    intakeWrist.setPosition(intakeWristStraightPos);
                    slidePositionTargetEx = slideMinEx;
                    exAddPower = true;
                    exConstantPID = -1.5;
                    follower.followPath(scoreSub1);
                    setPathState(39);
                }
                break;
            case 39:
                if(extendoSlidesLimit.isPressed()){
                    deltaLeft.setPosition(deltaLeftTransferPos);
                    deltaRight.setPosition(deltaRightTransferPos);
                    outputClaw.setPosition(outputClawOpenPos);
                    exAddPower = true;
                    exConstantPID = -1.5;
                    setPathState(40);
                }
                break;
            case 40:
                if(pathTimer.getElapsedTimeSeconds() > OUTPUTARM_READY){
                    outputClaw.setPosition(outputClawClosedPos);
                    exAddPower = true;
                    exConstantPID = -1.5;
                    setPathState(41);
                }
                break;
            case 41:
                if(pathTimer.getElapsedTimeSeconds() > OUTPUTCLAW_TIME){
                    intakeClaw.setPosition(intakeClawOpenPos);
                    exAddPower = true;
                    exConstantPID = -1.5;
                    setPathState(42);
                }
                break;
            case 42:
                if(pathTimer.getElapsedTimeSeconds() > 0.2){
                    slidePositionTargetVe = slideMaxVe;
                    deltaLeft.setPosition(deltaLeftSamplePos);
                    deltaRight.setPosition(deltaRightSamplePos);
                    outputWrist.setPosition(outputWristSwitchPos);
                    exAddPower = false;
                    exConstantPID = 0.0;
                    setPathState(43);
                }
                break;
            case 43:
                if(!follower.isBusy() && ms.getCurrentPosition() > 1800){
                    outputClaw.setPosition(outputClawOpenPos);
                    setPathState(44);
                }

            case 44:
                if(!follower.isBusy()){
                    follower.setMaxPower(1.0);
                    follower.followPath(sub2);
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
        outputClaw.setPosition(outputClawClosedPos);
        deltaLeft.setPosition(deltaLeftSpecimenPos);
        deltaRight.setPosition(deltaRightSpecimenPos);
        outputWrist.setPosition(outputWristSpecimenPos);

        intakeClaw.setPosition(intakeClawOpenPos);
        intakeWrist.setPosition(intakeWristStraightPos);
        alpha.setPosition(alphaTransferPos);
        beta.setPosition(betaTransferPos);
        limelight.start();
    }

    @Override
    public void start(){
        super.start();
        limelight.pipelineSwitch(9);
    }
}
