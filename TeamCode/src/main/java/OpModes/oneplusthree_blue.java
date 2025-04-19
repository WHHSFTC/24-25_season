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
public class oneplusthree_blue extends intothedeep_auto{

    private final Pose startPos = new Pose(7.1791, 79.21, Math.toRadians(180));
    private final Pose scoreSpecimenPos = new Pose(34, 79.21, Math.toRadians(210));
    private final Pose scorePos = new Pose(14.2, 126.0, Math.toRadians(315));
    private final Pose preloadControlPos = new Pose(25, 100, Math.toRadians(315));
    private final Pose sample1Pos = new Pose(19.1, 103.2, Math.toRadians(30));
    private final Pose sample2Pos = new Pose(15.9, 125.2, Math.toRadians(5));
    private final Pose sample3Pos = new Pose(24.5, 117.4, Math.toRadians(47));
    private final Pose parkControlPos = new Pose(55, 135, Math.toRadians(90));
    private final Pose parkPos = new Pose(60, 90, Math.toRadians(90));

    private Path scorePreload, park;
    private PathChain grabSample1, grabSample2, grabSample3, scoreSample1, scoreSample2, scoreSample3;


    public void buildPaths(){
        scorePreload = new Path(new BezierLine(new Point(startPos), new Point(scoreSpecimenPos)));
        scorePreload.setLinearHeadingInterpolation(startPos.getHeading(), scoreSpecimenPos.getHeading());

        grabSample1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scoreSpecimenPos), new Point(20, 100), new Point(sample1Pos)))
                .setLinearHeadingInterpolation(scoreSpecimenPos.getHeading(), sample1Pos.getHeading())
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

        park = new Path(new BezierCurve(new Point(scorePos), new Point(parkControlPos), new Point(parkPos)));
        park.setLinearHeadingInterpolation(scorePos.getHeading(), parkPos.getHeading());
    }

    @Override
    public void autonomousPathUpdate(){
        switch (pathState){
            case 0:
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
                    follower.setMaxPower(0.7);
                    follower.followPath(grabSample1);
                    setPathState(3);
                }
                break;

            case 3:
                if(follower.getPose().getX() < 30){
                    outputClaw.setPosition(outputClawOpenPos);
                }

                if(pathTimer.getElapsedTimeSeconds() > 0.5){
                    slidePositionTargetVe = slideMinVe;
                    deltaLeft.setPosition(deltaLeftPreTransfer);
                    deltaRight.setPosition(deltaRightPreTransfer);
                    outputWrist.setPosition(outputWristStraightPos);
                    outputClaw.setPosition(outputClawOpenPos);
                }

                if(ms.getCurrentPosition() < 30){
                    setPathState(4);
                }
                break;


            case 4:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5){
                    slidePositionTargetEx = slideMaxEx;
                    alpha.setPosition(alphaIntakePos);
                    beta.setPosition(betaIntakePos);
                    intakeWrist.setPosition(0.12);
                    setPathState(5);
                }
                break;

            case 5:
                if(pathTimer.getElapsedTimeSeconds() > 0.8 && extendo.getCurrentPosition() > 300){
                    alpha.setPosition(alphaLowerPos);
                    beta.setPosition(betaLowerPos);
                    setPathState(6);
                }
                break;

            case 6:
                if(pathTimer.getElapsedTimeSeconds() >= 0.5){
                    intakeClaw.setPosition(intakeClawClosedPos);
                    deltaRight.setPosition(deltaRightPreTransfer);
                    deltaLeft.setPosition(deltaLeftPreTransfer);
                    setPathState(7);
                }
                break;

            case 7:
                if(pathTimer.getElapsedTimeSeconds() >= 0.5){
                    alpha.setPosition(alphaTransferPos);
                    beta.setPosition(betaTransferPos);
                    intakeWrist.setPosition(intakeWristStraightPos);
                    slidePositionTargetEx = slideMinEx;
                    exAddPower = true;
                    setPathState(8);
                }
                break;

            case 8:
                if(extendoSlidesLimit.isPressed()){
                    exAddPower = false;
                    deltaLeft.setPosition(deltaLeftTransferPos);
                    deltaRight.setPosition(deltaRightTransferPos);
                    setPathState(9);
                }
                break;

            case 9:
                if(pathTimer.getElapsedTimeSeconds() >= OUTPUTARM_READY){
                    outputClaw.setPosition(outputClawClosedPos);
                    setPathState(10);
                }
                break;

            case 10:
                if(pathTimer.getElapsedTimeSeconds() >= OUTPUTCLAW_TIME){
                    intakeClaw.setPosition(intakeClawOpenPos);
                    setPathState(11);
                }
                break;

            case 11:
                slidePositionTargetVe = slideMaxVe;
                deltaLeft.setPosition(deltaLeftSamplePos);
                deltaRight.setPosition(deltaRightSamplePos);
                outputWrist.setPosition(outputWristSwitchPos);
                setPathState(12);
                break;

            case 12:
                follower.followPath(scoreSample1);
                setPathState(13);
                break;

            case 13:
                if(!follower.isBusy() && slidePositionTargetVe > 1750){
                    outputClaw.setPosition(outputClawOpenPos);
                    setPathState(14);
                }
                break;

            case 14:
                if(pathTimer.getElapsedTimeSeconds() > 0.3){
                    follower.setMaxPower(0.7);
                    follower.followPath(grabSample2);
                    setPathState(15);
                }
                break;

            case 15:
                if(!follower.isBusy()){
                    slidePositionTargetVe = slideMinVe;
                    deltaRight.setPosition(deltaRightPreTransfer);
                    deltaLeft.setPosition(deltaLeftPreTransfer);
                    outputWrist.setPosition(outputWristStraightPos);

                    slidePositionTargetEx = slideMaxEx;
                    alpha.setPosition(alphaIntakePos);
                    beta.setPosition(betaIntakePos);
                    setPathState(16);
                }
                break;

            case 16:
                if(ms.getCurrentPosition() < 45 && extendo.getCurrentPosition() > 300){
                    alpha.setPosition(alphaLowerPos);
                    beta.setPosition(betaLowerPos);
                    setPathState(17);
                }
                break;

            case 17:
                if(pathTimer.getElapsedTimeSeconds() >= 0.5){
                    intakeClaw.setPosition(intakeClawClosedPos);
                    setPathState(18);
                }
                break;

            case 18:
                if(pathTimer.getElapsedTimeSeconds() >= 0.5){
                    alpha.setPosition(alphaTransferPos);
                    beta.setPosition(betaTransferPos);
                    intakeWrist.setPosition(intakeWristStraightPos);
                    slidePositionTargetEx = slideMinEx;
                    exAddPower = true;
                    setPathState(19);
                }
                break;

            case 19:
                if(extendoSlidesLimit.isPressed()){
                    exAddPower = false;
                    deltaLeft.setPosition(deltaLeftTransferPos);
                    deltaRight.setPosition(deltaRightTransferPos);
                    setPathState(20);
                }
                break;

            case 20:
                if(pathTimer.getElapsedTimeSeconds() >= OUTPUTARM_READY){
                    outputClaw.setPosition(outputClawClosedPos);
                    setPathState(21);
                }
                break;

            case 21:
                if(pathTimer.getElapsedTimeSeconds() >= OUTPUTCLAW_TIME){
                    intakeClaw.setPosition(intakeClawOpenPos);
                    setPathState(22);
                }
                break;

            case 22:
                slidePositionTargetVe = slideMaxVe;
                deltaLeft.setPosition(deltaLeftSamplePos);
                deltaRight.setPosition(deltaRightSamplePos);
                outputWrist.setPosition(outputWristSwitchPos);
                setPathState(23);
                break;

            case 23:
                if(ms.getCurrentPosition() > 1750){
                    follower.followPath(scoreSample2);
                    setPathState(24);
                }
                break;

            case 24:
                if(!follower.isBusy() && slidePositionTargetVe > 1750){
                    outputClaw.setPosition(outputClawOpenPos);
                    setPathState(25);
                }
                break;

            case 25:
                if(pathTimer.getElapsedTimeSeconds() > 0.3){
                    follower.setMaxPower(0.7);
                    follower.followPath(grabSample3);
                    setPathState(26);
                }
                break;

            case 26:
                if(!follower.isBusy()){
                    slidePositionTargetVe = slideMinVe;
                    deltaRight.setPosition(deltaRightPreTransfer);
                    deltaLeft.setPosition(deltaLeftPreTransfer);
                    outputWrist.setPosition(outputWristStraightPos);

                    slidePositionTargetEx = slideMaxEx;
                    alpha.setPosition(alphaIntakePos);
                    beta.setPosition(betaIntakePos);
                    intakeWrist.setPosition(0.12);
                    setPathState(27);
                }
                break;

            case 27:
                if(ms.getCurrentPosition() < 30){
                    alpha.setPosition(alphaLowerPos);
                    beta.setPosition(betaLowerPos);
                    setPathState(28);
                }
                break;

            case 28:
                if(pathTimer.getElapsedTimeSeconds() >= INTAKELOWER_TIME){
                    intakeClaw.setPosition(intakeClawClosedPos);
                    setPathState(29);
                }
                break;

            case 29:
                if(pathTimer.getElapsedTimeSeconds() >= INTAKECLAW_TIME){
                    alpha.setPosition(alphaTransferPos);
                    beta.setPosition(betaTransferPos);
                    intakeWrist.setPosition(intakeWristStraightPos);
                    slidePositionTargetEx = slideMinEx;
                    exAddPower = true;
                    setPathState(30);
                }
                break;

            case 30:
                if(extendoSlidesLimit.isPressed()){
                    exAddPower = false;
                    deltaLeft.setPosition(deltaLeftTransferPos);
                    deltaRight.setPosition(deltaRightTransferPos);
                    setPathState(31);
                }
                break;

            case 31:
                if(pathTimer.getElapsedTimeSeconds() >= OUTPUTARM_READY){
                    outputClaw.setPosition(outputClawClosedPos);
                    setPathState(32);
                }
                break;

            case 32:
                if(pathTimer.getElapsedTimeSeconds() >= OUTPUTCLAW_TIME){
                    intakeClaw.setPosition(intakeClawOpenPos);
                    setPathState(33);
                }
                break;

            case 33:
                slidePositionTargetVe = slideMaxVe;
                deltaLeft.setPosition(deltaLeftSamplePos);
                deltaRight.setPosition(deltaRightSamplePos);
                outputWrist.setPosition(outputWristSwitchPos);
                setPathState(34);
                break;

            case 34:
                if(ms.getCurrentPosition() > 1750){
                    follower.followPath(scoreSample3);
                    setPathState(35);
                }
                break;

            case 35:
                if(!follower.isBusy() && slidePositionTargetVe > 1750){
                    outputClaw.setPosition(outputClawOpenPos);
                    setPathState(36);
                }
                break;

            case 36:
                follower.setMaxPower(0.9);
                follower.followPath(park);
                setPathState(37);
                break;

            case 37:
                if(pathTimer.getElapsedTimeSeconds() >= 0.6){
                    deltaLeft.setPosition(deltaLeftPreTransfer);
                    deltaRight.setPosition(deltaRightPreTransfer);
                    outputWrist.setPosition(outputWristStraightPos);
                    slidePositionTargetVe = 1150;
                    setPathState(38);
                }
                break;

            case 38:
                slidePositionTargetVe = 1000;
                setPathState(-1);
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
    }

}
