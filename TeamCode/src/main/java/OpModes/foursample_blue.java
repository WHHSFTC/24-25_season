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
@Autonomous(preselectTeleOp = "intothedeep_tele_blue")
public class foursample_blue extends intothedeep_auto{
    private final Pose startPos = new Pose(7.21, 103.1791, Math.toRadians(270));
    private final Pose scorePos = new Pose(13.5, 126.3, Math.toRadians(315));
    private final Pose preloadControlPos = new Pose(27.52, 115.77, Math.toRadians(315));
    private final Pose sample1Pos = new Pose(16.2, 123.7, Math.toRadians(350));
    private final Pose sample2Pos = new Pose(16.1, 126.2, Math.toRadians(5));
    private final Pose sample3Pos = new Pose(16.9, 130.8, Math.toRadians(19));
    private final Pose parkControlPos = new Pose(55, 135, Math.toRadians(90));
    private final Pose parkPos = new Pose(60, 90, Math.toRadians(90));

    private Path scorePreload, park;
    private PathChain grabSample1, grabSample2, grabSample3, scoreSample1, scoreSample2, scoreSample3;


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

        park = new Path(new BezierCurve(new Point(scorePos), new Point(parkControlPos), new Point(parkPos)));
        park.setLinearHeadingInterpolation(scorePos.getHeading(), parkPos.getHeading());
    }

    @Override
    public void autonomousPathUpdate(){
        switch (pathState){
            case 0:
                follower.followPath(scorePreload);
                slidePositionTargetVe = slideMaxVe;
                outputWrist.setPosition(outputWristSwitchPos);
                deltaRight.setPosition(deltaRightSamplePos);
                deltaLeft.setPosition(deltaLeftSamplePos);
                setPathState(1);
                break;

            case 1:
                if(!follower.isBusy() && ms.getCurrentPosition() > 1750){
                    outputClaw.setPosition(outputClawOpenPos);
                    slidePositionTargetEx = slideMaxEx;
                    alpha.setPosition(alphaIntakePos);
                    beta.setPosition(betaIntakePos);
                    setPathState(2);
                }
                break;

            case 2:
                if(pathTimer.getElapsedTimeSeconds() > 0.3){
                    follower.setMaxPower(0.7);
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
                if(pathTimer.getElapsedTimeSeconds() >= 0.5){
                    intakeClaw.setPosition(intakeClawClosedPos);
                    setPathState(5);
                }
                break;

            case 5:
                if(pathTimer.getElapsedTimeSeconds() >= 0.5){
                    alpha.setPosition(alphaTransferPos);
                    beta.setPosition(betaTransferPos);
                    //intakeClaw.setPosition(intakeClawBarelyClosedPos);
                    intakeWrist.setPosition(intakeWristStraightPos);
                    slidePositionTargetEx = slideMinEx;
                    exAddPower = true;
                    setPathState(6);
                }
                break;

            case 6:
                if(extendoSlidesLimit.isPressed() && ms.getCurrentPosition() < 30){
                    exAddPower = false;
                    deltaLeft.setPosition(deltaLeftTransferPos);
                    deltaRight.setPosition(deltaRightTransferPos);
                    setPathState(7);
                }
                break;

            case 7:
                if(pathTimer.getElapsedTimeSeconds() >= OUTPUTARM_READY){
                    outputClaw.setPosition(outputClawClosedPos);
                    setPathState(8);
                }
                break;

            case 8:
                if(pathTimer.getElapsedTimeSeconds() >= OUTPUTCLAW_TIME){
                    intakeClaw.setPosition(intakeClawOpenPos);
                    setPathState(9);
                }
                break;

            case 9:
                slidePositionTargetVe = slideMaxVe;
                deltaLeft.setPosition(deltaLeftSamplePos);
                deltaRight.setPosition(deltaRightSamplePos);
                outputWrist.setPosition(outputWristSwitchPos);
                setPathState(10);
                break;

            case 10:
                if(ms.getCurrentPosition() > 1750){
                    follower.followPath(scoreSample1);
                    setPathState(11);
                }
                break;

            case 11:
                if(!follower.isBusy() && slidePositionTargetVe > 1750){
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
                if(pathTimer.getElapsedTimeSeconds() >= 0.5){
                    intakeClaw.setPosition(intakeClawClosedPos);
                    setPathState(15);
                }
                break;

            case 15:
                if(pathTimer.getElapsedTimeSeconds() >= 0.5){
                    //intakeClaw.setPosition(intakeClawBarelyClosedPos);
                    alpha.setPosition(alphaTransferPos);
                    beta.setPosition(betaTransferPos);
                    intakeWrist.setPosition(intakeWristStraightPos);
                    slidePositionTargetEx = slideMinEx;
                    exAddPower = true;
                    setPathState(16);
                }
                break;

            case 16:
                if(extendoSlidesLimit.isPressed() && ms.getCurrentPosition() < 30){
                    exAddPower = false;
                    deltaLeft.setPosition(deltaLeftTransferPos);
                    deltaRight.setPosition(deltaRightTransferPos);
                    setPathState(17);
                }
                break;

            case 17:
                if(pathTimer.getElapsedTimeSeconds() >= OUTPUTARM_READY){
                    outputClaw.setPosition(outputClawClosedPos);
                    setPathState(18);
                }
                break;

            case 18:
                if(pathTimer.getElapsedTimeSeconds() >= OUTPUTCLAW_TIME){
                    intakeClaw.setPosition(intakeClawOpenPos);
                    setPathState(19);
                }
                break;

            case 19:
                slidePositionTargetVe = slideMaxVe;
                deltaLeft.setPosition(deltaLeftSamplePos);
                deltaRight.setPosition(deltaRightSamplePos);
                outputWrist.setPosition(outputWristSwitchPos);
                setPathState(20);
                break;

            case 20:
                if(ms.getCurrentPosition() > 1750){
                    follower.followPath(scoreSample2);
                    setPathState(21);
                }
                break;

            case 21:
                if(!follower.isBusy() && slidePositionTargetVe > 1750){
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
                if(pathTimer.getElapsedTimeSeconds() >= INTAKELOWER_TIME){
                    intakeClaw.setPosition(intakeClawClosedPos);
                    setPathState(25);
                }
                break;

            case 25:
                if(pathTimer.getElapsedTimeSeconds() >= INTAKECLAW_TIME){
                    //intakeClaw.setPosition(intakeClawBarelyClosedPos);
                    alpha.setPosition(alphaTransferPos);
                    beta.setPosition(betaTransferPos);
                    intakeWrist.setPosition(intakeWristStraightPos);
                    slidePositionTargetEx = slideMinEx;
                    exAddPower = true;
                    setPathState(26);
                }
                break;

            case 26:
                if(extendoSlidesLimit.isPressed() && ms.getCurrentPosition() < 30){
                    exAddPower = false;
                    deltaLeft.setPosition(deltaLeftTransferPos);
                    deltaRight.setPosition(deltaRightTransferPos);
                    setPathState(27);
                }
                break;

            case 27:
                if(pathTimer.getElapsedTimeSeconds() >= OUTPUTARM_READY){
                    outputClaw.setPosition(outputClawClosedPos);
                    setPathState(28);
                }
                break;

            case 28:
                if(pathTimer.getElapsedTimeSeconds() >= OUTPUTCLAW_TIME){
                    intakeClaw.setPosition(intakeClawOpenPos);
                    setPathState(29);
                }
                break;

            case 29:
                slidePositionTargetVe = slideMaxVe;
                deltaLeft.setPosition(deltaLeftSamplePos);
                deltaRight.setPosition(deltaRightSamplePos);
                outputWrist.setPosition(outputWristSwitchPos);
                setPathState(30);
                break;

            case 30:
                if(ms.getCurrentPosition() > 1750){
                    follower.followPath(scoreSample3);
                    setPathState(31);
                }
                break;

            case 31:
                if(!follower.isBusy() && slidePositionTargetVe > 1750){
                    outputClaw.setPosition(outputClawOpenPos);
                    setPathState(32);
                }
                break;

            case 32:
                follower.setMaxPower(0.7);
                follower.followPath(park);
                setPathState(33);
                break;

            case 33:
                if(pathTimer.getElapsedTimeSeconds() >= 0.6){
                    deltaLeft.setPosition(deltaLeftPreTransfer);
                    deltaRight.setPosition(deltaRightPreTransfer);
                    outputWrist.setPosition(outputWristStraightPos);
                    slidePositionTargetVe = 1150;
                    setPathState(34);
                }
                break;

            case 34:
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
