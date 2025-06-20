package OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.PwmControl;


@Config
@Autonomous (preselectTeleOp = "intothedeep_tele_blue")
public class fourspec_blue extends intothedeep_auto{

    private final Pose startPos = new Pose(7.1791, 65.21, Math.toRadians(180));
    private final Pose preloadPos = new Pose(35, 65.21, Math.toRadians(210));
    private final Pose controlSpec1Pos = new Pose(20, 50, Math.toRadians(315));
    private final Pose spec1Pos = new Pose(25.2, 45.5, Math.toRadians(315));
    private final Pose turnSpec1Pos = new Pose(24.2, 45.8, Math.toRadians(240));
    private final Pose spec2Pos = new Pose(30.9, 39.5, Math.toRadians(300));
    private final Pose turnSpec2Pos = new Pose(30.0, 40.3, Math.toRadians(230));
    private final Pose intakePos = new Pose(24.7, 53.9, Math.toRadians(225)); //24.5, 53.5, 225
    private final Pose scoreSpec1Pos = new Pose(36, 70, Math.toRadians(225));
    private final Pose scoreSpec2Pos = new Pose(36, 71, Math.toRadians(225));
    private final Pose scoreSpec3Pos = new Pose(36, 74, Math.toRadians(225));
    private final Pose parkPos = new Pose(20, 53, Math.toRadians(225));
    private final Pose scoreControl = new Pose(23.5, 66.4, Math.toRadians(225));
    private Path scorePreload, park;
    private PathChain grabSpec1, turnSpec1, grabSpec2, turnSpec2, intakeSpec1, scoreSpec1, intakeSpec2, scoreSpec2, intakeSpec3, scoreSpec3;


    public void buildPaths(){
        scorePreload = new Path(new BezierLine(new Point(startPos), new Point(preloadPos)));
        scorePreload.setLinearHeadingInterpolation(startPos.getHeading(), preloadPos.getHeading());

        grabSpec1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(preloadPos), /* Control Point */ new Point(controlSpec1Pos), new Point(spec1Pos)))
                .setLinearHeadingInterpolation(preloadPos.getHeading(), spec1Pos.getHeading())
                .build();

        turnSpec1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(spec1Pos), new Point(turnSpec1Pos)))
                .setLinearHeadingInterpolation(spec1Pos.getHeading(), turnSpec1Pos.getHeading())
                .build();

        grabSpec2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(turnSpec1Pos), new Point(spec2Pos)))
                .setLinearHeadingInterpolation(turnSpec1Pos.getHeading(), spec2Pos.getHeading())
                .build();

        turnSpec2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(spec2Pos), new Point(turnSpec2Pos)))
                .setLinearHeadingInterpolation(spec2Pos.getHeading(), turnSpec2Pos.getHeading())
                .build();

        intakeSpec1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(spec2Pos), new Point(intakePos)))
                .setLinearHeadingInterpolation(turnSpec2Pos.getHeading(), intakePos.getHeading())
                .build();

        scoreSpec1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intakePos), new Point(scoreSpec1Pos)))
                .setLinearHeadingInterpolation(intakePos.getHeading(), scoreSpec1Pos.getHeading())
                .setZeroPowerAccelerationMultiplier(0.4)
                .build();

        intakeSpec2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoreSpec1Pos), new Point(intakePos)))
                .setLinearHeadingInterpolation(scoreSpec1Pos.getHeading(), intakePos.getHeading())
                .build();

        scoreSpec2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intakePos), new Point(scoreSpec2Pos)))
                .setLinearHeadingInterpolation(intakePos.getHeading(), scoreSpec2Pos.getHeading())
                .setZeroPowerAccelerationMultiplier(0.4)
                .build();

        intakeSpec3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoreSpec2Pos), new Point(intakePos)))
                .setLinearHeadingInterpolation(scoreSpec2Pos.getHeading(), intakePos.getHeading())
                .build();

        scoreSpec3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intakePos), new Point(scoreSpec3Pos)))
                .setLinearHeadingInterpolation(intakePos.getHeading(), scoreSpec3Pos.getHeading())
                .setZeroPowerAccelerationMultiplier(0.4)
                .build();

        park = new Path(new BezierLine(new Point(scoreSpec3Pos), new Point(parkPos)));
        scorePreload.setLinearHeadingInterpolation(scoreSpec3Pos.getHeading(), parkPos.getHeading());
    }

    @Override
    public void autonomousPathUpdate(){
        switch (pathState){
            case 0:
                follower.setMaxPower(0.9);
                follower.followPath(scorePreload);
                slidePositionTargetVe = 500;
                setPathState(1);
                break;
            case 1:
                if(follower.getPose().getX() > 34 && ms.getCurrentPosition() > 300){
                    slidePositionTargetVe = 885;
                    outputWrist.setPosition(0.15);
                    setPathState(2);
                }
                break;
            case 2:
                if(ms.getCurrentPosition() > 750){
                    follower.setMaxPower(0.6);
                    follower.followPath(grabSpec1);
                    setPathState(3);
                }
            case 3:
                if(follower.getPose().getX() < 28){
                    slidePositionTargetVe = slideMinVe;
                    deltaLeft.setPosition(deltaLeftPreTransfer);
                    deltaRight.setPosition(deltaRightPreTransfer);
                    outputWrist.setPosition(outputWristStraightPos);
                }

                if(ms.getCurrentPosition() < 50){
                    outputClaw.setPosition(outputClawOpenPos);
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
                if(pathTimer.getElapsedTimeSeconds() > 0.5){
                    alpha.setPosition(alphaLowerPos);
                    beta.setPosition(betaLowerPos);
                    setPathState(6);
                }
                break;

            case 6:
                if(pathTimer.getElapsedTimeSeconds() >= INTAKELOWER_TIME){
                    outputClaw.setPosition(outputClawOpenPos);
                    intakeClaw.setPosition(intakeClawClosedPos);
                    setPathState(7);
                }
                break;

            case 7:
                if(pathTimer.getElapsedTimeSeconds() >= INTAKECLAW_TIME){
                    alpha.setPosition(alphaIntakePos);
                    beta.setPosition(betaIntakePos);
                    follower.setMaxPower(1.0);
                    follower.turnDegrees(80, false);
                    setPathState(8);
                }
                break;

            case 8:
                if(pathTimer.getElapsedTimeSeconds() > 0.9){ //follower.getPose().getHeading() > 230
                    intakeClaw.setPosition(intakeClawOpenPos);
                    setPathState(9);
                }
                break;

            case 9:
                if(pathTimer.getElapsedTimeSeconds() > 0.3){
                    slidePositionTargetEx = slideMinEx;
                    exAddPower = true;
                    setPathState(10);
                }
                break;

            case 10:
                if(extendo.getCurrentPosition() < 30){
                    follower.setMaxPower(0.70);
                    follower.followPath(grabSpec2);
                    setPathState(11);
                }
                break;

            case 11:
                if(!follower.isBusy()){
                    slidePositionTargetEx = slideMaxEx;
                    intakeWrist.setPosition(0.36);
                    setPathState(12);
                }
            case 12:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.4){
                    alpha.setPosition(alphaLowerPos);
                    beta.setPosition(betaLowerPos);
                    setPathState(13);
                }
                break;

            case 13:
                if(pathTimer.getElapsedTimeSeconds() >= 0.15){
                    intakeClaw.setPosition(intakeClawClosedPos);
                    setPathState(14);
                }
                break;

            case 14:
                if(pathTimer.getElapsedTimeSeconds() >= INTAKECLAW_TIME){
                    alpha.setPosition(alphaIntakePos);
                    beta.setPosition(betaIntakePos);
                    //follower.followPath(turnSpec2);
                    follower.setMaxPower(1.0);
                    follower.turnDegrees(80, false);
                    //follower.turn;
                    setPathState(15);
                }
                break;

            case 15:
                if(pathTimer.getElapsedTimeSeconds() > 0.8){ //follower.getPose().getHeading() > 230
                    intakeClaw.setPosition(intakeClawOpenPos);
                    setPathState(16);
                }
                break;

            case 16:
                if(pathTimer.getElapsedTimeSeconds() > 0.3){
                    slidePositionTargetEx = slideMinEx;
                    intakeWrist.setPosition(intakeWristStraightPos);
                    exAddPower = true;
                    follower.setMaxPower(0.8);
                    follower.followPath(intakeSpec1);
                    setPathState(17);
                }
                break;

            case 17:
                if(!follower.isBusy() && extendo.getCurrentPosition() < 60){ //path timer > 0.7
                    exAddPower = false;
                    slidePositionTargetEx = slideMaxEx;
                    alpha.setPosition(alphaIntakePos);
                    beta.setPosition(betaIntakePos);
                    setPathState(18);
                }
                break;

            case 18:
                if(extendo.getCurrentPosition() > 300){
                    alpha.setPosition(alphaLowerPos);
                    beta.setPosition(betaLowerPos);
                    setPathState(19);
                }
                break;

            case 19:
                if(pathTimer.getElapsedTimeSeconds() >= INTAKELOWER_TIME){
                    intakeClaw.setPosition(intakeClawClosedPos);
                    outputClaw.setPosition(outputClawOpenPos);
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
                    exConstantPID = -1.5;
                    setPathState(21);
                }
                break;

            case 21:
                if(extendo.getCurrentPosition() < intakeThreshold){
                    exAddPower = true;
                    exConstantPID = -1.5;
                    deltaLeft.setPosition(deltaLeftTransferPos);
                    deltaRight.setPosition(deltaRightTransferPos);
                    setPathState(22);
                }
                break;

            case 22:
                if(pathTimer.getElapsedTimeSeconds() >= OUTPUTARM_READY){
                    exAddPower = true;
                    exConstantPID = -1.5;
                    outputClaw.setPosition(outputClawClosedPos);
                    setPathState(23);
                }
                break;

            case 23:
                if(pathTimer.getElapsedTimeSeconds() >= OUTPUTCLAW_TIME){
                    exAddPower = true;
                    exConstantPID = -1.5;
                    intakeClaw.setPosition(intakeClawOpenPos);
                    follower.followPath(scoreSpec1);
                    setPathState(24);
                }
                break;

            case 24:
                if(pathTimer.getElapsedTimeSeconds() > 0.05){
                    exAddPower = false;
                    exConstantPID = 0.0;
                    slidePositionTargetVe = 365;
                    deltaLeft.setPosition(deltaLeftSpecimenPos);
                    deltaRight.setPosition(deltaRightSpecimenPos);
                    outputWrist.setPosition(outputWristSpecimenPos);
                    setPathState(25);
                }
                break;

            case 25:
                if(follower.getPose().getX() > 34.5){
                    slidePositionTargetVe = 875;
                    outputWrist.setPosition(0.15);
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
                    setPathState(28);
                }
                break;

            //spec2
            case 28:
                if(!follower.isBusy()){
                    alpha.setPosition(alphaLowerPos);
                    beta.setPosition(betaLowerPos);
                    setPathState(29);
                }
                break;

            case 29:
                if(pathTimer.getElapsedTimeSeconds() >= INTAKELOWER_TIME){
                    intakeClaw.setPosition(intakeClawClosedPos);
                    outputClaw.setPosition(outputClawOpenPos);
                    setPathState(30);
                }
                break;

            case 30:
                if(pathTimer.getElapsedTimeSeconds() >= INTAKECLAW_TIME){
                    alpha.setPosition(alphaTransferPos);
                    beta.setPosition(betaTransferPos);
                    intakeWrist.setPosition(intakeWristStraightPos);
                    slidePositionTargetEx = slideMinEx;
                    exAddPower = true;
                    exConstantPID = -1.5;

                    setPathState(31);
                }
                break;

            case 31:
                if(extendo.getCurrentPosition() < intakeThreshold){
                    exAddPower = true;
                    exConstantPID = -1.5;
                    deltaLeft.setPosition(deltaLeftTransferPos);
                    deltaRight.setPosition(deltaRightTransferPos);
                    setPathState(32);
                }
                break;

            case 32:
                if(pathTimer.getElapsedTimeSeconds() >= OUTPUTARM_READY){
                    exAddPower = true;
                    exConstantPID = -1.5;
                    outputClaw.setPosition(outputClawClosedPos);
                    setPathState(33);
                }
                break;

            case 33:
                if(pathTimer.getElapsedTimeSeconds() >= OUTPUTCLAW_TIME){
                    intakeClaw.setPosition(intakeClawOpenPos);
                    exAddPower = true;
                    exConstantPID = -1.5;
                    follower.setMaxPower(0.50);
                    follower.followPath(scoreSpec2);
                    setPathState(34);
                }
                break;

            case 34:
                if(pathTimer.getElapsedTimeSeconds() > 0.15){
                    exAddPower = false;
                    exConstantPID = 0.0;
                    slidePositionTargetVe = 365;
                    deltaLeft.setPosition(deltaLeftSpecimenPos);
                    deltaRight.setPosition(deltaRightSpecimenPos);
                    outputWrist.setPosition(outputWristSpecimenPos);
                    setPathState(35);
                }
                break;

            case 35:
                if(follower.getPose().getX() > 34.5){
                    slidePositionTargetVe = 875;
                    outputWrist.setPosition(0.15);
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
                    setPathState(38);
                }
                break;

            case 38:
                if(!follower.isBusy()){
                    alpha.setPosition(alphaLowerPos);
                    beta.setPosition(betaLowerPos);
                    setPathState(39);
                }
                break;

            case 39:
                if(pathTimer.getElapsedTimeSeconds() >= INTAKELOWER_TIME){
                    intakeClaw.setPosition(intakeClawClosedPos);
                    outputClaw.setPosition(outputClawOpenPos);
                    setPathState(40);
                }
                break;

            case 40:
                if(pathTimer.getElapsedTimeSeconds() >= INTAKECLAW_TIME){
                    alpha.setPosition(alphaTransferPos);
                    beta.setPosition(betaTransferPos);
                    intakeWrist.setPosition(intakeWristStraightPos);
                    slidePositionTargetEx = slideMinEx;
                    exAddPower = true;
                    exConstantPID = -1.5;
                    setPathState(41);
                }
                break;

            case 41:
                if(extendo.getCurrentPosition() < intakeThreshold){
                    exAddPower = true;
                    exConstantPID = -1.5;
                    deltaLeft.setPosition(deltaLeftTransferPos);
                    deltaRight.setPosition(deltaRightTransferPos);
                    setPathState(42);
                }
                break;

            case 42:
                if(pathTimer.getElapsedTimeSeconds() >= OUTPUTARM_READY){
                    exAddPower = true;
                    exConstantPID = -1.5;
                    outputClaw.setPosition(outputClawClosedPos);
                    setPathState(43);
                }
                break;

            case 43:
                if(pathTimer.getElapsedTimeSeconds() >= OUTPUTCLAW_TIME){
                    exAddPower = true;
                    exConstantPID = -1.5;
                    intakeClaw.setPosition(intakeClawOpenPos);
                    follower.setMaxPower(0.50);
                    follower.followPath(scoreSpec3);
                    setPathState(44);
                }
                break;

            case 44:
                if(pathTimer.getElapsedTimeSeconds() > 0.15){
                    exAddPower = false;
                    exConstantPID = 0.0;
                    slidePositionTargetVe = 365;
                    deltaLeft.setPosition(deltaLeftSpecimenPos);
                    deltaRight.setPosition(deltaRightSpecimenPos);
                    outputWrist.setPosition(outputWristSpecimenPos);
                    setPathState(45);
                }
                break;

            case 45:
                if(follower.getPose().getX() > 34.5){
                    slidePositionTargetVe = 875;
                    outputWrist.setPosition(0.15);
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
                    setPathState(-1);
                }
                break;
        }

    }

    @Override
    public void init(){
        super.init();
        follower.setStartingPose(startPos);
        buildPaths();
        deltaLeft.setPosition(deltaLeftSpecimenPos);
        deltaRight.setPosition(deltaRightSpecimenPos);
        outputWrist.setPosition(outputWristSpecimenPos);
        outputClaw.setPosition(outputClawClosedPos);

        intakeClaw.setPosition(intakeClawOpenPos);
        intakeWrist.setPosition(intakeWristStraightPos);
        alpha.setPosition(alphaTransferPos);
        beta.setPosition(betaTransferPos);
    }
}
