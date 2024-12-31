package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Config
@Autonomous (preselectTeleOp = "intothedeep_tele")
public class five_specimen extends intothedeep_auto{

    TrajectoryActionBuilder preloadSpecimen;
    TrajectoryActionBuilder getToFirst;
    TrajectoryActionBuilder getToSecond;
    TrajectoryActionBuilder getToThird;
    TrajectoryActionBuilder transition;
    TrajectoryActionBuilder splineToOutput2;
    TrajectoryActionBuilder splineToIntake3;
    TrajectoryActionBuilder splineToOutput3;
    TrajectoryActionBuilder splineToIntake4;
    TrajectoryActionBuilder splineToOutput4;
    TrajectoryActionBuilder splineToIntake5;
    TrajectoryActionBuilder splineToOutput5;
    TrajectoryActionBuilder park;

    Action pSpec;
    Action gtFirst;
    Action gtSecond;
    Action gtThird;
    Action trans;
    Action stOutput2;
    Action stIntake3;
    Action stOutput3;
    Action stIntake4;
    Action stOutput4;
    Action stIntake5;
    Action stOutput5;
    Action raiseSpecSlides;
    Action specimenWrist;

    Action par;

    public void init(){
        super.init();

        Pose2d startPos = new Pose2d(-7.1791, 65.21, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPos);

        Actions.runBlocking(
                new ParallelAction(
                        outputwrist(outputWristSpecimenPos),
                        outputarm(deltaLeftSpecimenPos, deltaRightSpecimenPos),
                        outputclaw(outputClawClosedPos),
                        intakeclaw(intakeClawOpenPos),
                        intakearm(alphaTransferPos, betaTransferPos),
                        intakewrist(intakeWristStraightPos)
                )
        );

        preloadSpecimen = drive.actionBuilder(startPos)
                .strafeToConstantHeading(new Vector2d(-6, 31));

        getToFirst = drive.actionBuilder(new Pose2d(0, 31, Math.toRadians(97)))
                .setTangent(Math.toRadians(95))
                .splineToConstantHeading(new Vector2d(-33, 31), Math.toRadians(270))
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-44, 15), Math.toRadians(97))
                .strafeToConstantHeading(new Vector2d(-44, 55));

        getToSecond = drive.actionBuilder(new Pose2d(-44, 55, Math.toRadians(97)))
                .setTangent(Math.toRadians(290))
                .splineToConstantHeading(new Vector2d(-54, 15), Math.toRadians(97))
                .strafeToConstantHeading(new Vector2d(-52, 56));

        getToThird = drive.actionBuilder(new Pose2d(-52, 53, Math.toRadians(97)))
                .setTangent(Math.toRadians(290))
                .splineToConstantHeading(new Vector2d(-64, 15), Math.toRadians(97))
                .strafeToConstantHeading(new Vector2d(-60, 56));

        transition = drive.actionBuilder(new Pose2d(-60,56,Math.toRadians(97)))
                .setTangent(Math.toRadians(300))
                .splineToLinearHeading(new Pose2d(-18, 52, Math.toRadians(135)), Math.toRadians(15));

        splineToOutput2 = drive.actionBuilder(new Pose2d(-18, 52, Math.toRadians(135)))
                .setTangent(270)
                .splineToLinearHeading(new Pose2d(-9, 41, Math.toRadians(135)), Math.toRadians(280));

        splineToIntake3 = drive.actionBuilder(new Pose2d(-3, 41, Math.toRadians(120)))
                .setTangent(90)
                .splineToLinearHeading(new Pose2d(-18, 52, Math.toRadians(135)), Math.toRadians(135));


        splineToOutput3 = drive.actionBuilder(new Pose2d(-18, 52, Math.toRadians(135)))
                .setTangent(270)
                .splineToLinearHeading(new Pose2d(-7, 41, Math.toRadians(135)), Math.toRadians(280));

        splineToIntake4 = drive.actionBuilder(new Pose2d(-3, 41, Math.toRadians(120)))
                .setTangent(90)
                .splineToLinearHeading(new Pose2d(-18, 52, Math.toRadians(135)), Math.toRadians(135));


        splineToOutput4 = drive.actionBuilder(new Pose2d(-18, 52, Math.toRadians(135)))
                .setTangent(270)
                .splineToLinearHeading(new Pose2d(-5, 41, Math.toRadians(135)), Math.toRadians(280));

        splineToIntake5 = drive.actionBuilder(new Pose2d(-3, 41, Math.toRadians(120)))
                .setTangent(90)
                .splineToLinearHeading(new Pose2d(-18, 52, Math.toRadians(135)), Math.toRadians(135));

        splineToOutput5 = drive.actionBuilder(new Pose2d(-18, 52, Math.toRadians(135)))
                .setTangent(270)
                .splineToLinearHeading(new Pose2d(-3, 41, Math.toRadians(135)), Math.toRadians(280));

        park = drive.actionBuilder(new Pose2d(-3, 41, Math.toRadians(135)))
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-60, 60, Math.toRadians(270)), Math.toRadians(90));


        pSpec = preloadSpecimen.build();
        gtFirst = getToFirst.build();
        gtSecond = getToSecond.build();
        gtThird = getToThird.build();
        trans = transition.build();
        stOutput2 = splineToOutput2.build();
        stIntake3 = splineToIntake3.build();
        stOutput3 = splineToOutput3.build();
        stIntake4 = splineToIntake4.build();
        stOutput4 = splineToOutput4.build();
        stIntake5 = splineToIntake5.build();
        stOutput5 = splineToOutput5.build();
        par = park.build();
    }

    @Override
    public void childLoop(){
        super.childLoop();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(pSpec,
                        vertslides(slideSpecimenVe)),

                        new ParallelAction(vertslides(slidePositionTargetVe += 250),
                                outputwrist(outputWristSpecimenPos)),

                        new ParallelAction(gtFirst,
                                outputarm(deltaLeftPreTransfer, deltaRightPreTransfer),
                                outputwrist(outputWristStraightPos),
                                vertslides(slideMin),
                                outputclaw(outputClawOpenPos)),
                        gtSecond,
                        gtThird,
                        trans,
                        stOutput2,
                        stIntake3,
                        stOutput3,
                        stIntake4,
                        stOutput4,
                        stIntake5,
                        stOutput5,
                        par
                )
        );
    }

}