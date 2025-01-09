package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.Vector;

@Config
@Autonomous (preselectTeleOp = "intothedeep_tele")
public class five_specimen extends intothedeep_auto{

    TrajectoryActionBuilder preloadSpecimen;
    TrajectoryActionBuilder getToFirst;
    TrajectoryActionBuilder getToSecond;
    TrajectoryActionBuilder getToThird;
    TrajectoryActionBuilder transition;
    TrajectoryActionBuilder retract;
    TrajectoryActionBuilder retractEx1;
    TrajectoryActionBuilder retractEx2;
    TrajectoryActionBuilder retractEx3;
    TrajectoryActionBuilder retractEx4;
    TrajectoryActionBuilder retractEx5;
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
    Action retraction;
    Action retract1;
    Action retract2;
    Action retract3;
    Action retract4;
    Action retract5;
    Action stOutput2;
    Action stIntake3;
    Action stOutput3;
    Action stIntake4;
    Action stOutput4;
    Action stIntake5;
    Action stOutput5;

    Action par;

    public void init(){
        super.init();

        Pose2d startPos = new Pose2d(-7.1791, 65.21, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPos);


        preloadSpecimen = drive.actionBuilder(startPos)
                .strafeToLinearHeading(new Vector2d(-6, 37), Math.toRadians(120))
                .afterTime(0.1, vertslides(750, false));

        getToFirst = drive.actionBuilder(new Pose2d(-6, 37, Math.toRadians(120)))
                .strafeToLinearHeading(new Vector2d(-6, 43), Math.toRadians(97))
                .afterTime(0.15, new OutputClaw(outputClawOpenPos))
                .strafeToConstantHeading(new Vector2d(-6, 48))

                .afterTime(0.7, new ParallelAction(outputarm(deltaLeftPreTransfer,deltaRightPreTransfer), outputwrist(outputWristStraightPos)))
                .afterTime(0.8, vertslides(slideMinVe, true))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-32, 39), Math.toRadians(270))
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-44, 17), Math.toRadians(97))
                .strafeToConstantHeading(new Vector2d(-44, 55))
                .afterTime(0.1, vertslides(slideMinVe, true))
                .setTangent(Math.toRadians(290))
                .splineToConstantHeading(new Vector2d(-54, 17), Math.toRadians(97))
                .strafeToConstantHeading(new Vector2d(-52, 55))

                .setTangent(Math.toRadians(300))
                .splineToLinearHeading(new Pose2d(-28, 48, Math.toRadians(145)), Math.toRadians(15))
                .afterTime(0.1, extendslides(slideMaxEx, false))
                .afterTime(0.4, intakearm(alphaLowerPos, betaLowerPos))
                .afterTime(0.7, intakeclaw(intakeClawClosedPos))
                .afterTime(1.3, intakearm(alphaTransferPos, betaTransferPos))
                .afterTime(1.5, new ExtendoSlides(slideMinEx, true))
                .endTrajectory();

        splineToOutput2 = drive.actionBuilder(new Pose2d(-28, 48, Math.toRadians(145)))

                .setTangent(270)
                .splineToLinearHeading(new Pose2d(-8, 37, Math.toRadians(135)), Math.toRadians(280))
                .afterTime(0.1, vertslides(750, false))
                .strafeToConstantHeading(new Vector2d(-8, 43))
                .afterTime(0.3, outputclaw(outputClawOpenPos));

        splineToIntake3 = drive.actionBuilder(new Pose2d(-8, 47, Math.toRadians(135)))
                .setTangent(90)
                .splineToLinearHeading(new Pose2d(-28, 48, Math.toRadians(135)), Math.toRadians(135))
                .afterTime(0.3, new IntakeArm(alphaLowerPos, betaLowerPos))
                .afterTime(0.6, new IntakeClaw(intakeClawClosedPos))
                .afterTime(1.1, new IntakeArm(alphaTransferPos, betaTransferPos));

        retractEx1 = drive.actionBuilder(new Pose2d(-28, 48, Math.toRadians(145)))
                .strafeToConstantHeading(new Vector2d(-30, 48))
                .afterTime(0.1, extendslides(slideMinEx, true));


        splineToOutput3 = drive.actionBuilder(new Pose2d(-29, 48, Math.toRadians(135)))
                .setTangent(270)
                .splineToLinearHeading(new Pose2d(-8, 37, Math.toRadians(135)), Math.toRadians(280))
                .afterTime(0.1, vertslides(750, false))
                .strafeToConstantHeading(new Vector2d(-8, 43))
                .afterTime(0.3, outputclaw(outputClawOpenPos));

        splineToIntake4 = drive.actionBuilder(new Pose2d(-8, 47, Math.toRadians(120)))
                .setTangent(90)
                .splineToLinearHeading(new Pose2d(-28, 48, Math.toRadians(135)), Math.toRadians(135))
                .afterTime(0.3, new IntakeArm(alphaLowerPos, betaLowerPos))
                .afterTime(0.5, new IntakeClaw(intakeClawClosedPos))
                .afterTime(0.9, new IntakeArm(alphaTransferPos, betaTransferPos));

        retractEx2 = drive.actionBuilder(new Pose2d(-28, 48, Math.toRadians(145)))
                .strafeToConstantHeading(new Vector2d(-29, 48))
                .afterTime(0.1, extendslides(slideMinEx, true));

        splineToOutput4 = drive.actionBuilder(new Pose2d(-18, 52, Math.toRadians(135)))
                .setTangent(270)
                .splineToLinearHeading(new Pose2d(-8, 37, Math.toRadians(135)), Math.toRadians(280))
                .afterTime(0.1, vertslides(750, false))
                .strafeToConstantHeading(new Vector2d(-8, 43))
                .afterTime(0.3, outputclaw(outputClawOpenPos));

        splineToIntake5 = drive.actionBuilder(new Pose2d(-3, 41, Math.toRadians(120)))
                .setTangent(90)
                .splineToLinearHeading(new Pose2d(-28, 48, Math.toRadians(135)), Math.toRadians(135))
                .afterTime(0.3, new IntakeArm(alphaLowerPos, betaLowerPos))
                .afterTime(0.5, new IntakeClaw(intakeClawClosedPos))
                .afterTime(0.9, new IntakeArm(alphaTransferPos, betaTransferPos));

        retractEx3 = drive.actionBuilder(new Pose2d(-28, 48, Math.toRadians(145)))
                .strafeToConstantHeading(new Vector2d(-29, 48))
                .afterTime(0.1, extendslides(slideMinEx, true));

        splineToOutput5 = drive.actionBuilder(new Pose2d(-18, 52, Math.toRadians(135)))
                .setTangent(270)
                .splineToLinearHeading(new Pose2d(-8, 37, Math.toRadians(135)), Math.toRadians(280))
                .afterTime(0.1, vertslides(750, false))
                .strafeToConstantHeading(new Vector2d(-8, 43))
                .afterTime(0.3, outputclaw(outputClawOpenPos));

        park = drive.actionBuilder(new Pose2d(-3, 41, Math.toRadians(135)))
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-60, 60, Math.toRadians(270)), Math.toRadians(90));

        pSpec = preloadSpecimen.build();
        gtFirst = getToFirst.build();
        stOutput2 = splineToOutput2.build();
        stIntake3 = splineToIntake3.build();
        retract1 = retractEx1.build();
        stOutput3 = splineToOutput3.build();
        stIntake4 = splineToIntake4.build();
        retract2 = retractEx2.build();
        stOutput4 = splineToOutput4.build();
        stIntake5 = splineToIntake5.build();
        retract3 = retractEx3.build();
        stOutput5 = splineToOutput5.build();
        par = park.build();

        Actions.runBlocking(
                new ParallelAction(
                        outputwrist(outputWristSpecimenPos),
                        outputarm(deltaLeftSpecimenPos, deltaRightSpecimenPos),
                        outputclaw(outputClawClosedPos),
                        intakeclaw(intakeClawOpenPos),
                        intakearm(alphaTransferPos, betaTransferPos),
                        intakewrist(intakeWristStraightPos),
                        springtoggle(springToggleOffPos)
                )
        );
    }

    @Override
    public void childLoop(){
        super.childLoop();

            Actions.runBlocking(
                    new SequentialAction(
                                    pSpec,
                                    gtFirst,
                                    stOutput2
                    )
            );

                        /*new ParallelAction(
                                stIntake3,
                                reset()
                        ),
                        retract1,
                        new ParallelAction(
                                stOutput3,
                                transfer()
                        ),
                        new ParallelAction(
                                stIntake4,
                                reset()
                        ),
                        retract2,
                        new ParallelAction(
                                stOutput4,
                                transfer()
                        ),
                        new ParallelAction(
                            stIntake5,
                            reset()
                        ),
                        retract3,
                        new ParallelAction(
                                stOutput5,
                                transfer()
                        ),
                        new ParallelAction(
                                par,
                                reset()
                        )*/

    }

}