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
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
@Config
@Autonomous (preselectTeleOp = "intothedeep_tele_blue")
public class five_specimen_blue extends intothedeep_auto{
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
    TrajectoryActionBuilder moveforward;
    TrajectoryActionBuilder moveforward2;
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
    Action mf;
    Action mf2;
    Action par;
    public void init(){
        super.init();
        Pose2d startPos = new Pose2d(-7.1791, 65.21, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPos);
        splineToOutput2 = drive.actionBuilder(new Pose2d(-20, 52, Math.toRadians(145)))
                .setTangent(270)
                .splineToLinearHeading(new Pose2d(-6, 40, Math.toRadians(120)), Math.toRadians(280))
                .afterTime(0.1, vertslides(900, false));
        stOutput2 = splineToOutput2.build();
        moveforward = drive.actionBuilder(new Pose2d(-6, 40, Math.toRadians(120)))
                .strafeToConstantHeading(new Vector2d(-6, 46))
                .afterTime(0.1, outputclaw(outputClawOpenPos));
        mf = moveforward.build();
        splineToIntake3 = drive.actionBuilder(new Pose2d(-6, 46, Math.toRadians(120)))
                .setTangent(90)
                .splineToLinearHeading(new Pose2d(-20, 52, Math.toRadians(145)), Math.toRadians(135))
                .afterTime(0.2, new IntakeArm(alphaLowerPos, betaLowerPos))
                .afterTime(0.5, new IntakeClaw(intakeClawClosedPos))
                .afterTime(1.2, new IntakeArm(alphaTransferPos, betaTransferPos));
        stIntake3 = splineToIntake3.build();
        splineToOutput3 = drive.actionBuilder(new Pose2d(-20, 52, Math.toRadians(145)))
                .setTangent(270)
                .splineToLinearHeading(new Pose2d(-4, 40, Math.toRadians(120)), Math.toRadians(280))
                .afterTime(0.1, vertslides(900, false));
        stOutput3 = splineToOutput3.build();
        moveforward2 = drive.actionBuilder(new Pose2d(-4, 40, Math.toRadians(120)))
                .strafeToConstantHeading(new Vector2d(-4, 46))
                .afterTime(0.1, outputclaw(outputClawOpenPos));
        mf2 = moveforward.build();
        park = drive.actionBuilder(new Pose2d(-4, 46, Math.toRadians(120)))
                .strafeToLinearHeading(new Vector2d(-28, 66), Math.toRadians(150));
        par = park.build();
        preloadSpecimen = drive.actionBuilder(startPos)
                .strafeToLinearHeading(new Vector2d(-6, 37), Math.toRadians(120))
                .afterTime(0.15, vertslides(900, false));
        getToFirst = drive.actionBuilder(new Pose2d(-6, 37, Math.toRadians(120)))
                .strafeToLinearHeading(new Vector2d(-6, 43), Math.toRadians(97))
                .afterTime(0.15, new OutputClaw(outputClawOpenPos))
                .strafeToConstantHeading(new Vector2d(-6, 48))
                .afterTime(0.7, new ParallelAction(outputarm(deltaLeftPreTransfer,deltaRightPreTransfer), outputwrist(outputWristStraightPos)))
                .afterTime(0.8, vertslides(slideMinVe, true))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-34, 39), Math.toRadians(270))
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-44, 15), Math.toRadians(97))
                .strafeToConstantHeading(new Vector2d(-44, 61))
                .afterTime(0.1, vertslides(slideMinVe, true))
                .setTangent(Math.toRadians(290))
                .splineToConstantHeading(new Vector2d(-54, 15), Math.toRadians(97))
                .strafeToConstantHeading(new Vector2d(-52, 61))
                .setTangent(Math.toRadians(300))
                .splineToLinearHeading(new Pose2d(-20, 52, Math.toRadians(145)), Math.toRadians(15))
                .afterTime(0.1, extendslides(slideMaxEx, false))
                .afterTime(0.4, intakearm(alphaLowerPos, betaLowerPos))
                .afterTime(0.7, intakeclaw(intakeClawClosedPos))
                .afterTime(1.3, intakearm(alphaTransferPos, betaTransferPos))
                .afterTime(1.5, new ExtendoSlides(slideMinEx, true))
                .afterTime(1.9, outputarm(deltaLeftTransferPos, deltaRightTransferPos))
                .afterTime(2.1, outputclaw(outputClawClosedPos))
                .afterTime(2.4, intakeclaw(intakeClawOpenPos))
                .afterTime(2.8, new ParallelAction(outputarm(deltaLeftSpecimenPos, deltaRightSpecimenPos), outputwrist(outputWristSpecimenPos)))
                .afterTime(3.2, stOutput2)
                .afterTime(5.5, mf)
                .afterTime(6.3, new ParallelAction(outputarm(deltaLeftPreTransfer, deltaRightPreTransfer),
                        outputwrist(outputWristStraightPos), vertslides(slideMinVe, true),
                        extendslides(slideMaxEx, false)))
                .afterTime(7.8, stIntake3)
                .afterTime(10.2, new ParallelAction(extendslides(slideMinEx, true), vertslides(slideMinVe, true)))
                .afterTime(10.6, outputarm(deltaLeftTransferPos, deltaRightTransferPos))
                .afterTime(10.8, outputclaw(outputClawClosedPos))
                .afterTime(11.1, intakeclaw(intakeClawOpenPos))
                .afterTime(11.5, new ParallelAction(outputarm(deltaLeftSpecimenPos, deltaRightSpecimenPos), outputwrist(outputWristSpecimenPos)))
                .afterTime(11.9, stOutput3)
                .afterTime(14.2, mf2)
                .afterTime(14.6, new ParallelAction(outputarm(deltaLeftPreTransfer, deltaRightPreTransfer),
                        outputwrist(outputWristStraightPos), vertslides(slideMinVe, false),
                        extendslides(slideMaxEx, false)))
                .afterTime(14.8, par);
        pSpec = preloadSpecimen.build();
        gtFirst = getToFirst.build();
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
                        gtFirst
                )
        );
    }
}