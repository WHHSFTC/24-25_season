package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.Localizer;
import org.firstinspires.ftc.teamcode.roadrunner.tuning.TuningOpModes;

import org.opencv.core.Mat;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous
public class five_specimen extends OpMode{

    MecanumDrive drive;
    TrajectoryActionBuilder preloadSpecimen;
    TrajectoryActionBuilder getToFirst;
    TrajectoryActionBuilder getToSecond;
    TrajectoryActionBuilder getToThird;
    TrajectoryActionBuilder transition;
    TrajectoryActionBuilder splineToOutput;
    TrajectoryActionBuilder splineToIntake;
    TrajectoryActionBuilder park;

    Action pSpec;
    Action gtFirst;
    Action gtSecond;
    Action gtThird;
    Action trans;
    Action stOutput;
    Action stIntake;
    Action par;

    public void init(){
        Pose2d startPos = new Pose2d(-7.1791, 65.21, Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap, startPos);

        preloadSpecimen = drive.actionBuilder(startPos)
                .strafeToConstantHeading(new Vector2d(0, 31));

        getToFirst = drive.actionBuilder(new Pose2d(0, 31, Math.toRadians(90)))
                .setTangent(Math.toRadians(95))
                .splineToConstantHeading(new Vector2d(-33, 31), Math.toRadians(270))
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-44, 10), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(-44, 55));

        getToSecond = drive.actionBuilder(new Pose2d(-44, 55, Math.toRadians(90)))
                .setTangent(Math.toRadians(290))
                .splineToConstantHeading(new Vector2d(-53, 10), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(-53, 53));

        getToThird = drive.actionBuilder(new Pose2d(-53, 53, Math.toRadians(90)))
                .setTangent(Math.toRadians(290))
                .splineToConstantHeading(new Vector2d(-62, 10), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(-62, 53));

        transition = drive.actionBuilder(new Pose2d(-62,53,Math.toRadians(90)))
                .setTangent(Math.toRadians(300))
                .splineToLinearHeading(new Pose2d(-22, 46, Math.toRadians(135)), Math.toRadians(15));

        splineToOutput = drive.actionBuilder(new Pose2d(-22, 46, Math.toRadians(135)))
                .setTangent(270)
                .splineToLinearHeading(new Pose2d(-7, 33, Math.toRadians(120)), Math.toRadians(280));

        splineToIntake = drive.actionBuilder(new Pose2d(-7, 33, Math.toRadians(120)))
                .setTangent(90)
                .splineToLinearHeading(new Pose2d(-22, 46, Math.toRadians(135)), Math.toRadians(135));

        park = drive.actionBuilder(new Pose2d(-7, 33, Math.toRadians(120)))
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-60, 60, Math.toRadians(270)), Math.toRadians(90));

        pSpec = preloadSpecimen.build();
        gtFirst = getToFirst.build();
        gtSecond = getToSecond.build();
        gtThird = getToThird.build();
        trans = transition.build();
        stOutput = splineToOutput.build();
        stIntake = splineToIntake.build();
        par = park.build();
    }

    public void loop(){
        Actions.runBlocking(
                new SequentialAction(
                        pSpec,
                        gtFirst,
                        gtSecond,
                        gtThird,
                        trans,
                        stOutput,
                        stIntake,
                        stOutput,
                        stIntake,
                        stOutput,
                        stIntake,
                        stOutput,
                        par
                )
        );
    }

    public void stop(){

    }
}
