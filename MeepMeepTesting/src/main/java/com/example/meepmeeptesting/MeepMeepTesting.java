package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;

import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Trajectory;

import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);
        //Trajectory =


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-8, 62.5, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(0, 31))
                .setTangent(Math.toRadians(95))
                .splineToConstantHeading(new Vector2d(-33, 31), Math.toRadians(270))

                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-44, 10), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(-44, 55))

                .setTangent(Math.toRadians(290))
                .splineToConstantHeading(new Vector2d(-53, 10), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(-53, 53))

                .setTangent(Math.toRadians(290))
                .splineToConstantHeading(new Vector2d(-62, 10), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(-62, 53))

                .setTangent(Math.toRadians(300))
                .splineToLinearHeading(new Pose2d(-22, 46, Math.toRadians(135)), Math.toRadians(15))
                .splineToLinearHeading(new Pose2d(-7, 33, Math.toRadians(120)), Math.toRadians(280))

                .setTangent(90)
                .splineToLinearHeading(new Pose2d(-22, 46, Math.toRadians(135)), Math.toRadians(135))
                .setTangent(270)
                .splineToLinearHeading(new Pose2d(-7, 33, Math.toRadians(120)), Math.toRadians(280))

                .setTangent(90)
                .splineToLinearHeading(new Pose2d(-22, 46, Math.toRadians(135)), Math.toRadians(135))
                .setTangent(270)
                .splineToLinearHeading(new Pose2d(-7, 33, Math.toRadians(120)), Math.toRadians(280))

                .setTangent(90)
                .splineToLinearHeading(new Pose2d(-22, 46, Math.toRadians(135)), Math.toRadians(135))
                .setTangent(270)
                .splineToLinearHeading(new Pose2d(-7, 33, Math.toRadians(120)), Math.toRadians(280))
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-60, 60, Math.toRadians(270)), Math.toRadians(90))


                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}