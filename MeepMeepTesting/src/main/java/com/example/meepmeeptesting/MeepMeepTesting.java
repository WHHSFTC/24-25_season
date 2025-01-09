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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-7.1791, 65.21, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-6, 37), Math.toRadians(120))
                .strafeToLinearHeading(new Vector2d(-6, 43), Math.toRadians(97))
                .strafeToConstantHeading(new Vector2d(-6, 48))

                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-32, 39), Math.toRadians(270))

                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-32, 39), Math.toRadians(270))
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-40, 15), Math.toRadians(97))
                .splineToConstantHeading(new Vector2d(-40, 48), Math.toRadians(290))
                .setTangent(Math.toRadians(290))
                .splineToConstantHeading(new Vector2d(-50, 15), Math.toRadians(97))
                .splineToConstantHeading(new Vector2d(-50, 48), Math.toRadians(290))
                .setTangent(Math.toRadians(290))
                .splineToConstantHeading(new Vector2d(-60, 15), Math.toRadians(97))
                .splineToConstantHeading(new Vector2d(-60, 47), Math.toRadians(290))

                .setTangent(Math.toRadians(290))
                .splineToLinearHeading(new Pose2d(-22, 46, Math.toRadians(135)), Math.toRadians(15))

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