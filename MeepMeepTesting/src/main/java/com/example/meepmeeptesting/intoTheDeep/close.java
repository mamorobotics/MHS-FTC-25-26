package com.example.meepmeeptesting.intoTheDeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import org.rowlandhall.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class close {
    public static void main(String[] args)
    {
        MeepMeep meepMeep = new MeepMeep(600);

        //blue
        RoadRunnerBotEntity bot1 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(30, 30, 39.66141269883581, Math.toRadians(180), 4.6)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(64, -12, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(53,-12, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(57,-63, Math.toRadians(135)))
                                // loop
                                // loop iteration 0
                                .lineToLinearHeading(new Pose2d(44, -64+(0*10), Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(60.5+(0.5*0),-62.5-(0.5*0), Math.toRadians(135)))
                                // loop iteration 1
                                .lineToLinearHeading(new Pose2d(44, -64+(1*10), Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(60.5+(0.5*1),-62.5-(0.5*1), Math.toRadians(135)))
                                // end loop
                                .lineToLinearHeading(new Pose2d(10.5,-36, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(10.5,-27, Math.toRadians(90)))
                                .build()
                );

        //red
        RoadRunnerBotEntity bot2 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(30, 30, 39.66141269883581, Math.toRadians(180), 4.6)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-12,60,Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-60, 60, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-60, 36, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-60, 60, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-6,36, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(-60, 42, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(-6,36, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(-60, 42, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(-6,36, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(-60, 42, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(-6,36, Math.toRadians(270)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot1)
                //.addEntity(bot2)
                .start();
    }
}