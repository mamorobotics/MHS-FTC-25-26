package com.example.meepmeeptesting.decode;

import com.acmerobotics.roadrunner.Pose2d;

import javax.imageio.ImageIO;
import java.nio.file.Path;
import java.nio.file.Paths;

import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

public class blueClose {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);
        Vector2d shootingPos = new Vector2d(-6,6);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-60, -34, Math.toRadians(90)))
                        .splineTo(new Vector2d(-12,-10), Math.toRadians(180))
                //SCAN APRIL TAG HERE
                        .waitSeconds(1)
                        .turn(Math.toRadians(90))
                .lineToY(-38)
                //INTAKE HERE
                        .waitSeconds(1)
                                .splineTo(shootingPos, Math.toRadians(225))
                        //SHOOT BALLS HERE
                        .waitSeconds(1)
                        .turn(Math.toRadians(45))
                        .splineTo(new Vector2d(12,-38), Math.toRadians(270))
                        //INTAKE HERE
                        .waitSeconds(1)
                        .turn(Math.toRadians(180))
                        .splineTo(shootingPos, Math.toRadians(225))
                        //SHOOT BALLS HERE
                        .waitSeconds(1)
                        .splineTo(new Vector2d(37, -38), Math.toRadians(270))
                        //INTAKE HERE
                        .waitSeconds(1)
                        .turn(Math.toRadians(180))
                        .splineTo(shootingPos, Math.toRadians(225))
                        //SHOOT BALLS HERE
                        .waitSeconds(1)
                        .turn(Math.toRadians(135))
                        .lineToX(12)

                .build());

//        Image img = null;
//
//        Path relativePath = Paths.get("assets", "decode-custMAth.om-field.png");
//        Path absolutePath = relativePath.toAbsolutePath();
//        System.out.println("Absolute Path to Image: " + absolutePath);
//
//        try { img = ImageIO.read(new File(absolutePath.toString())); }
//        catch(IOException e) {}
//
//
//        meepMeep.setBackground(img)
      meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
