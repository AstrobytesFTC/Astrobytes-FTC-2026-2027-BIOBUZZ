package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import javax.imageio.ImageIO;
import java.awt.Image;
import java.io.IOException;
import java.util.Objects;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-63, -60, Math.toRadians(0)))
//               //blue side auto path
//               myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-63, 16, Math.toRadians(0)))                .lineToX(-53)
//                .strafeTo(new Vector2d(-21,52))

                .strafeTo(new Vector2d(-50,-60))




                //red side auto path
//                 myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(63, -16, Math.toRadians(180)))
//                  .lineToX(53)
//                  .strafeTo(new Vector2d(21,-52))
//                  .strafeTo(new Vector2d(-32,-55))

                  //same path but spline
//                .strafeTo(new Vector2d(21, -50))
//                .splineToLinearHeading(new Pose2d(-32, -55, Math.toRadians(180)), Math.toRadians(180))





//                .turn(Math.toRadians(90))
//                .lineToX(0)
//                .turn(Math.toRadians(90))
//                .lineToY(0)
//                .turn(Math.toRadians(90))
                .build());

        try {
            Image biobuzzfield = ImageIO.read(
                    Objects.requireNonNull(MeepMeepTesting.class.getResourceAsStream("/biobuzzfield.jpeg"))
            );
            meepMeep.setBackground(biobuzzfield);
        } catch (IOException e) {
            e.printStackTrace();
        }

        meepMeep
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

