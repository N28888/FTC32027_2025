package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-12, -61, 0))
                        .turn(Math.toRadians(90))
                        .forward(80)
                        .turn(Math.toRadians(45))
                        .build());

        String osName = System.getProperty("os.name");
        System.out.println(osName);

        if (osName.startsWith("Mac OS")) {
            Image img = null;
            try {
                img = ImageIO.read(new File("/Users/yifanjin/Documents/Github/AC_robot_2026/MeepMeepTesting/src/picture/2026_map.png"));
            } catch (IOException ignored) {
            }

            assert img != null;
            meepMeep.setBackground(img)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(myBot)
                    .start();
        } else if (osName.startsWith("Windows")) {
            Image img = null;
            try {
                img = ImageIO.read(new File("C:Users\\JYF28\\Documents\\Github\\AC_robot_2026\\MeepMeepTesting\\src\\picture\\2026_map.png"));
            } catch (IOException ignored) {
            }

            assert img != null;
            meepMeep.setBackground(img)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(myBot)
                    .start();
        }

    }
}