package org.firstinspires.ftc.teamcode.ours.util;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;

public class AprilTagLocalisationHelpers {
    static Vector2D[] allTags = {
            new Vector2D(-72, 48), //11
            new Vector2D(0, 72),   //12
            new Vector2D(72, 48),  //13
            new Vector2D(72, -48), //14
            new Vector2D(0, -72),  //15
            new Vector2D(-72, -48) //16
    };

    public static double getAverageRotationFromTags(@NonNull List<AprilTagDetection> currentDetections) {
        List<Double> calculatedRotations = new ArrayList<>();

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                double apparentRotation = detection.ftcPose.yaw;
                double trueRotation = 0;

                switch (detection.id) {
                    case 15:
                        trueRotation = 270 - apparentRotation;
                        break;
                    case 11:
                    case 16:
                        trueRotation = 180 - apparentRotation;
                        break;
                    case 13:
                    case 14:
                        trueRotation = 360 - apparentRotation;
                        break;
                    case 12:
                        trueRotation = 90 - apparentRotation;
                        break;
                }
                if(trueRotation>=360){trueRotation -= 360;}
                calculatedRotations.add(trueRotation);
            }
        }

        double totalRotations = 0;
        for(int i = 0; i < calculatedRotations.size(); i++) totalRotations += calculatedRotations.get(i);

        return (totalRotations / calculatedRotations.size());
    }

    public static double[] rotatePoint(double theta, double[] point) {
        theta = Math.toRadians(theta);
        return new double[] {
                (point[0] * Math.cos(theta)) - (point[1] * Math.sin(theta)),
                (point[0] * Math.sin(theta)) + (point[1] * Math.cos(theta))
        };
    }

    public static Vector2d localToGlobal(List<Integer> ids, List<double[]> localCoords){
        double globalX = 0;
        double globalY = 0;

        if (!localCoords.isEmpty() && !ids.isEmpty() ) {
            for (int i = 0; i < ids.size(); i++) {
                globalX += (allTags[ids.get(i) - 11].getX() + localCoords.get(i)[0]);
                globalY += (allTags[ids.get(i) - 11].getY() - localCoords.get(i)[1]);
            }
        }

        globalX /= ids.size();
        globalY /= ids.size();

        return new Vector2d(globalX, globalY);
    }
}
