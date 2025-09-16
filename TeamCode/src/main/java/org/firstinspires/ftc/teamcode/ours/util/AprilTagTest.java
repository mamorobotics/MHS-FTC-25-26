package org.firstinspires.ftc.teamcode.ours.util;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "AprilTagTest", group = "Utility")
public class AprilTagTest extends OpMode {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void init() {
        initAprilTag();
    }

    @Override
    public void loop() {
        try {
            telemetryAprilTag();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        visionPortal.resumeStreaming();
        telemetry.update();
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().setLensIntrinsics(973.979f, 973.979f, 314.418f, 262.399f).build();

        aprilTag.setDecimation(3);

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        builder.setCameraResolution(new Size(640, 480));

        builder.enableLiveView(true);
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        builder.setAutoStopLiveView(false);

        builder.addProcessor(aprilTag);

        visionPortal = builder.build();
    }

    private double getAverageRotation(double n) throws InterruptedException {
        double avgRotation = 0;

        for (int i = 0; i < n; i++) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            avgRotation += AprilTagLocalisationHelpers.getAverageRotationFromTags(currentDetections);

            TimeUnit.MILLISECONDS.sleep(17);
        }

        return avgRotation / n;
    }

    private void telemetryAprilTag() throws InterruptedException {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        double currentRotation = getAverageRotation(60);

        telemetry.addData("# AprilTags Detected", currentDetections.size());


        telemetry.addData("Calculated Rotation", currentRotation);
        telemetry.addData("Theta Radians ", Math.toRadians(90 - currentRotation));

        List<double[]> localSpaceLocations = new ArrayList<>();
        List<Integer> ids = new ArrayList<>();
        Vector2d globalVec;
        Pose2d globalPos;

        for (AprilTagDetection detection : currentDetections) {
            localSpaceLocations.add(AprilTagLocalisationHelpers.rotatePoint(90 - currentRotation,
                    new double[] {detection.ftcPose.x, detection.ftcPose.y}));
            ids.add(detection.id);

            double[] x = AprilTagLocalisationHelpers.rotatePoint(90 - currentRotation,
                    new double[] {detection.ftcPose.x, detection.ftcPose.y});

            telemetry.addData("Rot P X", x[0]);
            telemetry.addData("Rot P Y", x[1]);

            telemetry.addData("L Pos X", detection.ftcPose.x);
            telemetry.addData("L Pos Y", detection.ftcPose.y);

        }

        if(currentDetections.size() != 0) {
            globalVec = AprilTagLocalisationHelpers.localToGlobal(ids, localSpaceLocations);
            globalPos = new Pose2d(globalVec, currentRotation);

            telemetry.addData("Calculated Average Global X", globalVec.getX());
            telemetry.addData("Calculated Average Global Y", globalVec.getY());
            telemetry.addData("Calculated Average Global Heading", globalPos.getHeading());
        }
    }
}

