package org.firstinspires.ftc.teamcode.legacy.autonomous.centerStage;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.legacy.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.legacy.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.legacy.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;


public class BlueClose extends LinearOpMode {

    static DcMotor scoop;

    double botWidth = 14.25 / 2.0;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    Pose2d startpos = new Pose2d(72 - botWidth, 72 - 10, Math.toRadians(270));


    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    //private TfodProcessor tfod;

    private static final String[] LABELS = {
            "dumbell",
    };

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    private int minConfidenceValue = 90;
    @Override
    public void runOpMode() throws InterruptedException {
        int scoopTopPos = 1565;
        int scoopBottomPos = 512;
        int scoopLiftPos = 700;
        int scoopBLiftPos = 180;
        double attcSpeed = 0.15;

        scoop = hardwareMap.get(DcMotorEx.class, "scoop");
        scoop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        scoop.setDirection(DcMotorSimple.Direction.REVERSE);
        scoop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Vector2d centerPoint = null;

        waitForStart();

        scoop.setTargetPosition(scoopBottomPos);
        scoop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        scoop.setPower(attcSpeed);

        if (centerPoint != null) {
            if (centerPoint.getX() < 460) {
                telemetry.addData("left", "");
            } else if (centerPoint.getX() >= 460) {
                telemetry.addData("center", "");
            } else {
                telemetry.addData("right", "");
            }
        }
        telemetry.update();

        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(270)));
        Trajectory left = drive.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(270)))
                .strafeLeft(57, drive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        drive.followTrajectory(left);

        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(270)));
        Trajectory back = drive.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(270)))
                .back(4, drive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        drive.followTrajectory(back);

        TrajectorySequence seq0;

        drive.setPoseEstimate(startpos);

        TrajectorySequence seq3;
        seq3 = drive.trajectorySequenceBuilder(startpos)
                .lineToLinearHeading(new Pose2d(48, 72 - botWidth - 6, Math.toRadians(270)))
                .build();
        drive.followTrajectorySequence(seq3);

        if (centerPoint == null) {
            telemetry.addData("right", "");
            telemetry.update();
            seq0 = drive.trajectorySequenceBuilder(seq3.end())
                    .lineToLinearHeading(new Pose2d(12, 36, Math.toRadians(180)))
                    .build();
            drive.followTrajectorySequence(seq0);
        } else if (centerPoint.getX() < 380) {
            telemetry.addData("left", "");
            telemetry.update();
            seq0 = drive.trajectorySequenceBuilder(seq3.end())
                    .lineToLinearHeading(new Pose2d(24 + botWidth, 41, Math.toRadians(225)))
                    .build();
            drive.followTrajectorySequence(seq0);
        } else{
            telemetry.addData("center", "");
            telemetry.update();
            seq0 = drive.trajectorySequenceBuilder(seq3.end())
                    .lineToLinearHeading(new Pose2d(16, 34, Math.toRadians(270)))
                    .build();
            drive.followTrajectorySequence(seq0);
        }

        scoop.setTargetPosition(0);
        scoop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        scoop.setPower(attcSpeed);
        while(scoop.getCurrentPosition() != scoop.getTargetPosition()){}

        telemetry.update();

        TrajectorySequence seq1 = drive.trajectorySequenceBuilder(seq0.end())
                .lineToLinearHeading(new Pose2d(48, 72 - botWidth - 6, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(72 - botWidth, 72 - botWidth - 6, Math.toRadians(180)))
                .build();
        drive.followTrajectorySequence(seq1);
    }
}