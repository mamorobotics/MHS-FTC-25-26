package org.firstinspires.ftc.teamcode.ours.autonomous.centerStage;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

public class BlueFar extends LinearOpMode {

    static DcMotor slideLeft;
    static DcMotor slideRight;

    int slideTopPos = 1565;
    int slideBottomPos = 512;
    int slideMidPos = 700;
    int scoopBLiftPos  = 180;
    double attcSpeed = 0.3;


    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    Pose2d startPos = new Pose2d(12, 72-8, Math.toRadians(270));

    @Override
    public void runOpMode() throws InterruptedException {
        slideLeft = hardwareMap.get(DcMotorEx.class, "slideLeft");
        slideRight = hardwareMap.get(DcMotorEx.class, "slideRight");
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setDirection(DcMotorSimple.Direction.FORWARD);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        drive.setPoseEstimate(startPos);

        telemetry.update();

        //.lineToLinearHeading(new Pose2d(-60, 60, Math.toRadians(180)))
//                .lineToLinearHeading(new Pose2d(-60, 36, Math.toRadians(180)))
//                .lineToLinearHeading(new Pose2d(-60, 60, Math.toRadians(180)))
//                .lineToSplineHeading(new Pose2d(-6,36, Math.toRadians(270)))
//                .lineToLinearHeading(new Pose2d(-60, 42, Math.toRadians(270)))
//                .lineToLinearHeading(new Pose2d(-6,36, Math.toRadians(270)))
//                .lineToLinearHeading(new Pose2d(-60, 42, Math.toRadians(270)))
//                .lineToLinearHeading(new Pose2d(-6,36, Math.toRadians(270)))
//                .lineToLinearHeading(new Pose2d(-60, 42, Math.toRadians(270)))
//                .lineToLinearHeading(new Pose2d(-6,36, Math.toRadians(270)))

        TrajectorySequence seq1 = drive.trajectorySequenceBuilder(startPos)
                .lineToLinearHeading(new Pose2d(-60, 60, Math.toRadians(180)))
                .build();
        drive.followTrajectorySequence(seq1);

        slideRight.setTargetPosition(slideTopPos);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setPower(attcSpeed);
        slideLeft.setTargetPosition(slideTopPos);
        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideLeft.setPower(attcSpeed);

        for(int i=0; i<2; i++){
            TrajectorySequence seq2 = drive.trajectorySequenceBuilder(startPos)
                    .lineToLinearHeading(new Pose2d(60, 42, Math.toRadians(270)))
                    .build();
            drive.followTrajectorySequence(seq2);

            slideRight.setTargetPosition(slideTopPos);
            slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideRight.setPower(attcSpeed);
            slideLeft.setTargetPosition(slideTopPos);
            slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideLeft.setPower(attcSpeed);

            TrajectorySequence seq3 = drive.trajectorySequenceBuilder(startPos)
                    .lineToLinearHeading(new Pose2d(54,54, Math.toRadians(45)))
                    .build();
            drive.followTrajectorySequence(seq2);

            slideRight.setTargetPosition(slideBottomPos);
            slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideRight.setPower(attcSpeed);
            slideLeft.setTargetPosition(slideBottomPos);
            slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideLeft.setPower(attcSpeed);
        }
    }
}