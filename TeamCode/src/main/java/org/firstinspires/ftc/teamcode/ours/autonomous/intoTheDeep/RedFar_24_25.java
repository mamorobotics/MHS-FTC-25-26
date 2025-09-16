package org.firstinspires.ftc.teamcode.ours.autonomous.intoTheDeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;


@Autonomous(name = "RedFar_24_25")
public class RedFar_24_25 extends LinearOpMode {

    static DcMotor slideLeft, slideRight, ArmM;
    private static Servo ArmS, ClawS;

    int slideTopPos = 2156;
    int slideClipPos = 1900;
    int slideBottomPos = 0;
    int armSamplePickUp = 1700;
    int armTransfer = 950;
    int armPickUp = 1740;
    int armDrop = 166;

    double clawOpen = 0.05;
    double clawClosed = 0.4;
    double wristPickUp = 0.64;
    double wristDrop  = 0.43;
    double wristSamplePickUp = 0.76;
    double armAttcSpeed = 0.3;
    double liftAttcSpeed = 1;


    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    Pose2d startPos = new Pose2d(-63, -12, Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {
        ClawS = hardwareMap.get(Servo.class, "ClawServo");
        ClawS.setPosition(clawClosed);

        slideLeft = hardwareMap.get(DcMotorEx.class, "LiftMotorLeft");
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        slideRight = hardwareMap.get(DcMotorEx.class, "LiftMotorRight");
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setDirection(DcMotorSimple.Direction.REVERSE);

        ArmM = hardwareMap.get(DcMotor.class, "ArmMotor");
        ArmM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        ArmS = hardwareMap.get(Servo.class, "ArmServo");

        waitForStart();

        drive.setPoseEstimate(startPos);

        telemetry.update();

        slideRight.setTargetPosition(slideTopPos);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setPower(liftAttcSpeed);
        slideLeft.setTargetPosition(slideTopPos);
        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideLeft.setPower(liftAttcSpeed);
        ArmM.setTargetPosition(armDrop);
        ArmM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmM.setPower(armAttcSpeed);

        sleep(500);

        ArmS.setPosition(wristDrop);

        TrajectorySequence seq2 = drive.trajectorySequenceBuilder(startPos)
                .lineToLinearHeading(new Pose2d(-34.5, 0, Math.toRadians(180)))
                .build();
        drive.followTrajectorySequence(seq2);

        drive.waitForIdle();

        slideRight.setTargetPosition(slideClipPos);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setPower(liftAttcSpeed);
        slideLeft.setTargetPosition(slideClipPos);
        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideLeft.setPower(liftAttcSpeed);

        while(slideRight.getCurrentPosition() <= slideClipPos + 10) {
            sleep(1);
        }

        sleep(100);

        ClawS.setPosition(clawOpen);

        sleep(500);

        slideRight.setTargetPosition(slideBottomPos);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setPower(liftAttcSpeed);
        slideLeft.setTargetPosition(slideBottomPos);
        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideLeft.setPower(liftAttcSpeed);
        ArmS.setPosition(wristPickUp);

        sleep(500);

        TrajectorySequence seq3 = drive.trajectorySequenceBuilder(seq2.end())
                .lineToLinearHeading(new Pose2d(-50, -49, Math.toRadians(0)))
                .build();
        drive.followTrajectorySequence(seq3);

        drive.waitForIdle();

        ArmM.setTargetPosition(armPickUp);
        ArmM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmM.setPower(armAttcSpeed);

        while(ArmM.getCurrentPosition() != armPickUp) {
            sleep(1);
        }

        ClawS.setPosition(clawClosed);

        sleep(500);

        ArmM.setTargetPosition(armPickUp - 200);
        ArmM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmM.setPower(armAttcSpeed);

        sleep(250);

        TrajectorySequence seq4 = drive.trajectorySequenceBuilder(seq3.end())
                .lineToLinearHeading(new Pose2d(-45, -55, Math.toRadians(180)))
                .build();
        drive.followTrajectorySequence(seq4);

        drive.waitForIdle();

        ClawS.setPosition(clawOpen);
        ArmM.setTargetPosition(armTransfer);
        ArmM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmM.setPower(armAttcSpeed);

        sleep(4000);

        ArmS.setPosition(wristSamplePickUp);
        ArmM.setTargetPosition(armSamplePickUp);
        ArmM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmM.setPower(armAttcSpeed);

        while(ArmM.getCurrentPosition() != armSamplePickUp) {
            sleep(1);
        }

        ClawS.setPosition(clawClosed);

        sleep(1000);

        slideRight.setTargetPosition(slideTopPos);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setPower(liftAttcSpeed);
        slideLeft.setTargetPosition(slideTopPos);
        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideLeft.setPower(liftAttcSpeed);
        ArmM.setTargetPosition(armDrop);
        ArmM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmM.setPower(armAttcSpeed);
        ArmS.setPosition(wristDrop);

        TrajectorySequence seq5 = drive.trajectorySequenceBuilder(seq4.end())
                .lineToLinearHeading(new Pose2d(-39.5, -5, Math.toRadians(180)))
                .build();
        drive.followTrajectorySequence(seq5);

        drive.waitForIdle();

        sleep(500);

        slideRight.setTargetPosition(slideClipPos);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setPower(liftAttcSpeed);
        slideLeft.setTargetPosition(slideClipPos);
        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideLeft.setPower(liftAttcSpeed);

        while(slideRight.getCurrentPosition() != slideClipPos) {
            sleep(1);
        }

        sleep(500);

        ClawS.setPosition(clawOpen);
        ArmS.setPosition(0.85);
        slideRight.setTargetPosition(slideBottomPos);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setPower(liftAttcSpeed);
        slideLeft.setTargetPosition(slideBottomPos);
        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideLeft.setPower(liftAttcSpeed);
        ArmM.setTargetPosition(0);
        ArmM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmM.setPower(armAttcSpeed);

        sleep(500);

        TrajectorySequence seq6 = drive.trajectorySequenceBuilder(seq5.end())
                .lineToLinearHeading(new Pose2d(-63,-48, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-68,-48, Math.toRadians(0)))
                .build();
        drive.followTrajectorySequence(seq6);
    }
}