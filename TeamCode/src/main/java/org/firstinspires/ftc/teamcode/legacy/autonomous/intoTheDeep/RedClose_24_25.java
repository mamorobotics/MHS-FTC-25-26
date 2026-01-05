package org.firstinspires.ftc.teamcode.legacy.autonomous.intoTheDeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.legacy.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.legacy.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;


@Autonomous(name = "RedClose_24_25")
public class RedClose_24_25 extends LinearOpMode {
    static DcMotor slideLeft, slideRight, ArmM;
    private static Servo ArmS, ClawS;

    int slideTopPos = 4300;
    int slideBottomPos = 0;
    double clawOpen = 0.05;
    double clawClosed = 0.4;
    int armTransfer = 950;
    int armPickUp = 1740;
    int armDrop  = 300;
    double wristPickUp = 0.64;
    double wristDrop  = 0.25;
    double armAttcSpeed = 0.3;
    double liftAttcSpeed = 1;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    Pose2d startPos = new Pose2d(-64, 12, Math.toRadians(90));

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

        ArmS = hardwareMap.get(Servo.class, "ArmServo");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        drive.setPoseEstimate(startPos);

        telemetry.update();

        ArmM.setTargetPosition(armDrop);
        ArmM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmM.setPower(armAttcSpeed);
        ArmS.setPosition(wristDrop);
        slideRight.setTargetPosition(slideTopPos);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setPower(liftAttcSpeed);
        slideLeft.setTargetPosition(slideTopPos);
        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideLeft.setPower(liftAttcSpeed);

        TrajectorySequence seq1 = drive.trajectorySequenceBuilder(startPos)
                .lineToLinearHeading(new Pose2d(-53,12, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-57,63, Math.toRadians(315)))
                .build();
        drive.followTrajectorySequence(seq1);

        ClawS.setPosition(clawOpen);

        sleep(1000);

        ArmS.setPosition(wristPickUp);
        ArmM.setTargetPosition((armDrop + armPickUp)/2);
        ArmM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmM.setPower(armAttcSpeed);
        slideRight.setTargetPosition(slideBottomPos);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setPower(liftAttcSpeed);
        slideLeft.setTargetPosition(slideBottomPos);
        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideLeft.setPower(liftAttcSpeed);

        startPos = seq1.end();

        for(int i=0; i<2; i++){
            ArmM.setTargetPosition(armPickUp);
            ArmM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmM.setPower(armAttcSpeed);

            TrajectorySequence seq2 = drive.trajectorySequenceBuilder(startPos)
                    .lineToLinearHeading(new Pose2d(-44, 64-(i*10), Math.toRadians(0)))
                    .build();
            drive.followTrajectorySequence(seq2);

            drive.waitForIdle();

            sleep(1000);

            ClawS.setPosition(clawClosed);

            sleep(400);

            slideRight.setTargetPosition(slideTopPos);
            slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideRight.setPower(liftAttcSpeed);
            slideLeft.setTargetPosition(slideTopPos);
            slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideLeft.setPower(liftAttcSpeed);

            TrajectorySequence seq3 = drive.trajectorySequenceBuilder(seq2.end())
                    .lineToLinearHeading(new Pose2d(-60.5-i,63, Math.toRadians(315)))
                    .build();
            drive.followTrajectorySequence(seq3);

            drive.waitForIdle();

            ArmM.setTargetPosition(armDrop);
            ArmM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmM.setPower(armAttcSpeed);
            ArmS.setPosition(wristDrop);


            while(ArmM.getCurrentPosition() != armDrop) {
                sleep(1);
            }

            ClawS.setPosition(clawOpen);

            sleep(200);

            ArmS.setPosition(wristPickUp);
            slideRight.setTargetPosition(slideBottomPos);
            slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideRight.setPower(liftAttcSpeed);
            slideLeft.setTargetPosition(slideBottomPos);
            slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideLeft.setPower(liftAttcSpeed);

            startPos = seq3.end();
        }

        ArmM.setTargetPosition(armTransfer);
        ArmM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmM.setPower(armAttcSpeed);
        ArmS.setPosition(wristDrop);
        slideRight.setTargetPosition(slideBottomPos);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setPower(liftAttcSpeed);
        slideLeft.setTargetPosition(slideBottomPos);
        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideLeft.setPower(liftAttcSpeed);

        TrajectorySequence seq4 = drive.trajectorySequenceBuilder(startPos)
                .lineToLinearHeading(new Pose2d(-10.5,36, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(-10.5,27, Math.toRadians(270)))
                .build();
        drive.followTrajectorySequence(seq4);
        ArmM.setTargetPosition(1035);
        ArmM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmM.setPower(armAttcSpeed * 0.8);

        sleep(2000);
        ClawS.setPosition(clawOpen);
    }
}