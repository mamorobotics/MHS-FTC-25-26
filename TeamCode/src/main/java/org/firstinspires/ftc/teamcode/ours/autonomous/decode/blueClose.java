package org.firstinspires.ftc.teamcode.ours.autonomous.decode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;      // RR 0.5.x geometry
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;    // RR 0.5.x geometry
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ours.BotConstants;




import org.firstinspires.ftc.teamcode.ours.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.ours.drive.Subsystems;
import org.firstinspires.ftc.teamcode.ours.teleop.DriveTrain;


@Autonomous(name = "Blue Close Decode")
public class blueClose extends LinearOpMode {


    static DriveTrain driveTrain = new DriveTrain();

    // Flywheel:

    private static ElapsedTime stopwatch = new ElapsedTime();

    private static DcMotorEx FlyL, FlyR;
    private static CRServo IntakeBrushL, IntakeBrushR, TransferL, TransferR;
    private static Servo Gate, RampL, RampR;






    @Override
    public void runOpMode() throws InterruptedException {
        FlyL = hardwareMap.get(DcMotorEx.class, "leftLauncher");
        FlyR = hardwareMap.get(DcMotorEx.class, "rightLauncher");
        FlyL.setDirection(DcMotor.Direction.REVERSE);
        FlyR.setDirection(DcMotor.Direction.FORWARD);
        FlyL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FlyR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FlyL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FlyR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FlyL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, BotConstants.flywheelCoefficients);
        FlyR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, BotConstants.flywheelCoefficients);

        IntakeBrushL = hardwareMap.get(CRServo.class, "leftIntake");
        IntakeBrushR = hardwareMap.get(CRServo.class, "rightIntake");
        IntakeBrushL.setDirection(CRServo.Direction.FORWARD);
        IntakeBrushR.setDirection(CRServo.Direction.REVERSE);

        TransferL = hardwareMap.get(CRServo.class, "leftTransfer");
        TransferR = hardwareMap.get(CRServo.class, "rightTransfer");
        TransferL.setDirection(CRServo.Direction.FORWARD);
        TransferR.setDirection(CRServo.Direction.REVERSE);

        Gate = hardwareMap.get(Servo.class, "gate");
        Gate.setPosition(BotConstants.GATE_DOWN_POS);

        RampR = hardwareMap.get(Servo.class, "rightRamp");
        RampL = hardwareMap.get(Servo.class, "leftRamp");

        RampL.setDirection(Servo.Direction.REVERSE);

        startMechs();



        telemetry.addLine("Autonomous Initialized");
        telemetry.update();

        Pose2d startPose = new Pose2d(-60, -34, Math.toRadians(0));
        Pose2d shootingPose = new Pose2d(-18, -16, Math.toRadians(225));
        Pose2d closeRowPose = new Pose2d(-12, -30, Math.toRadians(270));
        Pose2d middleRowPose = new Pose2d(12, -30, Math.toRadians(270));
        Pose2d farRowPose = new Pose2d(37, -30, Math.toRadians(270));

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        Vector2d shootingPos = new Vector2d(-18, -16);
        Vector2d closeRow   = new Vector2d(-12, -30);
        Vector2d middleRow  = new Vector2d(12, -30);
        Vector2d farRow     = new Vector2d(37, -30);



        Action seq1 = drive.actionBuilder(startPose)
                //.waitSeconds(18)
                .strafeToLinearHeading(shootingPos,Math.toRadians(225))
                //.strafeToLinearHeading(new Vector2d(-16,-45), Math.toRadians(0))
                //SCAN APRIL TAG HERE
                .build();

        //SHOOT
        Action seq12 = drive.actionBuilder(shootingPose)
                .strafeToLinearHeading(new Vector2d(-16,-45), Math.toRadians(0))
                .build();

        //INTAKE

        Action seqIntakeForward1 = drive.actionBuilder(closeRowPose)
                .lineToY(-52)
                .build(); // FILL THIS PART IN

        Action seq2 = drive.actionBuilder(new Pose2d(-12,-52,Math.toRadians(270)))
                .turn(Math.toRadians(-45))
                .strafeTo(shootingPos)
                .build();

        //SHOOT

        Action seq3 = drive.actionBuilder(shootingPose)
                .turn(Math.toRadians(45))
                .splineTo(middleRow, Math.toRadians(270))
                .build();

        //INTAKE

        Action seqIntakeForward2 = drive.actionBuilder(middleRowPose)
                .lineToY(-52)
                .build(); // FILL THIS PART IN; // FILL THIS PART IN

        Action seq4 = drive.actionBuilder(new Pose2d(12,-52,Math.toRadians(270)))
                .turn(Math.toRadians(-45))
                .strafeTo(shootingPos)
                .build();

        //SHOOT

        //Action seq5 = drive.actionBuilder(shootingPose)
          //      .splineTo(farRow, Math.toRadians(270))
          //      .build();

        //INTAKE

        //TrajectoryActionBuilder blueClose = drive.actionBuilder(startPose)



        while (!isStarted() && !isStopRequested()) {

            telemetry.addLine("BlueCloseAuto ready");
            telemetry.update();
        }

        if (isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(
                seq1

        ));

        launch(1000, 0.41);

        Actions.runBlocking(new SequentialAction(
                seq12
        ));



//        drive.followTrajectorySequence(blueClose);

/*        Action trajectoryChosen = blueClose.build();

        Actions.runBlocking(
            new SequentialAction(
                    trajectoryChosen
            )
        );
*/
        while (opModeIsActive()) {
            idle();
        }
    }

    public void launch(double vel, double angle) {
        FlyL.setVelocity(vel);
        FlyR.setVelocity(vel);

        RampL.setPosition(angle);
        RampR.setPosition(angle);




        stopwatch.reset();

        while(stopwatch.time() < 4) {
            TransferL.setPower(0);
            TransferR.setPower(0);
        }

        Gate.setPosition(BotConstants.GATE_DOWN_POS);

        TransferL.setPower(0.15);
        TransferR.setPower(0.15);

        stopwatch.reset();

        while(stopwatch.time() < 3) {
            TransferL.setPower(0.15);
            TransferR.setPower(0.15);
        }

        Gate.setPosition(BotConstants.GATE_UP_POS);
        TransferL.setPower(0);
        TransferR.setPower(0);

        while(stopwatch.time() < 4) {
            TransferL.setPower(0);
            TransferR.setPower(0);
        }


        Gate.setPosition(BotConstants.GATE_DOWN_POS);

        TransferL.setPower(0.15);
        TransferR.setPower(0.15);

        IntakeBrushL.setPower(0.15);
        IntakeBrushR.setPower(0.15);

        stopwatch.reset();

        while(stopwatch.time() < 3) {
            TransferL.setPower(0.15);
            TransferR.setPower(0.15);
        }


    }


    public void startMechs() {
        Gate.setPosition(BotConstants.GATE_UP_POS);
    }


    public void intake() {
        Gate.setPosition(BotConstants.GATE_UP_POS);
        IntakeBrushL.setPower(0.4);
        IntakeBrushR.setPower(0.4);

        TransferL.setPower(0.15);
        TransferR.setPower(0.15);
    }
}

/*

subsystems.launchAll(shootingPose),
                seq12,
                new ParallelAction(
                        subsystems.Intake(),
                        seqIntakeForward1
                ),
                seq2,
                subsystems.launchAll(shootingPose),
                seq3,
                new ParallelAction(
                        subsystems.Intake(),
                        seqIntakeForward2
                ),
                seq4,
                subsystems.launchAll(shootingPose)
 */
