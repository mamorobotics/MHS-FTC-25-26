

package org.firstinspires.ftc.teamcode.ours.autonomous.decode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ours.BotConstants;
import org.firstinspires.ftc.teamcode.ours.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.ours.drive.Subsystems;
import org.firstinspires.ftc.teamcode.ours.teleop.DriveTrain;


@Autonomous(name = "Blue Far Decode")
public class blueFarMishaTest extends LinearOpMode {
    static DriveTrain driveTrain = new DriveTrain();
    private static ElapsedTime stopwatch = new ElapsedTime();

    private static DcMotorEx FlyL, FlyR;
    private static CRServo IntakeBrushL, IntakeBrushR, TransferL, TransferR;
    private static Servo Gate, RampL, RampR;

    MecanumDrive drive;

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

        Pose2d startPose = new Pose2d(60, -10, Math.toRadians(180));
        Pose2d shootingPose = new Pose2d(56, -8, Math.toRadians(200));
        Pose2d farRowPose = new Pose2d(35.5, -30, Math.toRadians(270));
        Pose2d middleRowPose = new Pose2d(12, -30, Math.toRadians(270));

        drive = new MecanumDrive(hardwareMap, startPose);

        Vector2d shootingPos = new Vector2d(56, -8);
        Vector2d farRow = new Vector2d(35.5, -30);
        Vector2d middleRow = new Vector2d(12, -30);

        // Go to shooting position
        Action seq1 = drive.actionBuilder(startPose)
                .strafeToLinearHeading(new Vector2d(56, -7), Math.toRadians(209))
                .waitSeconds(1)
                .build();

        // Go to far row
        Action seq12 = drive.actionBuilder(shootingPose)
                .strafeToLinearHeading(farRow, Math.toRadians(270))
                .build();

        // Intake forward 1
        Action seqIntakeForward1_1 = drive.actionBuilder(farRowPose)
                .strafeToLinearHeading(new Vector2d(35.5, -35), Math.toRadians(270))
                .build();

        // Intake forward 2
        Action seqIntakeForward1_2 = drive.actionBuilder(new Pose2d(35.5, -35, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(35.5, -40), Math.toRadians(270))
                .build();

        // Go back to shooting position
        Action seq2 = drive.actionBuilder(new Pose2d(35.5, -40, Math.toRadians(270)))
                .strafeToLinearHeading(shootingPos, Math.toRadians(200))
                .build();

        // Go to middle row
        Action seq3 = drive.actionBuilder(shootingPose)
                .strafeToLinearHeading(middleRow, Math.toRadians(270))
                .build();

        // Intake forward middle
        Action seqIntakeForward2 = drive.actionBuilder(middleRowPose)
                .lineToY(-52)
                .build();

        // Go back to shooting position from middle
        Action seq4 = drive.actionBuilder(new Pose2d(12, -52, Math.toRadians(270)))
                .strafeToLinearHeading(shootingPos, Math.toRadians(200))
                .build();


        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("BlueFarAuto ready");
            telemetry.update();
        }

        if (isStopRequested()) return;

        ////////////////////////////////////////
        /// Runtime actions
        ////////////////////////////////////////

        // Go to shooting position
        Actions.runBlocking(seq1);

        // Shoot first set
        launch(1050);

        // Go to far row
        Actions.runBlocking(seq12);

        // Intake
        intake();

        // Forward 1
        Actions.runBlocking(seqIntakeForward1_1);

        // Forward 2
        Actions.runBlocking(seqIntakeForward1_2);

        // Go back to shooting position
        Actions.runBlocking(seq2);

        // Shoot second set
        launch(1050);

        // Go to middle row
        Actions.runBlocking(seq3);

        // Intake
        intake();

        // Forward into middle row
        Actions.runBlocking(seqIntakeForward2);

        // Go back to shooting
        Actions.runBlocking(seq4);

        // Shoot third set
        launch(1050);

        while (opModeIsActive()) {
            idle();
        }
    }

    public void launch(double vel) {
        FlyL.setVelocity(vel);
        FlyR.setVelocity(vel);

        // Wait for flywheel to ramp up
        stopwatch.reset();
        while (stopwatch.time() < 4) {
            idle();
        }

        // Launch ball 1
        Gate.setPosition(BotConstants.GATE_DOWN_POS);
        TransferL.setPower(0.15);
        TransferR.setPower(0.15);
        stopwatch.reset();
        while (stopwatch.time() < 3) {
            idle();
        }

        // Reset for ball 2
        Gate.setPosition(BotConstants.GATE_UP_POS);
        TransferL.setPower(0);
        TransferR.setPower(0);
        stopwatch.reset();
        while (stopwatch.time() < 1) {
            idle();
        }

        // Launch ball 2
        Gate.setPosition(BotConstants.GATE_DOWN_POS);
        TransferL.setPower(0.15);
        TransferR.setPower(0.15);
        stopwatch.reset();
        while (stopwatch.time() < 3) {
            idle();
        }

        // Reset for ball 3
        Gate.setPosition(BotConstants.GATE_UP_POS);
        TransferL.setPower(0);
        TransferR.setPower(0);
        stopwatch.reset();
        while (stopwatch.time() < 1) {
            idle();
        }

        // Launch ball 3
        Gate.setPosition(BotConstants.GATE_DOWN_POS);
        TransferL.setPower(0.15);
        TransferR.setPower(0.15);
        stopwatch.reset();
        while (stopwatch.time() < 3) {
            idle();
        }

        // Stop everything
        Gate.setPosition(BotConstants.GATE_UP_POS);
        TransferL.setPower(0);
        TransferR.setPower(0);
        FlyL.setVelocity(0);
        FlyR.setVelocity(0);
    }

    public void intake() {
        Gate.setPosition(BotConstants.GATE_UP_POS);

        IntakeBrushL.setPower(0.15);
        IntakeBrushR.setPower(0.15);
        TransferL.setPower(0.15);
        TransferR.setPower(0.15);

        // Wait for intake to grab
        stopwatch.reset();
        while (stopwatch.time() < 3) {
            idle();
        }

        // Stop and close gate
        TransferL.setPower(0);
        TransferR.setPower(0);
        IntakeBrushL.setPower(0);
        IntakeBrushR.setPower(0);
        Gate.setPosition(BotConstants.GATE_DOWN_POS);
    }

    public void startMechs() {
        Gate.setPosition(BotConstants.GATE_UP_POS);
    }
}