package org.firstinspires.ftc.teamcode.ours.autonomous.decode;

import com.acmerobotics.roadrunner.Action;
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
import org.firstinspires.ftc.teamcode.ours.BotConstants;




import org.firstinspires.ftc.teamcode.ours.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.ours.teleop.DriveTrain;


@Autonomous(name = "Blue Close Decode")
public class blueClose extends LinearOpMode {

    private static DcMotorEx FlyL, FlyR;
    private static CRServo IntakeBrushL, IntakeBrushR, TransferL, TransferR;
    private static Servo Gate, RampL, RampR;
    static DriveTrain driveTrain = new DriveTrain();

    // Flywheel:

    private double flywheelSpeed = 0.0;
    private static final double flywheelStep = 300;
    private static final double smallFlywheelStep = 50;
    private static final double flywheelFixedSpeed = 0.5;

    private static double flywheelLeftSpeed = 0;
    private static double flywheelRightSpeed = 0;

    private boolean flywheelOn = false;

    // Intake / Transfer:

    private boolean intakeSpin = false;
    private boolean transferSpin = false;
    private static final double intakeStopSpeed = 0;
    private static final double intakeRunSpeed = 0.4;


    // Gate:
    private static final double gateDownPos = 0.01;
    private static final double gateUpPos = 0.04;
    private boolean gateUp = false; // currently not used
    private static final double gateStep = 0.005;
    private static final double gatePos = 0.01;

    private static double rampPos = 0.38;

    private static final double rampStep = 0.01;

    @Override
    public void runOpMode() throws InterruptedException {


        driveTrain.setDriveTrain(hardwareMap, "leftFront", "leftBack", "rightFront", "rightBack");
        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveTrain.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveTrain.reverse(DriveTrain.DriveTrainSide.LEFT);

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
        Gate.setPosition(gateDownPos);

        RampR = hardwareMap.get(Servo.class, "rightRamp");
        RampL = hardwareMap.get(Servo.class, "leftRamp");

        RampL.setDirection(Servo.Direction.REVERSE);

        telemetry.addLine("Autonomous Initialized");
        telemetry.update();

        Pose2d startPose = new Pose2d(-60, -34, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);



        Vector2d shootingPos = new Vector2d(-6, 6);
        Vector2d closeRow   = new Vector2d(-12, -30);
        Vector2d middleRow  = new Vector2d(12, -30);
        Vector2d farRow     = new Vector2d(37, -30);

        // this is deprecated
//        drive.setPoseEstimate(startPose);

        TrajectoryActionBuilder blueClose = drive.actionBuilder(startPose)
                .splineTo(new Vector2d(-12, -10), Math.toRadians(180))

                //SCAN APRIL TAG HERE
                .waitSeconds(1)

                .turn(Math.toRadians(90))
                .lineToY(-30)

                //INTAKE ACTION HERE
                .waitSeconds(1)

                .turn(Math.toRadians(-45))
                .strafeTo(shootingPos)

                //SHOOT BALLS HERE
                .waitSeconds(1)

                .turn(Math.toRadians(45))
                .splineTo(middleRow, Math.toRadians(270))

                //INTAKE ACTION HERE
                .waitSeconds(1)

                .turn(Math.toRadians(-45))
                .strafeTo(shootingPos)

                //SHOOT BALLS HERE
                .waitSeconds(1)

                .splineTo(farRow, Math.toRadians(270))

                //INTAKE ACTION HERE
                .waitSeconds(1)

                .turn(Math.toRadians(-45))
                .strafeTo(shootingPos)

                //SHOOT BALLS HERE
                .waitSeconds(1)

                .turn(Math.toRadians(135))
                .lineToX(12);


        while (!isStarted() && !isStopRequested()) {

            telemetry.addLine("BlueCloseAuto ready");
            telemetry.update();
        }

        if (isStopRequested()) return;

//        drive.followTrajectorySequence(blueClose);

        Action trajectoryChosen = blueClose.build();

        Actions.runBlocking(
            new SequentialAction(
                    trajectoryChosen
            )
        );

        while (opModeIsActive()) {
            idle();
        }
    }
}

