package org.firstinspires.ftc.teamcode.ours.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@ TeleOp(name = "DecodeTeleOp")
public class DecodeDriverMode extends OpMode
{
    private static DcMotorEx FlyL, FlyR;
    private static CRServo IntakeBrushL, IntakeBrushR, TransferL, TransferR;
    private static Servo Gate;
    static DriveTrain driveTrain = new DriveTrain();

    // Flywheel:

    private double flywheelSpeed = 0.0;          // current flywheel power [0..1]
    private static final double flywheelStep = 300;  // how much triggers change per loop
    private static final double smallFlywheelStep = 50; // how much change per loop
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
    private static final double gateStep = 0.005; // only for testing / calibration
    private static final double gatePos = 0.01; // only for testing / calibration


    // Ramp:

    private double rampStep = 0.05; // PLACEHOLDER - calibrate
    private double rampPos = 0.5; // PLACEHOLDER - calibrate

//
//    /// Misha Edit: adding boolean var -- intakeSpin: toggle on/off with x key
//    /// Also, lastXpadDown -- if pressed: down, else: up
//
//    private boolean lastXpadPressed = false;
//
//    private boolean bPressed = false;


    @Override
    public void init()
    {
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

        telemetry.addLine("TeleOp Initialized");
        telemetry.update();


    }
    
    @Override
    public void loop()
    {

        // Drivetrain
        double y = -gamepad1.left_stick_y;
        double x =  gamepad1.left_stick_x;  // strafe
        double r =  gamepad1.right_stick_x; // rotate

        // Press right button for slow mode
        double speed = gamepad1.right_bumper ? 0.4 : 1.0;

        driveTrain.move(x, y, r, speed);


        // Launcher
        manageLauncher();

        // Intake / Transfer
        manageIntake();

        // Gate
        manageGate();

        // Ramp
        manageRamp();

        }

        void manageLauncher() {
            if (gamepad2.aWasPressed()) {
                flywheelOn = !flywheelOn;
            }

            if (gamepad2.dpadUpWasPressed()) flywheelSpeed += flywheelStep;
            if (gamepad2.dpadDownWasPressed()) flywheelSpeed -= flywheelStep;
            if (gamepad2.dpadRightWasPressed()) flywheelSpeed += smallFlywheelStep;
            if (gamepad2.dpadLeftWasPressed()) flywheelSpeed -= smallFlywheelStep;

            flywheelSpeed = Range.clip(flywheelSpeed, 0.0, 3000);
            telemetry.addData("Flywheel Set Speed:", flywheelSpeed);
            flywheelLeftSpeed = FlyL.getVelocity(); // divide by 60 bc: divide degrees / 360 to get rotations, but * 60 to get per minute
            flywheelRightSpeed = FlyR.getVelocity(); // divide by 60 bc: divide degrees / 360 to get rotations, but * 60 to get per minute
            telemetry.addData("Left Flywheel Speed: ", flywheelLeftSpeed);
            telemetry.addData("Right Flywheel Speed: ", flywheelRightSpeed);



            if (flywheelOn) {
                FlyL.setVelocity(flywheelSpeed);
                FlyR.setVelocity(flywheelSpeed);
//                FlyL.setVelocity(flywheelSpeed * 60, AngleUnit.DEGREES); // multiply rpms by 360 to get degrees per min, divide by 60 to get per second
//                FlyR.setVelocity(flywheelSpeed * 60, AngleUnit.DEGREES); // multiply rpms by 360 to get degrees per min, divide by 60 to get per second
            }
            else {
                FlyL.setVelocity(0);
                FlyR.setVelocity(0);
            }
        }


        void manageIntake() {
            if (gamepad2.xWasPressed()) {
                intakeSpin = !intakeSpin;
            }

            if (gamepad2.yWasPressed()) {
                transferSpin = !transferSpin;
            }


            if (intakeSpin) {
                IntakeBrushL.setPower(intakeRunSpeed);
                IntakeBrushR.setPower(intakeRunSpeed);
            }
            else {
                IntakeBrushL.setPower(intakeStopSpeed);
                IntakeBrushR.setPower(intakeStopSpeed);
            }

            if (transferSpin) {
                TransferL.setPower(intakeRunSpeed);
                TransferR.setPower(intakeRunSpeed);
            }
            else {
                TransferL.setPower(intakeStopSpeed);
                TransferR.setPower(intakeStopSpeed);
            }
        }

        void manageGate() {
            if(gamepad2.b) {
                Gate.setPosition(gateDownPos);
            }
            else {
                Gate.setPosition(gateUpPos);
            }
        }

        void manageRamp() {
            if(gamepad2.dpadDownWasPressed()) {
                rampPos -= rampStep;
            }
            else if (gamepad2.dpadUpWasPressed()) {
                rampPos += rampStep;
            }
            rampPos = Range.clip(rampPos, 0, 1);
            // set ramp pos here - currently missing
        }

        void launch() {
            // automate the launching procedure
        }
    }


