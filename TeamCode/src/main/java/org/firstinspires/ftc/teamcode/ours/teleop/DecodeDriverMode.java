package org.firstinspires.ftc.teamcode.ours.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.Telemetry;

@ TeleOp(name = "DecodeTeleOp")
public class DecodeDriverMode extends OpMode
{
    private static DcMotor FlyL, FlyR;
    private static CRServo IntakeBrushL, IntakeBrushR, TransferL, TransferR;
    private static Servo Gate;
    static DriveTrain driveTrain = new DriveTrain();

    // Flywheel:

    private double flywheelPower = 0.0;          // current flywheel power [0..1]
    private static final double flywheelStep = 0.02;  // how much triggers change per loop
    private static final double flywheelFixedSpeed = 0.5;

    private boolean flywheelOn = false;

    // Intake / Transfer:

    private boolean intakeSpin = false;
    private boolean transferSpin = false;
    private static final double intakeStopSpeed = 0;
    private static final double intakeRunSpeed = 0.4;


    // Gate:
    private static final double gateDownPos = 0.01;
    private static final double gateUpPos = 0.03;
    private boolean gateUp = false; // currently not used
    private static final double gateStep = 0.005; // only for testing / calibration
    private static final double gatePos = 0.01; // only for testing / calibration


    // Ramp:

    private double rampPos = 0.5; // PLACEHOLDER - calibrate
    private double rampStep = 0.05; // PLACEHOLDER - calibrate


    private boolean lastA2 = false;
    private boolean lastX2 = false;
    private boolean lastY2 = false;
    private boolean lastB2 = false;

//    private boolean lastDpadUp = false;
//    private boolean lastDpadDown = false;
//    private boolean lastA = false;
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
        driveTrain.setDriveTrain(hardwareMap, "frontLeft", "backLeft", "frontRight", "backRight");
        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveTrain.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveTrain.reverse(DriveTrain.DriveTrainSide.LEFT);

        FlyL = hardwareMap.get(DcMotor.class, "leftLauncher");
        FlyR = hardwareMap.get(DcMotor.class, "rightLauncher");
        FlyL.setDirection(DcMotor.Direction.FORWARD);
        FlyR.setDirection(DcMotor.Direction.REVERSE);
        FlyL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FlyR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
            if (gamepad2.a && !lastA2) {
                flywheelOn = !flywheelOn;
            }
            lastA2 = gamepad2.a;


            double up = gamepad2.right_trigger;
            double down = gamepad2.left_trigger;

            if (up > 0.05) flywheelPower += flywheelStep * up;
            if (down > 0.05) flywheelPower -= flywheelStep * down;

            flywheelPower = Range.clip(flywheelPower, 0.0, 1.0);
            telemetry.addData("Flywheel Power:", flywheelPower);

            if (flywheelOn) {
                FlyL.setPower(flywheelPower);
                FlyR.setPower(flywheelPower);
            }
            else {
                FlyL.setPower(0);
                FlyR.setPower(0);
            }
        }


        void manageIntake() {
            if (gamepad2.x && !lastX2) {
                intakeSpin = !intakeSpin;
            }
            lastX2 = gamepad2.x;

            if (gamepad2.y && !lastY2) {
                transferSpin = !transferSpin;
            }

            lastY2 = gamepad2.y;

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


