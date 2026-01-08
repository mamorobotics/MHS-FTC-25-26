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
    private static Servo Ramp;
    static DriveTrain driveTrain = new DriveTrain();

    private double flywheelPower = 0.0;          // current flywheel power [0..1]
    private static final double flywheelStep = 0.02;  // how much triggers change per loop
    private static final double flywheelFixedSpeed = 0.75;

    /// Misha edit: adding intake fixed speed. Value should be between 0.5 - 1.0 (0.5 = stop, 1 = max speed forward)
    private static final double intakeRunSpeed = 0.9;
    //May need to adjust below stop speed.
    private static final double intakeStopSpeed = 0.5;
    private double rampPos = 0.50;               // start centered-ish
    private static final double rampStep = 0.05;


    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastA = false;

    /// Misha Edit: adding boolean var -- intakeSpin: toggle on/off with x key
    /// Also, lastXpadDown -- if pressed: down, else: up
    private boolean intakeSpin = false;
    private boolean lastXpadPressed = false;


    @Override
    public void init()
    {
        driveTrain.setDriveTrain(hardwareMap, "frontLeft", "backLeft", "frontRight", "backRight");
        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveTrain.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //driveTrain.reverse(DriveTrain.DriveTrainSide.LEFT);

        FlyL = hardwareMap.get(DcMotor.class, "leftLauncher");
        FlyR = hardwareMap.get(DcMotor.class, "rightLauncher");

        FlyL.setDirection(DcMotor.Direction.FORWARD);
        FlyR.setDirection(DcMotor.Direction.REVERSE);

        FlyL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FlyR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        IntakeBrushL = hardwareMap.get(CRServo.class, "leftIntake");
        IntakeBrushR = hardwareMap.get(CRServo.class, "rightIntake");
        TransferL = hardwareMap.get(CRServo.class, "leftTransfer");
        TransferR = hardwareMap.get(CRServo.class, "rightTransfer");
        //IntakeBandL = hardwareMap.get(Servo.class, "leftIntakeBand");
        //IntakeBandR = hardwareMap.get(Servo.class, "rightIntakeBand");
        Ramp = hardwareMap.get(Servo.class, "gate");

        //Misha edit: adding inverted directions to intake motors -- direction may be flipped.
        IntakeBrushL.setDirection(CRServo.Direction.FORWARD);
        IntakeBrushR.setDirection(CRServo.Direction.REVERSE);
        TransferL.setDirection(CRServo.Direction.FORWARD);
        TransferR.setDirection(CRServo.Direction.REVERSE);
        //IntakeBandL.setDirection(Servo.Direction.FORWARD);
        //IntakeBandR.setDirection(Servo.Direction.REVERSE);

        /// Misha Edit: setting zero power setting for intake servos to brake/stop moving
        /// Actually, there is no such thing, not using this.
        ///IntakeBrushL.setZeroPowerBehavior(Servo.ZeroPowerBehavior.BRAKE);


        Ramp.setPosition(rampPos);

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

        boolean a = gamepad2.a;

        boolean b = gamepad2.b;

        if (a && !lastA) {
            flywheelPower = flywheelFixedSpeed;
        }
        lastA = a;

        double up = gamepad2.right_trigger;
        double down = gamepad2.left_trigger;

        if (up > 0.05) flywheelPower += flywheelStep * up;
        if (down > 0.05) flywheelPower -= flywheelStep * down;

        flywheelPower = Range.clip(flywheelPower, 0.0, 1.0);

        FlyL.setPower(flywheelPower);
        FlyR.setPower(flywheelPower);


        boolean dUp = gamepad2.dpad_up;
        boolean dDown = gamepad2.dpad_down;

        if (dUp && !lastDpadUp) {
            rampPos += rampStep;
        }
        if (dDown && !lastDpadDown) {
            rampPos -= rampStep;
        }

        lastDpadUp = dUp;
        lastDpadDown = dDown;

        rampPos = Range.clip(rampPos, 0.0, 1.0);
        //Ramp.setPosition(rampPos);


        ///  Misha Edit: press x -- turn intake motors on, press x again -- turn intake motors off

        boolean xPressed;
        xPressed = gamepad1.x;

        //// This code should work after re-coding servos to continually rotate like motor.
        /// Note on how to use servos:
        /// servo.setDirection(Servo.Direction.FORWARD);
        /// servo.setPosition(1.0);   // spins forward
        /// servo.setPosition(0.5);   // stop --- may need to adjust value slightly
        /// servo.setPosition(0.0);   // spins backward

        ///Function to toggle spin between on/off

        if (xPressed && !lastXpadPressed) {
            intakeSpin = !intakeSpin;

        lastXpadPressed = xPressed;


        if (intakeSpin) {
            IntakeBrushL.setPower(intakeRunSpeed);
            IntakeBrushR.setPower(intakeRunSpeed);
            TransferL.setPower(intakeRunSpeed);
            TransferR.setPower(intakeRunSpeed);
            //IntakeBandL.setPower(intakeRunspeed);
            //IntakeBandR.setPower(intakeRunSpeed);
        } else {
            IntakeBrushL.setPower(intakeStopSpeed);
            IntakeBrushR.setPower(intakeStopSpeed);
            TransferL.setPower(intakeStopSpeed);
            TransferR.setPower(intakeStopSpeed);
            //IntakeBandL.setPosition(intakeStopSpeed);
            //IntakeBandR.setPosition(intakeStopSpeed);
        }

        if (b) {
            Ramp.setPosition(0);
        }
        else {
            Ramp.setPosition(1);
        }



        }
    }

}
