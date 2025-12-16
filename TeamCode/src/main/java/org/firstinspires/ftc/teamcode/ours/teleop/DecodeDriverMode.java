package org.firstinspires.ftc.teamcode.ours.teleop;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.Telemetry;

@ TeleOp(name = "DecodeTeleOp")
public class DecodeDriverMode extends OpMode
{
    private static DcMotor FlyL, FlyR;
    private static Servo IntakeBrushL, IntakeBrushR, IntakeBandL, IntakeBandR, TransferL, TransferR, Ramp;

    static DriveTrain driveTrain = new DriveTrain();

    private double flywheelPower = 0.0;          // current flywheel power [0..1]
    private static final double flywheelStep = 0.02;  // how much triggers change per loop
    private static final double flywheelFixedSpeed = 0.75;


    private double rampPos = 0.50;               // start centered-ish
    private static final double rampStep = 0.05;


    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastA = false;



    @Override
    public void init()
    {
        driveTrain.setDriveTrain(hardwareMap, "leftFront", "leftRear", "rightFront", "rightRear");
        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveTrain.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveTrain.reverse(DriveTrain.DriveTrainSide.LEFT);

        FlyL = hardwareMap.get(DcMotor.class, "leftFlywheel");
        FlyR = hardwareMap.get(DcMotor.class, "rightFlywheel");

        FlyL.setDirection(DcMotor.Direction.FORWARD);
        FlyR.setDirection(DcMotor.Direction.REVERSE);

        FlyL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FlyR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        IntakeBrushL = hardwareMap.get(Servo.class, "leftBrushIntake");
        IntakeBrushR = hardwareMap.get(Servo.class, "rightBrushIntake");
        TransferL = hardwareMap.get(Servo.class, "leftTransfer");
        TransferR = hardwareMap.get(Servo.class, "rightTransfer");
        IntakeBandL = hardwareMap.get(Servo.class, "leftIntakeBand");
        IntakeBandR = hardwareMap.get(Servo.class, "rightIntakeBand");
        Ramp = hardwareMap.get(Servo.class, "ramp");

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

        boolean a = gamepad1.a;
        if (a && !lastA) {
            flywheelPower = flywheelFixedSpeed;
        }
        lastA = a;

        double up = gamepad1.right_trigger;
        double down = gamepad1.left_trigger;

        if (up > 0.05) flywheelPower += flywheelStep * up;
        if (down > 0.05) flywheelPower -= flywheelStep * down;

        flywheelPower = Range.clip(flywheelPower, 0.0, 1.0);

        FlyL.setPower(flywheelPower);
        FlyR.setPower(flywheelPower);


        boolean dUp = gamepad1.dpad_up;
        boolean dDown = gamepad1.dpad_down;

        if (dUp && !lastDpadUp) {
            rampPos += rampStep;
        }
        if (dDown && !lastDpadDown) {
            rampPos -= rampStep;
        }

        lastDpadUp = dUp;
        lastDpadDown = dDown;

        rampPos = Range.clip(rampPos, 0.0, 1.0);
        Ramp.setPosition(rampPos);

    }

}
