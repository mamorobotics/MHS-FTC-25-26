package org.firstinspires.ftc.teamcode.ours.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "testDriving")
public class TestDriving extends OpMode {

    private DriveTrain driveTrain = new DriveTrain();

    private double motorPower = 0.2;

    private Servo gate;

    private double gatepos;
    private double gateOpenPos = 0.01;
    private double gateClosePos = 0.02;

    private boolean gateDown = true;
    private double gateStep = 0.005;

    @Override
    public void init() {
        driveTrain.setDriveTrain(hardwareMap, "frontLeft", "backLeft", "frontRight", "backRight");
        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveTrain.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveTrain.reverse(DriveTrain.DriveTrainSide.LEFT);

        gate = hardwareMap.get(Servo.class, "gate");
    }


    @Override
    public void loop () {
        if (gamepad1.a) {
            DriveTrain.FL.setPower(motorPower);
            DriveTrain.BL.setPower(motorPower);
            DriveTrain.FR.setPower(motorPower);
            DriveTrain.BR.setPower(motorPower);
        }
        else {
            DriveTrain.FL.setPower(0);
            DriveTrain.BL.setPower(0);
            DriveTrain.FR.setPower(0);
            DriveTrain.BR.setPower(0);
        }

        if (gamepad2.aWasPressed()) {
            gatepos = gateOpenPos;
            gateDown = false;
        }
        if (gamepad2.bWasPressed()) {
            gatepos = gateClosePos;
            gateDown = true;
        }

        if (gamepad2.dpadUpWasPressed()) {
            gatepos += gateStep;
        }
        else if (gamepad2.dpadDownWasPressed()) {
            gatepos -= gateStep;
        }

        if (!gateDown) {
            gateOpenPos = gatepos;
        }
        if (gateDown) {
            gateClosePos = gatepos;
        }

        telemetry.addData("Gate Closed Pos:", gateClosePos);
        telemetry.addData("Gate Open Pos:", gateOpenPos);

        gate.setPosition(gatepos);
    }
}
