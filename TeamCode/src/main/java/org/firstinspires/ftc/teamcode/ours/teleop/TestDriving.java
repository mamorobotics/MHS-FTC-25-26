package org.firstinspires.ftc.teamcode.ours.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "testDriving")
public class TestDriving extends OpMode {

    private DriveTrain driveTrain = new DriveTrain();

    private double motorPower = 0.2;

    @Override
    public void init() {
        driveTrain.setDriveTrain(hardwareMap, "frontLeft", "backLeft", "frontRight", "backRight");
        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveTrain.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveTrain.reverse(DriveTrain.DriveTrainSide.LEFT);
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
    }
}
