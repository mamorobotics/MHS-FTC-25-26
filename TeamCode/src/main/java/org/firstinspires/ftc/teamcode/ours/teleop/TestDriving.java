package org.firstinspires.ftc.teamcode.ours.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "testDriving")
public class TestDriving extends OpMode {

    private DriveTrain driveTrain = new DriveTrain();

    private double motorPower = 0.2;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

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

        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);

        gate = hardwareMap.get(Servo.class, "gate");
    }


    @Override
    public void loop () {
        if (gamepad1.a) {
            driveTrain.FL.setPower(motorPower);
            driveTrain.BL.setPower(motorPower);
            driveTrain.FR.setPower(motorPower);
            driveTrain.BR.setPower(motorPower);
        }
        else {
            driveTrain.FL.setPower(0);
            driveTrain.BL.setPower(0);
            driveTrain.FR.setPower(0);
            driveTrain.BR.setPower(0);
        }

        if (gamepad1.b) {
            visionPortal.resumeStreaming();

            if (aprilTag.getDetections().size() > 0) {
                AprilTagDetection detection = aprilTag.getDetections()[0];
                double bearing = detection.ftcPose.bearing;

                double rotationSpeed = Math.clamp(-1 * bearing / 90.0, 0, 1);

                driveTrain.move(0, 0, rotationSpeed, 1);
            }
        } else
        {
            visionPortal.stopStreaming();
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
