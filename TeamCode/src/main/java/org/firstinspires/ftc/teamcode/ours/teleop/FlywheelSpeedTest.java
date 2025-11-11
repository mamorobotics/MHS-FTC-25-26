package org.firstinspires.ftc.teamcode.ours.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "FlywheelSpeedTest")
public class FlywheelSpeedTest extends LinearOpMode {

    private static final String FLYWHEEL_NAME = "flywheel";

    private static final double TICKS_PER_REV = 537.6;

    private static final double FLYWHEEL_DIAMETER_IN = 3.75;
    private static final double FLYWHEEL_CIRCUMFERENCE_IN = Math.PI * FLYWHEEL_DIAMETER_IN;


    private static final double SECONDS_PER_MINUTE = 60.0;
    private static final double INCHES_PER_FOOT = 12.0;
    private static final double INCHES_PER_METER = 39.37007874;

    private static final double POWER_STEP = 0.02;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, FLYWHEEL_NAME);

        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("FlywheelSpeedTest ready.");
        telemetry.addLine("Right stick: up = faster, down = slower.");
        telemetry.update();

        waitForStart();

        double targetPower = 0.0;

        while (opModeIsActive()) {
            double stick = -gamepad1.right_stick_y;

            targetPower += stick * POWER_STEP;
            targetPower = clamp01(targetPower);

            flywheel.setPower(targetPower);

            double ticksPerSec = Math.abs(flywheel.getVelocity());

            double revsPerSec = ticksPerSec / TICKS_PER_REV;                  // output-shaft rev/s
            double rpm        = revsPerSec * SECONDS_PER_MINUTE;

            double inchesPerSec = revsPerSec * FLYWHEEL_CIRCUMFERENCE_IN;
            double feetPerSec   = inchesPerSec / INCHES_PER_FOOT;
            double metersPerSec = inchesPerSec / INCHES_PER_METER;

            telemetry.addData("Stick", "%.2f", stick);
            telemetry.addData("Power", "%.2f", targetPower);
            telemetry.addData("Ticks/sec", "%.1f", ticksPerSec);
            telemetry.addData("RPM", "%.1f", rpm);
            telemetry.addData("Rim Speed", "%.2f in/s  |  %.2f ft/s  |  %.2f m/s",
                    inchesPerSec, feetPerSec, metersPerSec);
            telemetry.update();
        }

        flywheel.setPower(0.0);
    }

    private static double clamp01(double v) {
        if (v < 0.0) return 0.0;
        if (v > 1.0) return 1.0;
        return v;
    }
}
