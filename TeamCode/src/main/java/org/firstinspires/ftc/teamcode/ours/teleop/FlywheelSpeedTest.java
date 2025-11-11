package org.firstinspires.ftc.teamcode.ours.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "FlywheelSpeedTest")
public class FlywheelSpeedTest extends LinearOpMode {

    private static final String M1name = "flywheel1";
    private static final String M2name = "flywheel2";


    private static final double TICKS_PER_REV = 537.6;

    private static final double FLYWHEEL_DIAMETER_IN = 3.75;
    private static final double FLYWHEEL_CIRCUMFERENCE_IN = Math.PI * FLYWHEEL_DIAMETER_IN;


    private static final double SECONDS_PER_MINUTE = 60.0;
    private static final double INCHES_PER_FOOT = 12.0;
    private static final double INCHES_PER_METER = 39.37007874;

    private static final double POWER_STEP = 0.02;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx m1 = hardwareMap.get(DcMotorEx.class, M1name);
        DcMotorEx m2 = hardwareMap.get(DcMotorEx.class, M2name);

        m1.setDirection(DcMotorSimple.Direction.FORWARD);
        m2.setDirection(DcMotorSimple.Direction.REVERSE);
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("FlywheelSpeedTest ready.");
        telemetry.addLine("Right stick: up = faster, down = slower.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double stick = -gamepad1.right_stick_y;

            double power = stick;
            if (stick > 0) {
                power += 0.02 * stick;  // increase power gradually
            } else if (stick < 0) {
                power += 0.02 * stick;  // decrease power gradually
            }

            m1.setPower(power);
            m2.setPower(power);

            double tps1 = Math.abs(m1.getVelocity());
            double tps2 = Math.abs(m2.getVelocity());

            double rpm1 = (tps1 / TICKS_PER_REV) * SECONDS_PER_MINUTE;
            double rpm2 = (tps2 / TICKS_PER_REV) * SECONDS_PER_MINUTE;

            double ips1 = (rpm1 / SECONDS_PER_MINUTE) * FLYWHEEL_CIRCUMFERENCE_IN;
            double ips2 = (rpm2 / SECONDS_PER_MINUTE) * FLYWHEEL_CIRCUMFERENCE_IN;

            telemetry.addData("Stick (up=+)", "%.2f", stick);
            telemetry.addData("Power", "%.2f", power);

            telemetry.addLine("--- Motor 1 ---");
            telemetry.addData("Ticks/sec", "%.1f", tps1);
            telemetry.addData("RPM", "%.1f", rpm1);
            telemetry.addData("Rim Speed", "%.2f in/s | %.2f ft/s | %.2f m/s",
                    ips1, ips1 / INCHES_PER_FOOT, ips1 / INCHES_PER_METER);

            telemetry.addLine("--- Motor 2 ---");
            telemetry.addData("Ticks/sec", "%.1f", tps2);
            telemetry.addData("RPM", "%.1f", rpm2);
            telemetry.addData("Rim Speed", "%.2f in/s | %.2f ft/s | %.2f m/s",
                    ips2, ips2 / INCHES_PER_FOOT, ips2 / INCHES_PER_METER);

            telemetry.update();
        }

        m1.setPower(0.0);
        m2.setPower(0.0);
    }
}
