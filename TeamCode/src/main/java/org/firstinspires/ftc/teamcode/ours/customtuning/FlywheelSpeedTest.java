package org.firstinspires.ftc.teamcode.ours.customtuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range; // Import Range to keep power within limits

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

    private double currentPower = 0.0;

    private boolean aWasPressedLast = false;
    private boolean yWasPressedLast = false;

    private static final double POWER_STEP = 0.05;

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

        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);



        telemetry.addLine("FlywheelSpeedTest ready.");
        telemetry.addLine("Right stick: up = faster, down = slower.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double stick = -gamepad1.right_stick_y;

            if (Math.abs(stick) > 0.1) {
                currentPower = stick;
            }

            // 2. Y button increments the power (on a single press)
            if (gamepad1.y && !yWasPressedLast) {
                currentPower += POWER_STEP;
            }
            yWasPressedLast = gamepad1.y; // Update for next loop

            // 3. A button decrements the power (on a single press)
            if (gamepad1.a && !aWasPressedLast) {
                currentPower -= POWER_STEP;
            }
            aWasPressedLast = gamepad1.a; // Update for next loop


            // --- MOTOR CONTROL ---

            // 4. Clip the final power value to ensure it's valid (-1.0 to 1.0)
            currentPower = Range.clip(currentPower, -1.0, 1.0);
            // 5. Apply the final, calculated power to the motors
            m1.setPower(currentPower);
            m2.setPower(currentPower);

            double tps1 = Math.abs(m1.getVelocity());
            double tps2 = Math.abs(m2.getVelocity());

            double rpm1 = (tps1 / TICKS_PER_REV) * SECONDS_PER_MINUTE;
            double rpm2 = (tps2 / TICKS_PER_REV) * SECONDS_PER_MINUTE;

            double ips1 = (rpm1 / SECONDS_PER_MINUTE) * FLYWHEEL_CIRCUMFERENCE_IN;
            double ips2 = (rpm2 / SECONDS_PER_MINUTE) * FLYWHEEL_CIRCUMFERENCE_IN;

            telemetry.addData("Stick (up=+)", "%.2f", stick);
            telemetry.addData("Power", "%.2f", currentPower);

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
