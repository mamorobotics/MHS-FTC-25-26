package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
public class MotorTestOpMode extends LinearOpMode
{

    private DcMotor testMotor; // Declare your motor

    @Override
    public void runOpMode() {
        // 1. Hardware Map your motor
        testMotor = hardwareMap.get(DcMotor.class, "MotorTest");
        // Replace "yourMotorNameInConfig" with the actual name you gave your motor in the robot configuration.

        // Optional: Set motor direction if needed
        // For example, if your motor spins the wrong way:
        // testMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Optional: Set motor mode if needed (e.g., if you want to use encoders later)
        // testMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // or testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized. Press Play to run motor.");
        telemetry.update();

        waitForStart(); // Wait for the driver station to press PLAY

        if (opModeIsActive()) { // Ensure the OpMode is still active
            telemetry.addData("Status", "Running Motor...");
            telemetry.update();

            // 2. Set motor power to test
            double motorPower = 0.75; // Set a power level (e.g., 50%)
            testMotor.setPower(motorPower);

            // You can add a delay to let it run for a bit
            sleep(3000); // Run for 3 seconds

            // 3. Stop the motor
            testMotor.setPower(0);
            telemetry.addData("Status", "Motor Stopped.");
            telemetry.update();
        }

        telemetry.addData("Status", "OpMode Finished.");
        telemetry.update();
    }
}

}
