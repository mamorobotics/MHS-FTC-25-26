package org.firstinspires.ftc.teamcode.ours.customtuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class RampServoTesting extends OpMode {
    Servo rampR, rampL;

    double lowPosition = 0;

    double highPosition = 0.5;

    double curPosition = 0;

    @Override
    public void init() {
        rampR = hardwareMap.get(Servo.class, "rightRamp");
        rampL = hardwareMap.get(Servo.class, "leftRamp");

        rampL.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void loop () {
        telemetry.addLine("Press A for low position (0), press B for higher position (0.5)");
        telemetry.addLine("Press X for custom position, with DPad up for greater and DPad down for lower value");
        if(gamepad1.aWasPressed()) {
            rampR.setPosition(lowPosition);
            rampL.setPosition(lowPosition);
            telemetry.addData("Position", lowPosition);
        }

        if(gamepad1.bWasPressed()) {
            rampR.setPosition(highPosition);
            rampL.setPosition(highPosition);
            telemetry.addData("Position", highPosition);
        }

        if(gamepad1.xWasPressed()) {
            rampR.setPosition(curPosition);
            rampL.setPosition(curPosition);
            telemetry.addData("Position", curPosition);
        }

        if (gamepad1.dpadUpWasPressed()) {
            curPosition += 0.02;
            rampR.setPosition(curPosition);
            rampL.setPosition(curPosition);
            telemetry.addData("Position", curPosition);
        }

        if (gamepad1.dpadDownWasPressed()) {
            curPosition -= 0.02;
            rampR.setPosition(curPosition);
            rampL.setPosition(curPosition);
            telemetry.addData("Position", curPosition);
        }

        telemetry.addData("Position", curPosition);


    }

}
