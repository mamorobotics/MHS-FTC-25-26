package org.firstinspires.ftc.teamcode.ours.customtuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class FlywheelPidTune extends OpMode {
    public DcMotorEx flyL, flyR;

    public double highVelocity = 1200;

    public double lowVelocity = 800;

    double targetVelocity = highVelocity;

    public boolean left = false;

    public boolean right = false;

    double[] stepSizes = {10, 1, 0.1, 0.01};

    int stepIndex = 0;

    double[] coefsBase = {0,0,0,0};
    String[] coefsString = {"P", "I", "D", "F"};


    int varInd = 0;

    @Override
    public void init() {
        flyL = hardwareMap.get(DcMotorEx.class, "leftLauncher");
        flyR = hardwareMap.get(DcMotorEx.class, "rightLauncher");

        flyL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flyL.setDirection(DcMotor.Direction.REVERSE);
        flyR.setDirection(DcMotor.Direction.FORWARD);

        PIDFCoefficients Lcoef = new PIDFCoefficients(coefsBase[0], coefsBase[1], coefsBase[2], coefsBase[3]);
        PIDFCoefficients Rcoef = new PIDFCoefficients(coefsBase[0], coefsBase[1], coefsBase[2], coefsBase[3]);

        flyL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Lcoef);
        flyR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Rcoef);



    }

    @Override
    public void loop () {

        if(gamepad1.aWasPressed()) {
            targetVelocity = highVelocity;
        }
        if(gamepad1.bWasPressed()) {
            targetVelocity = lowVelocity;
        }

        if(gamepad1.dpadRightWasPressed()) {
            stepIndex = Range.clip(stepIndex + 1, 0, stepSizes.length-1);

        }

        if(gamepad1.dpadLeftWasPressed()) {
            stepIndex = Range.clip(stepIndex - 1, 0, stepSizes.length-1);

        }

        if (gamepad1.rightBumperWasPressed()) {
            left = false;
            right = true;
        }

        if (gamepad1.leftBumperWasPressed()) {
            left = true;
            right = false;
        }

        if(gamepad1.yWasPressed()) {
            left = false;
            right = false;
        }

        if (gamepad1.xWasPressed()) {
            varInd = (varInd + 1) % 4;
        }

        if (gamepad1.dpadUpWasPressed()) {
            coefsBase[varInd] += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            coefsBase[varInd] -= stepSizes[stepIndex];
        }


        double curLeftVel = flyL.getVelocity();
        double curRightVel = flyR.getVelocity();
        double error = 0;
        double cVel = 0;

        PIDFCoefficients newCoeff = new PIDFCoefficients(coefsBase[0], coefsBase[1], coefsBase[2], coefsBase[3]);

        if(left) {
            flyL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newCoeff);
            flyL.setVelocity(targetVelocity);
            error = targetVelocity - curLeftVel;
            cVel = curLeftVel;
        }
        if(right) {
            flyR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newCoeff);
            flyR.setVelocity(targetVelocity);
            error = targetVelocity - curRightVel;
            cVel = curRightVel;
        }
        if(!right & !left) {
            flyL.setVelocity(0);
            flyR.setVelocity(0);
        }

        telemetry.addData("Right", right);
        telemetry.addData("Left", left);
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Current Velocity", cVel);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addLine("--------------------");
        telemetry.addData("Tuning Variable", coefsString[varInd]);
        telemetry.addData("Tuning Value", coefsBase[varInd]);
        telemetry.addData("Step Size", stepSizes[stepIndex]);



    }

}
