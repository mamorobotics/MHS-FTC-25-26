package org.firstinspires.ftc.teamcode.ours.autonomous.decode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ours.BotConstants;
import org.firstinspires.ftc.teamcode.ours.drive.Subsystems;

@Autonomous(name = "TestAuto")
public class TestAuton extends LinearOpMode {
    private Subsystems subsystems;
    private static ElapsedTime stopwatch = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        subsystems = new Subsystems(hardwareMap, telemetry);

        telemetry.addLine("Autonomous Initialized");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {

            telemetry.addLine("Auto ready");
            telemetry.update();
        }

        if (isStopRequested()) return;

        // move forward and turn to the right angle

        launch(1050);


    }

    public void launch(double vel) {
        subsystems.launcher.FlyLeft.setVelocity(vel);
        subsystems.launcher.FlyRight.setVelocity(vel);


        stopwatch.reset();

        while(stopwatch.time() < 4) {
            subsystems.intake.TransferL.setPower(0);
            subsystems.intake.TransferR.setPower(0);
        }

        subsystems.gateRamp.Gate.setPosition(BotConstants.GATE_DOWN_POS);

        subsystems.intake.TransferL.setPower(0.15);
        subsystems.intake.TransferR.setPower(0.15);

        stopwatch.reset();

        while(stopwatch.time() < 3) {
            subsystems.intake.TransferL.setPower(0.15);
            subsystems.intake.TransferR.setPower(0.15);
        }

        subsystems.gateRamp.Gate.setPosition(BotConstants.GATE_UP_POS);
        subsystems.intake.TransferL.setPower(0);
        subsystems.intake.TransferR.setPower(0);

        while(stopwatch.time() < 4) {
            subsystems.intake.TransferL.setPower(0);
            subsystems.intake.TransferR.setPower(0);
        }


        subsystems.gateRamp.Gate.setPosition(BotConstants.GATE_DOWN_POS);

        subsystems.intake.TransferL.setPower(0.15);
        subsystems.intake.TransferR.setPower(0.15);

        stopwatch.reset();

        while(stopwatch.time() < 3) {
            subsystems.intake.TransferL.setPower(0.15);
            subsystems.intake.TransferR.setPower(0.15);
        }


    }

}
