package org.firstinspires.ftc.teamcode.ours.autonomous.decode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ours.drive.Subsystems;
import org.firstinspires.ftc.teamcode.ours.teleop.DriveTrain;

@Autonomous(name = "TestActions")
public class testActions extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Subsystems subsystems = new Subsystems(hardwareMap, telemetry);

        Actions.runBlocking(subsystems.initialize());


        telemetry.addLine("Autonomous Initialized");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {

            telemetry.addLine("Auto ready");
            telemetry.update();
        }

        if (isStopRequested()) return;

        Pose2d testPose = new Pose2d(-6, 6, Math.toRadians(180));


        telemetry.addLine("Started!");
        telemetry.update();
        Action first = new SequentialAction(
                subsystems.LaunchPrepare(testPose),
                new ParallelAction(
                        subsystems.gateRamp.gateDown(),
                        subsystems.launcher.launch(1000),
                        subsystems.intake.launchingRun(0.1)
                )

        );

        Action sec = new ParallelAction(
                subsystems.intake.launchingRun(0.3),
                new SleepAction(3)
        );

        Actions.runBlocking(first);

        while (opModeIsActive()) {
            idle();
        }
    }
}
