package org.firstinspires.ftc.teamcode.ours.autonomous.decode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;      // RR 0.5.x geometry
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;    // RR 0.5.x geometry
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.ours.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.ours.drive.Subsystems;
import org.firstinspires.ftc.teamcode.ours.teleop.DriveTrain;


@Autonomous(name = "Blue Far Decode")
public class blueFar extends LinearOpMode {
    static DriveTrain driveTrain = new DriveTrain();

    // Flywheel:

    @Override
    public void runOpMode() throws InterruptedException {
        Subsystems subsystems = new Subsystems(hardwareMap);

        subsystems.initialize();


        telemetry.addLine("Autonomous Initialized");
        telemetry.update();

        Pose2d startPose = new Pose2d(60, -10, Math.toRadians(180));
        Pose2d shootingPose = new Pose2d(56, -8, Math.toRadians(200));
        Pose2d closeRowPose = new Pose2d(-12, -30, Math.toRadians(270));
        Pose2d middleRowPose = new Pose2d(12, -30, Math.toRadians(270));
        Pose2d farRowPose = new Pose2d(37, -30, Math.toRadians(270));

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        Vector2d shootingPos = new Vector2d(56,-8);
        Vector2d closeRow   = new Vector2d(-12, -30);
        Vector2d middleRow  = new Vector2d(12, -30);
        Vector2d farRow     = new Vector2d(37, -30);



        Action seq1 = drive.actionBuilder(startPose)
                .strafeToLinearHeading(shootingPos, Math.toRadians(200))
                //SCAN APRIL TAG HERE
                .waitSeconds(1)
                .build();

        //SHOOT
        Action seq12 = drive.actionBuilder(shootingPose)
                .strafeToLinearHeading(farRow, Math.toRadians(270))
                .build();

        //INTAKE

        Action seqIntakeForward1 = drive.actionBuilder(farRowPose)
                .lineToY(-52)
                .build(); // FILL THIS PART IN

        Action seq2 = drive.actionBuilder(new Pose2d(37,-52,Math.toRadians(270)))
                .strafeToLinearHeading(shootingPos, Math.toRadians(200))
                .build();

        //SHOOT

        Action seq3 = drive.actionBuilder(shootingPose)
                .strafeToLinearHeading(middleRow, Math.toRadians(270))
                .build();

        //INTAKE

        Action seqIntakeForward2 = drive.actionBuilder(middleRowPose)
                .lineToY(-52)
                .build(); // FILL THIS PART IN; // FILL THIS PART IN

        Action seq4 = drive.actionBuilder(new Pose2d(12,-52,Math.toRadians(270)))
                .strafeToLinearHeading(shootingPos, Math.toRadians(200))
                .build();

        //SHOOT

        //Action seq5 = drive.actionBuilder(shootingPose)
        //      .splineTo(farRow, Math.toRadians(270))
        //      .build();

        //INTAKE

        //TrajectoryActionBuilder blueClose = drive.actionBuilder(startPose)

        Actions.runBlocking(new SequentialAction(
                seq1,
                subsystems.LaunchAll(shootingPose),
                seq12,
                new ParallelAction(
                        subsystems.Intake(),
                        seqIntakeForward1
                ),
                seq2,
                subsystems.LaunchAll(shootingPose),
                seq3,
                new ParallelAction(
                        subsystems.Intake(),
                        seqIntakeForward2
                ),
                seq4,
                subsystems.LaunchAll(shootingPose)
        ));

        while (!isStarted() && !isStopRequested()) {

            telemetry.addLine("BlueCloseAuto ready");
            telemetry.update();
        }

        if (isStopRequested()) return;

//        drive.followTrajectorySequence(blueClose);

/*        Action trajectoryChosen = blueClose.build();

        Actions.runBlocking(
            new SequentialAction(
                    trajectoryChosen
            )
        );
*/
        while (opModeIsActive()) {
            idle();
        }
    }
}

