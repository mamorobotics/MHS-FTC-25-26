package org.firstinspires.ftc.teamcode.ours.drive;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.hardware.bosch.BNO055Util;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ours.BotConstants;

public class Subsystems {
    public IntakeTransfer intake;
    public Launcher launcher;
    public GateRamp gateRamp;

    private double pickIntakeSpeed = 0.4; // speed for the intake when it is picking up artifacts

    private double pickTransferSpeed = 0.3; // speed for the transfer when it is picking up artifacts

    private double transportIntakeSpeed = 0.035; // speed for the intake to spin while moving, to hold a 3rd artifact in

    private double launchTransferSpeed = 0.15; // speed for the transfer when launching

    private double feedIntakeSpeed = 0.2;
    private double feedTransferSpeed = 0.15;
    private Telemetry telemetry1;



    public Subsystems(HardwareMap hardwareMap, Telemetry telemetry) {
        intake = new IntakeTransfer(hardwareMap);
        launcher = new Launcher(hardwareMap, telemetry);
        gateRamp = new GateRamp(hardwareMap, telemetry);
        telemetry1 = telemetry;
    }

    public Action initialize() {
        // reset everything imo at autonmous start ig
        return new ParallelAction(
                intake.intakeStop(),
                launcher.spinDown(),
                gateRamp.gateUp(),
                gateRamp.rampMove(BotConstants.RAMP_NEUTRAL_POS)
        );
    }



    public Action Intake() {
        return intake.intakeRun(pickIntakeSpeed, pickTransferSpeed);
    }

    public Action IntakeStop() {
        return intake.intakeStop();
    }

    public Action Transport() {
        return intake.transportHoldingRun(transportIntakeSpeed);
    }

    public Action LaunchPrepare(Pose2d pose) {
        double flywheelSpeed = PoseFlywheelSpeed(pose);
        double rampPos = PoseRampPosition(pose);

        return new ParallelAction(
                launcher.spinUp(flywheelSpeed),
                gateRamp.rampMove(rampPos)

        );
    }

    public Action launchOne(Pose2d pose) {
        double flywheelSpeed = PoseFlywheelSpeed(pose);
        double rampPos = PoseRampPosition(pose);


        return new SequentialAction(
                LaunchPrepare(pose),
                gateRamp.gateUp(),
                new SleepAction(0.5),
                gateRamp.gateDown(),
                new SleepAction(0.5),
                intake.launchingRun(launchTransferSpeed),
                launcher.launch(flywheelSpeed),
                gateRamp.gateUp(),
                intake.feedRun(feedIntakeSpeed, feedTransferSpeed)
        );
    }

    // NOTE - this is currently setup to launch 3 balls.
    public Action launchAll(Pose2d pose) {
        // imma be real i don't know why i made this a separate function from LaunchOne but there may be a reason which future me (or you) will discover
        Action all = new SequentialAction(
                launchOne(pose),
                launchOne(pose),
                launchOne(pose)
        );

        return all;
    }



    public double PoseFlywheelSpeed(Pose2d pose) {
        // fill in with functionality
        double distance = PoseGoalDistance(pose);
        double value = 1000;

        if (PoseIsFar(pose)) {
            value = DistanceFarSpeed(distance);
        }
        else {
            value = DistanceCloseSpeed(distance);
        }

        return Range.clip(value, 0, 3000);
    }

    public double PoseRampPosition(Pose2d pose) {
        // fill in with functionality
        double distance = PoseGoalDistance(pose);
        double value = 0.38;

        if (PoseIsFar(pose)) {
            value = DistanceFarRamp(distance);
        }
        else {
            value = DistanceCloseRamp(distance);
        }

        return Range.clip(value, 0.35, 0.46);
    }


    public double PoseGoalDistance(Pose2d pose) {
        double x_comp = pose.position.x - BotConstants.GOAL_X;
        double y_comp = pose.position.y - BotConstants.GOAL_Y;

        return Math.sqrt(Math.pow(x_comp, 2) + Math.pow(y_comp, 2));
    }

    public boolean PoseIsFar(Pose2d pose) {
        return pose.position.x > 20;
    }

    public double DistanceCloseSpeed(double distance) {
        double m = -3.535533906;
        double b = 1235;

        return m * distance + b;
    }

    public double DistanceCloseRamp(double distance) {
        double m = -0.0007071067812;
        double b = 0.437;

        return m * distance + b;
    }

    public double DistanceFarSpeed(double distance) {
        double m = 0;
        double b = 1100;

        return m * distance + b;
    }

    public double DistanceFarRamp(double distance) {
        double m = 0;
        double b = 0.38;

        return m * distance + b;
    }




}
