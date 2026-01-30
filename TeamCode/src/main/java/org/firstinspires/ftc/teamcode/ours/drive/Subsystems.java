package org.firstinspires.ftc.teamcode.ours.drive;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ours.BotConstants;

public class Subsystems {
    private IntakeTransfer intake;
    private Launcher launcher;
    private GateRamp gateRamp;

    private double pickIntakeSpeed = 0.4; // speed for the intake when it is picking up artifacts

    private double pickTransferSpeed = 0.3; // speed for the transfer when it is picking up artifacts

    private double transportIntakeSpeed = 0.035; // speed for the intake to spin while moving, to hold a 3rd artifact in

    private double launchTransferSpeed = 0.1; // speed for the transfer when launching

    private double feedIntakeSpeed = 0.2;
    private double feedTransferSpeed = 0.15;



    public Subsystems(HardwareMap hardwareMap) {
        intake = new IntakeTransfer(hardwareMap);
        launcher = new Launcher(hardwareMap);
        gateRamp = new GateRamp(hardwareMap);
    }

    public Action Initialize() {
        // reset everything imo at autonmous start ig
        return new ParallelAction(
                intake.intakeStop(),
                launcher.spinDown(),
                gateRamp.gateDown(),
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

    public Action LaunchOne(Pose2d pose) {
        double flywheelSpeed = PoseFlywheelSpeed(pose);
        double rampPos = PoseRampPosition(pose);

        Action one = new SequentialAction(
                LaunchPrepare(pose),
                gateRamp.gateDown(),
                intake.launchingRun(launchTransferSpeed),
                launcher.launch(flywheelSpeed),
                gateRamp.gateUp(),
                intake.feedRun(feedIntakeSpeed, feedTransferSpeed)
        );

        return one;
    }

    // NOTE - this is currently setup to launch 3 balls.
    public Action LaunchAll(Pose2d pose) {
        // imma be real i don't know why i made this a separate function from LaunchOne but there may be a reason which future me (or you) will discover
        Action all = new SequentialAction(
                LaunchOne(pose),
                LaunchOne(pose),
                LaunchOne(pose)
        );

        return all;
    }



    public double PoseFlywheelSpeed(Pose2d pose) {
        // fill in with functionality
        double x = pose.position.x;
        double y = pose.position.y;
        double value = 1000;
        return Range.clip(value, 0, 3000);
    }

    public double PoseRampPosition(Pose2d pose) {
        // fill in with functionality
        double x = pose.position.x;
        double y = pose.position.y;

        double value = 0.38;

        return Range.clip(value, 0.35, 0.46);
    }


}
