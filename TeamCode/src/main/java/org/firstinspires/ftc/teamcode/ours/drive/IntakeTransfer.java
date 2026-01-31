package org.firstinspires.ftc.teamcode.ours.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeTransfer {
    public CRServo IntakeBrushL, IntakeBrushR, TransferL, TransferR;

    public IntakeTransfer(HardwareMap hardwareMap) {
        IntakeBrushL = hardwareMap.get(CRServo.class, "leftIntake");
        IntakeBrushR = hardwareMap.get(CRServo.class, "rightIntake");
        IntakeBrushL.setDirection(CRServo.Direction.FORWARD);
        IntakeBrushR.setDirection(CRServo.Direction.REVERSE);

        TransferL = hardwareMap.get(CRServo.class, "leftTransfer");
        TransferR = hardwareMap.get(CRServo.class, "rightTransfer");
        TransferL.setDirection(CRServo.Direction.FORWARD);
        TransferR.setDirection(CRServo.Direction.REVERSE);
    }

    public class IntakeRun implements Action {
        private boolean initialized = false;

        private double intakeSpeed;
        private double transferSpeed;

        public IntakeRun(double setIntakeSpeed, double setTransferSpeed) {
            intakeSpeed = setIntakeSpeed;
            transferSpeed = setTransferSpeed;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!initialized) {
                IntakeBrushL.setPower(intakeSpeed);
                IntakeBrushR.setPower(intakeSpeed);

                TransferL.setPower(transferSpeed);
                TransferR.setPower(transferSpeed);

                initialized = true;
            }

            return false; // I don't think there's any condition checking needed.
        }
    }


    public Action intakeRun(double setIntakeSpeed, double setTransferSpeed) {
        return new IntakeRun(setIntakeSpeed, setTransferSpeed);
    }

    public class IntakeStop implements Action {
        private boolean initialized = false;


        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!initialized) {
                IntakeBrushL.setPower(0);
                IntakeBrushR.setPower(0);

                TransferL.setPower(0);
                TransferR.setPower(0);

                initialized = true;
            }

            return false; // I don't think there's any condition checking needed.
        }
    }


    public Action intakeStop() {
        return new IntakeStop();
    }


    public class TransportHoldingRun implements Action {
        private boolean initialized = false;

        private double intakeSpeed;

        public TransportHoldingRun(double setIntakeSpeed) {
            intakeSpeed = setIntakeSpeed;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!initialized) {
                IntakeBrushL.setPower(intakeSpeed);
                IntakeBrushR.setPower(intakeSpeed);

                TransferL.setPower(0);
                TransferR.setPower(0);

                initialized = true;
            }

            return false; // I don't think there's any condition checking needed.
        }
    }


    public Action transportHoldingRun(double setIntakeSpeed) {
        return new TransportHoldingRun(setIntakeSpeed);
    }


    public class LaunchingRun implements Action {
        private boolean initialized = false;

        private double transferSpeed;

        public LaunchingRun(double setTransferSpeed) {
            transferSpeed = setTransferSpeed;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!initialized) {
                IntakeBrushL.setPower(0);
                IntakeBrushR.setPower(0);

                TransferL.setPower(transferSpeed);
                TransferR.setPower(transferSpeed);

                initialized = true;
            }

            return false; // I don't think there's any condition checking needed.
        }
    }


    public Action launchingRun(double setTransferSpeed) {
        return new LaunchingRun(setTransferSpeed);
    }


    public class FeedRun implements Action {
        private boolean initialized = false;

        private double intakeSpeed;
        private double transferSpeed;

        public FeedRun(double setIntakeSpeed, double setTransferSpeed) {
            intakeSpeed = setIntakeSpeed;
            transferSpeed = setTransferSpeed;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!initialized) {
                IntakeBrushL.setPower(intakeSpeed);
                IntakeBrushR.setPower(intakeSpeed);

                TransferL.setPower(transferSpeed);
                TransferR.setPower(transferSpeed);
                initialized = true;
            }

            return false; // I don't think there's any condition checking needed.
        }
    }


    public Action feedRun(double setIntakeSpeed, double setTransferSpeed) {
        return new FeedRun(setIntakeSpeed, setTransferSpeed);
    }

}
