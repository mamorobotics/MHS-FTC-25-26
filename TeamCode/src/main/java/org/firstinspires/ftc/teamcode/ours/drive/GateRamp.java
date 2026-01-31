package org.firstinspires.ftc.teamcode.ours.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ours.BotConstants;

public class GateRamp {
    public Servo Gate, RampL, RampR;

    private Telemetry telemetry1;

    public GateRamp(HardwareMap hardwareMap, Telemetry telemetry) {
        Gate = hardwareMap.get(Servo.class, "gate");

        RampR = hardwareMap.get(Servo.class, "rightRamp");
        RampL = hardwareMap.get(Servo.class, "leftRamp");

        RampL.setDirection(Servo.Direction.REVERSE);

        telemetry1 = telemetry;
    }

    public class GateDown implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!initialized) {
                Gate.setPosition(BotConstants.GATE_DOWN_POS);
                initialized = true;
            }

            return false;

        }
    }

    public Action gateDown() {
        return new GateDown();
    }

    public class GateUp implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!initialized) {
                Gate.setPosition(BotConstants.GATE_UP_POS);
                initialized = true;
            }


            return false;
        }
    }

    public Action gateUp() {
        return new GateUp();
    }


    public class RampMove implements Action {
        private boolean initialized = false;

        private double pos;

        public RampMove(double rampPos) {
            pos = rampPos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                RampL.setPosition(pos);
                RampR.setPosition(pos);
                initialized = true;
            }

            return false;
        }
    }

    public Action rampMove(double rampPos) {
        return new RampMove(rampPos);
    }
}
