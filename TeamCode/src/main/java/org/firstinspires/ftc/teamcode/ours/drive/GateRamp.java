package org.firstinspires.ftc.teamcode.ours.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ours.BotConstants;

public class GateRamp {
    private static Servo Gate, RampL, RampR;

    public GateRamp(HardwareMap hardwareMap) {
        Gate = hardwareMap.get(Servo.class, "gate");

        RampR = hardwareMap.get(Servo.class, "rightRamp");
        RampL = hardwareMap.get(Servo.class, "leftRamp");

        RampL.setDirection(Servo.Direction.REVERSE);
    }

    public class GateDown implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!initialized) {
                Gate.setPosition(BotConstants.GATE_DOWN_POS);
                initialized = false;
            }

            return Gate.getPosition() < BotConstants.GATE_DOWN_POS - 0.005;
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
                initialized = false;
            }

            return Gate.getPosition() > BotConstants.GATE_UP_POS + 0.005;
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

            return !(RampL.getPosition() == pos && RampR.getPosition() == pos);
        }
    }

    public Action rampMove(double rampPos) {
        return new RampMove(rampPos);
    }
}
