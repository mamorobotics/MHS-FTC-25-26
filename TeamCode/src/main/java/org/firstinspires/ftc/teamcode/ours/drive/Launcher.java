package org.firstinspires.ftc.teamcode.ours.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.ours.BotConstants;

public class Launcher {
    public DcMotorEx FlyRight, FlyLeft;

    private Telemetry telemetry1;

    public Launcher(HardwareMap hardwareMap, Telemetry telemetry) {
        FlyLeft = hardwareMap.get(DcMotorEx.class, "leftLauncher");
        FlyRight = hardwareMap.get(DcMotorEx.class, "rightLauncher");

        FlyLeft.setDirection(DcMotor.Direction.REVERSE);
        FlyRight.setDirection(DcMotor.Direction.FORWARD);

        FlyLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FlyRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FlyLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FlyRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry1 = telemetry;
    }

    public class SpinUp implements Action {
        private boolean initialized = false;

        private double speed;

        public SpinUp(double setSpeed) {
            speed = setSpeed;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                FlyLeft.setVelocity(speed);
                FlyRight.setVelocity(speed);
                initialized = true;
            }

            double Lvel = FlyLeft.getVelocity();
            double Rvel = FlyRight.getVelocity();
            packet.put("leftVelocity", Lvel);
            packet.put("rightVelocity", Rvel);
            boolean above = Lvel > speed * 0.96 && Rvel > speed * 0.96;
            boolean below = Lvel < speed * 1.06 && Rvel < speed * 1.06;
            return !(above && below);
        }
    }

    public Action spinUp(double setSpeed) {
        return new SpinUp(setSpeed);
    }

    public class SpinDown implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                FlyLeft.setVelocity(0);
                FlyRight.setVelocity(0);
                initialized = true;
            }

            double Lvel = FlyLeft.getVelocity();
            double Rvel = FlyRight.getVelocity();
            packet.put("leftVelocity", Lvel);
            packet.put("rightVelocity", Rvel);
            return Lvel-80 > 0 && Rvel-80 > 0;
        }
    }

    public Action spinDown() {
        return new SpinDown();
    }


    public class Launch implements Action {
        private boolean initialized = false;

        private double speed;

        double launchSpeed;

        public Launch(double setSpeed) {

            speed = setSpeed;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                FlyLeft.setVelocity(speed);
                FlyRight.setVelocity(speed);
                initialized = true;
            }

            double Lvel = FlyLeft.getVelocity();
            double Rvel = FlyRight.getVelocity();
            packet.put("leftVelocity", Lvel);
            packet.put("rightVelocity", Rvel);

            boolean leftLaunched = FlyLeft.getCurrent(CurrentUnit.AMPS) > BotConstants.LAUNCH_AMPS;
            boolean rightLaunched = FlyRight.getCurrent(CurrentUnit.AMPS) > BotConstants.LAUNCH_AMPS;
            telemetry1.addData("LCurrent", FlyLeft.getCurrent(CurrentUnit.AMPS));
            telemetry1.update();


            return true;
        }
    }

    public Action launch(double setSpeed) {
        return new Launch(setSpeed);
    }

}
