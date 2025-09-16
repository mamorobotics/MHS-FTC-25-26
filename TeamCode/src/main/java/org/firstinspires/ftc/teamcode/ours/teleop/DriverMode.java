package org.firstinspires.ftc.teamcode.ours.teleop;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.IMU;

import org.apache.commons.math3.analysis.function.Max;
import org.apache.commons.math3.analysis.function.Min;
import org.firstinspires.ftc.robotcore.external.navigation.*;


@ TeleOp(name = "TeleOp")
public class DriverMode extends OpMode {
    static DriveTrain driveTrain = new DriveTrain();
    private static DcMotor LiftR, LiftL, ArmM;
    private static Servo ArmS;
    private static Servo clawS;

    int liftTargetPos = 0;
    int armTargetPos = 200;

    double WristServoPos = 0.71;

    int highMetalPos = 3055;
    int lowMetalPos = 1367;
    int highPlasticPos = 2156;
    int lowPlasticPos = 898;
    int highBucketPos = 4200;
    int lowBucketPos = 2156;

    private BHI260IMU imu;

    @Override
    public void init() {
        driveTrain.setDriveTrain(hardwareMap, "leftFront", "leftRear", "rightFront", "rightRear");
        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveTrain.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveTrain.reverse(DriveTrain.DriveTrainSide.LEFT);

        LiftR = hardwareMap.get(DcMotor.class, "LiftMotorRight");
        LiftL = hardwareMap.get(DcMotor.class, "LiftMotorLeft");
        LiftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LiftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ArmM = hardwareMap.get(DcMotor.class, "ArmMotor");
        ArmM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ArmS = hardwareMap.get(Servo.class, "ArmServo");
        clawS = hardwareMap.get(Servo.class, "ClawServo");

        LiftR.setDirection(DcMotorSimple.Direction.REVERSE);

        ArmM.setTargetPosition(armTargetPos);
        ArmM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmM.setPower(1);

        ArmS.setPosition(WristServoPos);
    }

    @Override
    public void loop() {
        telemetry.addData("LIFT:", LiftL.getCurrentPosition());
        telemetry.addData("ARM:", ArmM.getCurrentPosition());
        telemetry.addData("Wrist:", ArmS.getPosition());
        telemetry.update();

        double normalizedTargetSpeed = 1 - (LiftL.getCurrentPosition() / 4200.0);
        double targetSpeed = (0.6 * Math.pow(normalizedTargetSpeed, 2)) + 0.4;
        //deadZone(gamepad1.left_stick_x, 0.2)
        driveTrain.move(gamepad1.right_trigger - gamepad1.left_trigger, -gamepad1.left_stick_y, gamepad1.right_stick_x, targetSpeed);

        manageLift();
        manageArm();
        manageClaw();
    }

    double deadZone(double x, double deadzone) {
        return Math.min(Math.max((1.0/(1-deadzone)) * (x - deadzone), 0), (1.0/(1-deadzone)) * (x + deadzone));
    }

    void manageLift() {
        if(gamepad2.y){
            liftTargetPos = highBucketPos;
        } else if(gamepad2.a) {
            liftTargetPos = 0;
        } else if(gamepad2.b){
            liftTargetPos = lowBucketPos;
        } else if(gamepad2.x){

        } //else if(gamepad2.dpad_down){
//            liftTargetPos = lowMetalPos;
//        } else if(gamepad2.dpad_up){
//            liftTargetPos = highMetalPos;
//        } else if(gamepad2.dpad_left){
//            liftTargetPos = lowPlasticPos;
//        } else if(gamepad2.dpad_right){
//            liftTargetPos = highPlasticPos;
//        }
        LiftL.setTargetPosition(liftTargetPos);
        LiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LiftL.setPower(1);

        LiftR.setTargetPosition(liftTargetPos);
        LiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LiftR.setPower(1);
    }

    void manageArm() {
        armTargetPos += ((gamepad2.dpad_up ? 1 : 0) - (gamepad2.dpad_down ? 1 : 0)) * 10;

        armTargetPos = Math.min(Math.max(armTargetPos, 0), 1820);

        ArmM.setTargetPosition(armTargetPos);
        ArmM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmM.setPower(1);

        if(gamepad2.dpad_right){
            WristServoPos += 0.01;
        }else if(gamepad2.dpad_left){
            WristServoPos -= 0.01;
        }
        WristServoPos = Math.min(Math.max(WristServoPos, 0), 1);
        ArmS.setPosition(WristServoPos);
    }

    void manageClaw() {
        clawS.setPosition(0.05);
        if(gamepad2.left_trigger > 0.9){
            clawS.setPosition(0.4);
        }
        if (gamepad2.right_trigger > 0.9) {
            clawS.setPosition(0.15);
        }
    }
}