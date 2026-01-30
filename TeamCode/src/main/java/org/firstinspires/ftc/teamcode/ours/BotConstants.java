package org.firstinspires.ftc.teamcode.ours;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class BotConstants {
    public static double FlywheelKP = 4.5;
    public static double FlywheelKI = 0;
    public static double FlywheelKD = 125;
    public static double FlywheelKF = 13.6;

    public static PIDFCoefficients flywheelCoefficients = new PIDFCoefficients(FlywheelKP, FlywheelKI, FlywheelKD, FlywheelKF);

    public static double LAUNCH_AMPS = 2.5;

    public static double RAMP_NEUTRAL_POS = 0.38;

    public static double GATE_UP_POS = 0.04;

    public static double GATE_DOWN_POS = 0.01;

    public static double GOAL_X = -63;
    public static double GOAL_Y = -51;

}
