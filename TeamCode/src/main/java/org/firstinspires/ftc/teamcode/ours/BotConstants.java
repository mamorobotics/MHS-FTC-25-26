package org.firstinspires.ftc.teamcode.ours;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class BotConstants {
    public static double FlywheelKP = 4.5;
    public static double FlywheelKI = 0;
    public static double FlywheelKD = 125;
    public static double FlywheelKF = 13.6;

    public static PIDFCoefficients flywheelCoefficients = new PIDFCoefficients(FlywheelKP, FlywheelKI, FlywheelKD, FlywheelKF);

}
