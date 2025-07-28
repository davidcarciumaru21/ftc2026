package org.firstinspires.ftc.teamcode.systems.colorSensor;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.config.enums.AllianceColor;

@Config
public class ColorConfig {

    // This will be editable in the FTC Dashboard as a dropdown
    public static AllianceColor alliance = AllianceColor.RED;

    public static int redHsvValue1 = 10;
    public static int redHsvValue2 = 30;

    public static int blueHsvValue1 = 210;
    public static int blueHsvValue2 = 270;

    public static int yellowHsvValue1 = 35;
    public static int yellowHsvValue2 = 70;

    public AllianceColor getColor() {
        return alliance;
    }
    public int getRedHsvValue1() {
        return redHsvValue1;
    }
    public int getRedHsvValue2() {
        return redHsvValue2;
    }

    public int getBlueHsvValue1() {
        return blueHsvValue1;
    }
    public int getBlueHsvValue2() {
        return blueHsvValue2;
    }

    public int getYellowHsvValue1() {
        return yellowHsvValue1;
    }
    public int getYellowHsvValue2() {
        return yellowHsvValue2;
    }
}
