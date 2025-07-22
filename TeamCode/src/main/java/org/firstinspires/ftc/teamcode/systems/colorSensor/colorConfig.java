package org.firstinspires.ftc.teamcode.systems.colorSensor;

import com.acmerobotics.dashboard.config.Config;

@Config
public class colorConfig {
    public enum AllianceColor {
        RED,
        BLUE,
    }

    // FTC Dashboard will now show a dropdown with RED and BLUE options
    public static AllianceColor alliance = AllianceColor.RED;

    public AllianceColor getColor() {
        return alliance;
    }
}
