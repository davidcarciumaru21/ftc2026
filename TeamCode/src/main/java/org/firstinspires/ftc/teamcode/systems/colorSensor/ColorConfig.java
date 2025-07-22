package org.firstinspires.ftc.teamcode.systems.colorSensor;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.enums.AllianceColor;

@Config
public class ColorConfig {

    // This will be editable in the FTC Dashboard as a dropdown
    public static AllianceColor alliance = AllianceColor.RED;

    public AllianceColor getColor() {
        return alliance;
    }
}
