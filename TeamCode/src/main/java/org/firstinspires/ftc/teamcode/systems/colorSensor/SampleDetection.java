package org.firstinspires.ftc.teamcode.systems.colorSensor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import android.graphics.Color;

import static org.firstinspires.ftc.teamcode.enums.AllianceColor.*;

import org.firstinspires.ftc.teamcode.enums.AllianceColor;

public class SampleDetection {

    private final NormalizedColorSensor colorSensor;

    public SampleDetection(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "CSS");

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }
    }

    public boolean checkColor() {
        float[] hsv = new float[3];
        Color.colorToHSV(colorSensor.getNormalizedColors().toColor(), hsv);

        AllianceColor alliance = ColorConfig.alliance;

        // RED alliance: detect RED or YELLOW samples
        if (alliance == RED) {
            return isRed(hsv) || isYellow(hsv);
        }

        // BLUE alliance: detect BLUE or YELLOW samples
        return isBlue(hsv) || isYellow(hsv);
    }

    private boolean isRed(float[] hsv) {
        return hsv[0] >= 0 && hsv[0] <= 30;
    }

    private boolean isBlue(float[] hsv) {
        return hsv[0] >= 210 && hsv[0] <= 270;
    }

    private boolean isYellow(float[] hsv) {
        return hsv[0] >= 30 && hsv[0] <= 70;
    }
}
