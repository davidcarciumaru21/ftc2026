package org.firstinspires.ftc.teamcode.systems.colorSensor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import android.graphics.Color;

import static org.firstinspires.ftc.teamcode.enums.AllianceColor.*;

import org.firstinspires.ftc.teamcode.enums.AllianceColor;

public class SampleDetection {
    int redHsv1 = ColorConfig.redHsvValue1;
    int redHsv2 = ColorConfig.redHsvValue2;

    int yellowHsv1 = ColorConfig.yellowHsvValue1;
    int yellowHsv2 = ColorConfig.yellowHsvValue2;

    int blueHsv1 = ColorConfig.blueHsvValue1;
    int blueHsv2 = ColorConfig.blueHsvValue2;
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
        return hsv[0] >= redHsv1 && hsv[0] <= redHsv2;
    }

    private boolean isBlue(float[] hsv) {
        return hsv[0] >= blueHsv1 && hsv[0] <= blueHsv2;
    }

    private boolean isYellow(float[] hsv) {
        return hsv[0] >= yellowHsv1 && hsv[0] <= yellowHsv2;
    }
}
