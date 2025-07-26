package org.firstinspires.ftc.teamcode.systems.colorSensor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import android.graphics.Color;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Color Sensor HSV Test", group = "Test")
public class ColorSensorHSVTest extends LinearOpMode {

    private NormalizedColorSensor colorSensor;
    private DistanceSensor distanceSensor;

    @Override
    public void runOpMode() {

        // Initialize the sensor
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "CSS");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "CSS");

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }

        telemetry.addLine("Ready to start");
        telemetry.update();
        waitForStart();

        float[] hsv = new float[3];

        while (opModeIsActive()) {
            Color.colorToHSV(colorSensor.getNormalizedColors().toColor(), hsv);

            telemetry.addData("Hue", hsv[0]);
            telemetry.addData("Saturation", hsv[1]);
            telemetry.addData("Value (Brightness)", hsv[2]);
            telemetry.addData("Alpha", colorSensor.getNormalizedColors().alpha);
            telemetry.addData("Distance (cm)", distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}
