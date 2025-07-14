package org.firstinspires.ftc.teamcode.Utils;

import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public final class Utils {

    public static void displayMotorPowers(Telemetry telemetry,
                                          double frontLeftPower,
                                          double backLeftPower,
                                          double frontRightPower,
                                          double backRightPower) {
        telemetry.addData("FL power", frontLeftPower);
        telemetry.addLine("-----------------------------");
        telemetry.addData("BL power", backLeftPower);
        telemetry.addLine("-----------------------------");
        telemetry.addData("FR power", frontRightPower);
        telemetry.addLine("-----------------------------");
        telemetry.addData("BR power", backRightPower);
    }

    public static void displayCodeVersion(Telemetry telemetry, String version) {
        telemetry.addLine("-----------------------------");
        telemetry.addData("Version", version);
        telemetry.addLine("-----------------------------");
    }

    public static void displayCoordonates(Telemetry telemetry, double x, double y, double tetha) {
        telemetry.addData("x position", x);
        telemetry.addLine("-----------------------------");
        telemetry.addData("y position", y);
        telemetry.addLine("-----------------------------");
        telemetry.addData("heading in degrees", tetha);
    }

    public static double cmToInches(double cm) {
        return cm / 2.54;
    }

    public static double inchesToCm(double cm) {
        return cm * 2.54;
    }

    public static Vector2d robotNode(Vector2d poseCoordinates) {
        double xNode = poseCoordinates.x;
        double yNode = poseCoordinates.y;
        return new Vector2d(xNode, yNode);
    }
}
