package org.firstinspires.ftc.teamcode.Utils;

import org.firstinspires.ftc.teamcode.config.enums.DriveType;
import org.firstinspires.ftc.teamcode.config.enums.AllianceColor;

public final class TelemetryMethods {

    /**
     * Displays the power levels of the four drivetrain motors on the telemetry.
     *
     * @param telemetry The telemetry object for displaying data on the driver station.
     * @param frontLeftPower Power of the front-left motor.
     * @param backLeftPower Power of the back-left motor.
     * @param frontRightPower Power of the front-right motor.
     * @param backRightPower Power of the back-right motor.
     */
    public static void displayMotorPowers(org.firstinspires.ftc.robotcore.external.Telemetry telemetry,
                                          double frontLeftPower,
                                          double backLeftPower,
                                          double frontRightPower,
                                          double backRightPower) {
        telemetry.addLine("-----------------------------");
        telemetry.addData("FL Power", frontLeftPower);
        telemetry.addData("BL Power", backLeftPower);
        telemetry.addData("FR Power", frontRightPower);
        telemetry.addData("BR Power", backRightPower);
    }

    /**
     * Displays the version string of the code on the telemetry.
     * Useful for debugging and tracking code updates.
     *
     * @param telemetry The telemetry object to send data to driver station.
     * @param version The version string of the code.
     */
    public static void displayCodeVersion(org.firstinspires.ftc.robotcore.external.Telemetry telemetry, String version) {
        telemetry.addLine("-----------------------------");
        telemetry.addData("Version", version);
    }

    /**
     * Displays the robot's current position and heading on the telemetry.
     *
     * @param telemetry The telemetry object for output.
     * @param x The x-coordinate position.
     * @param y The y-coordinate position.
     * @param tetha The heading (orientation) in degrees.
     */
    public static void displayCoordonates(org.firstinspires.ftc.robotcore.external.Telemetry telemetry, double x, double y, double tetha) {
        telemetry.addLine("-----------------------------");
        telemetry.addData("X Position", x);
        telemetry.addData("Y Position", y);
        telemetry.addData("Heading In Degrees", tetha);
    }

    /**
     * Displays the driving mode of each gamepad.
     *
     * @param telemetry The telemetry object for output.
     * @param gamepad1DriveType First gamepad driving type.
     * @param gamepad2DriveType Second gamepad driving type.
     */
    public static void displayDriveModes(org.firstinspires.ftc.robotcore.external.Telemetry telemetry, DriveType gamepad1DriveType,  DriveType gamepad2DriveType) {
        telemetry.addLine("-----------------------------");
        if (gamepad1DriveType == DriveType.ROBOTCENTRIC) {
            telemetry.addLine("Gamepad1 Drive-Type: ROBOT-CENTRIC");
        } else {
            telemetry.addLine("Gamepad1 Drive-Type: FIELD-CENTRIC");
        }

        if (gamepad2DriveType == DriveType.ROBOTCENTRIC) {
            telemetry.addLine("Gamepad2 Drive-Type: ROBOT-CENTRIC");
        } else {
            telemetry.addLine("Gamepad2 Drive-Type: FIELD-CENTRIC");
        }
    }

    /**
     * Displays the driving mode of each gamepad.
     *
     * @param telemetry The telemetry object for output.
     * @param valid This shows if the sample in front of the intake has a valid color or not.
     */
    public static void displaySampleValidation(org.firstinspires.ftc.robotcore.external.Telemetry telemetry, boolean valid) {
        telemetry.addLine("-----------------------------");
        telemetry.addData("Valid Sample Type", valid);
    }

    /**
     * Displays the driving mode of each gamepad.
     *
     * @param telemetry The telemetry object for output.
     * @param AllianceColorVar This shows the color of the alliance.
     */
    public static void displayAlliance(org.firstinspires.ftc.robotcore.external.Telemetry telemetry, AllianceColor AllianceColorVar) {
        telemetry.addLine("-----------------------------");
        if (AllianceColorVar == AllianceColor.RED) {
            telemetry.addLine("Color Of The Alliance: RED");
        } else if (AllianceColorVar == AllianceColor.BLUE) {
            telemetry.addLine("Color Of The Alliance: BLUE");
        }
    }
}
