package org.firstinspires.ftc.teamcode.Utils;

/**
 * Utility class containing helper methods for telemetry display and unit conversions.
 */
public final class Telemetry {

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
        telemetry.addData("FL power", frontLeftPower);
        telemetry.addLine("-----------------------------");
        telemetry.addData("BL power", backLeftPower);
        telemetry.addLine("-----------------------------");
        telemetry.addData("FR power", frontRightPower);
        telemetry.addLine("-----------------------------");
        telemetry.addData("BR power", backRightPower);
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
        telemetry.addLine("-----------------------------");
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
        telemetry.addData("x position", x);
        telemetry.addLine("-----------------------------");
        telemetry.addData("y position", y);
        telemetry.addLine("-----------------------------");
        telemetry.addData("heading in degrees", tetha);
    }
}
