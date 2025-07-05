package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Utils {

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
}
