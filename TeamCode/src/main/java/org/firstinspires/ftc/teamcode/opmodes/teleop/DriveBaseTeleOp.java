package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.TelemetryMethods;
import org.firstinspires.ftc.teamcode.systems.robotHardware.Hardware;

@TeleOp(name = "DriveBase-TeleOp", group = "Dev-Teleops")
public class DriveBaseTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Create and initialize robot hardware instance
        final Hardware robot = new Hardware();
        robot.init(hardwareMap, (byte) 1);

        // Joystick sensitivity coefficients for tuning strafing, forward, and rotation control
        double coefX = 1.1;  // Strafing (left/right) compensation factor
        double coefY = 1.0;  // Forward/backward multiplier
        double coefRx = 1.0; // Rotation multiplier

        // Wait for start button to be pressed
        waitForStart();

        while (opModeIsActive()) {

            // Read joystick inputs (inverted right stick x for intuitive turning)
            double x = gamepad1.left_stick_x * coefX;
            double y = gamepad1.left_stick_y * coefY;
            double rx = -gamepad1.right_stick_x * coefRx;

            // Normalize power scaling to prevent motor commands exceeding max power range
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            // Standard mecanum wheel drive formulas
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            // Apply calculated power to motors
            robot.frontLeftMotor.setPower(frontLeftPower);
            robot.backLeftMotor.setPower(backLeftPower);
            robot.frontRightMotor.setPower(frontRightPower);
            robot.backRightMotor.setPower(backRightPower);

            // Display motor power values on telemetry for debugging
            TelemetryMethods.displayMotorPowers(telemetry, frontLeftPower, backLeftPower, frontRightPower, backRightPower);

            // Display version tag (for tracking builds or testing releases)
            TelemetryMethods.displayCodeVersion(telemetry, "7.5.25.17.29");

            // Push telemetry to driver station
            telemetry.update();
        }
    }
}
