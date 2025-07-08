package org.firstinspires.ftc.teamcode.teleop.driveModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.teleop.Hardware;
import org.firstinspires.ftc.teamcode.teleop.Utils;

@TeleOp(name = "DriveBase-TeleOp", group = "Dev-Teleops")
public class DriveBaseTeleop extends LinearOpMode {

    // Create Hardware instance
    private final Hardware robot = new Hardware();

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize hardware map
        robot.init(hardwareMap, 1);

        // Coefficients for joystick inputs
        double coefX = 1.1;
        double coefY = 1.0;
        double coefRx = 1.0;

        waitForStart();

        while (opModeIsActive()) {

            // Read joystick inputs and apply coefficients
            double x = gamepad1.left_stick_x * coefX;
            double y = gamepad1.left_stick_y * coefY;
            double rx = -gamepad1.right_stick_x * coefRx;

            // Normalize denominator to keep power in [-1,1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            // Calculate motor powers
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            // Set motor powers
            robot.frontLeftMotor.setPower(frontLeftPower);
            robot.backLeftMotor.setPower(backLeftPower);
            robot.frontRightMotor.setPower(frontRightPower);
            robot.backRightMotor.setPower(backRightPower);

            Utils.displayMotorPowers(telemetry, frontLeftPower, backLeftPower, frontRightPower, backRightPower);
            Utils.displayCodeVersion(telemetry, "7.5.25.17.29");
            telemetry.update();
        }
    }
}