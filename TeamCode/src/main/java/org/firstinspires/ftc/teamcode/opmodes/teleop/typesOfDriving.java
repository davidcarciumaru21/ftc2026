package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.Utils.Utils;
import org.firstinspires.ftc.teamcode.robotHardware.Hardware;

@TeleOp(name = "DriveBase-typesOfDriving", group = "Dev-Teleops")
public class typesOfDriving extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {

        boolean currentShareStateGamepad1;
        boolean lastShareGamepad1 = false;
        boolean currentShareStateGamepad2;
        boolean lastShareGamepad2 = false;

        byte driveModeGamepad1 = 1, driveModeGamepad2 = 1;

        // Create Hardware instance
        final Hardware robotHardware = new Hardware();

        // Initialize hardware map
        robotHardware.init(hardwareMap, 2);

        double coefXGamepad1 = 1.0;
        double coefYGamepad1 = 1.0;
        double coefRxGamepad1 = 1.0;

        double coefXGamepad2 = 1.0;
        double coefYGamepad2 = 1.0;
        double coefRxGamepad2 = 1.0;

        double x, y, rx, denominator;
        double frontLeftPower = 0, backLeftPower = 0, frontRightPower = 0, backRightPower = 0;

        double botHeading, rotX, rotY;

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN));
        imu.initialize(parameters);

        waitForStart();

        while(opModeIsActive()) {

            coefXGamepad1 = 1.0;
            coefYGamepad1 = 1.0;
            coefRxGamepad1 = 1.0;
            coefXGamepad2 = 1.0;
            coefYGamepad2 = 1.0;
            coefRxGamepad2 = 1.0;

            if (gamepad1.right_bumper) {
                coefXGamepad1 = 0.5;
                coefYGamepad1 = 0.5;
                coefRxGamepad1 = 0.5;
            } else if (gamepad1.left_bumper) {
                coefXGamepad1 = 0.25;
                coefYGamepad1 = 0.25;
                coefRxGamepad1 = 0.25;
            }

            if (gamepad2.right_bumper) {
                coefXGamepad2 = 0.5;
                coefYGamepad2 = 0.5;
                coefRxGamepad2 = 0.5;
            } else if (gamepad2.left_bumper) {
                coefXGamepad2 = 0.25;
                coefYGamepad2 = 0.25;
                coefRxGamepad2 = 0.25;
            }

            currentShareStateGamepad1 = gamepad1.share;
            currentShareStateGamepad2 = gamepad2.share;

            if (currentShareStateGamepad1 && !lastShareGamepad1 && driveModeGamepad1 == 1 ) {
                driveModeGamepad1 = 2;
            } else if(currentShareStateGamepad1 && !lastShareGamepad1) {
                driveModeGamepad1 = 1;
            }

            if (currentShareStateGamepad2 && !lastShareGamepad2 && driveModeGamepad2 == 1) {
                driveModeGamepad2 = 2;
            } else if(currentShareStateGamepad2 && !lastShareGamepad2) {
                driveModeGamepad2 = 1;
            }

            if (driveModeGamepad2 == 1) {
                // Read joystick inputs and apply coefficients
                x = gamepad1.left_stick_x * coefXGamepad2;
                y = gamepad1.left_stick_y * coefYGamepad2;
                rx = -gamepad1.right_stick_x * coefRxGamepad2;

                // Normalize denominator to keep power in [-1,1]
                denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

                // Calculate motor powers
                frontLeftPower = (y + x + rx) / denominator;
                backLeftPower = (y - x + rx) / denominator;
                frontRightPower = (y - x - rx) / denominator;
                backRightPower = (y + x - rx) / denominator;

            } else if (driveModeGamepad2 == 2) {
                // Read joystick inputs and apply coefficients
                x = gamepad1.left_stick_x * coefXGamepad2;
                y = gamepad1.left_stick_y * coefYGamepad2;
                rx = -gamepad1.right_stick_x * coefRxGamepad2;

                botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                // Rotate the movement direction counter to the bot's rotation
                rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                rotX *= coefXGamepad2;
                rotY *= coefYGamepad2;
                rx *= coefRxGamepad2;

                denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                frontLeftPower = (rotY + rotX + rx) / denominator;
                backLeftPower = (rotY - rotX + rx) / denominator;
                frontRightPower = (rotY - rotX - rx) / denominator;
                backRightPower = (rotY + rotX - rx) / denominator;
            }

            if (driveModeGamepad1 == 1) {
                // Read joystick inputs and apply coefficients
                x = gamepad1.left_stick_x * coefXGamepad1;
                y = gamepad1.left_stick_y * coefYGamepad1;
                rx = -gamepad1.right_stick_x * coefRxGamepad1;

                // Normalize denominator to keep power in [-1,1]
                denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

                // Calculate motor powers
                frontLeftPower = (y + x + rx) / denominator;
                backLeftPower = (y - x + rx) / denominator;
                frontRightPower = (y - x - rx) / denominator;
                backRightPower = (y + x - rx) / denominator;

            } else if (driveModeGamepad1 == 2) {
                // Read joystick inputs and apply coefficients
                x = gamepad1.left_stick_x * coefXGamepad1;
                y = gamepad1.left_stick_y * coefYGamepad1;
                rx = -gamepad1.right_stick_x * coefRxGamepad1;

                botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                // Rotate the movement direction counter to the bot's rotation
                rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                rotX *= coefXGamepad1;
                rotY *= coefYGamepad1;
                rx *= coefRxGamepad1;

                denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                frontLeftPower = (rotY + rotX + rx) / denominator;
                backLeftPower = (rotY - rotX + rx) / denominator;
                frontRightPower = (rotY - rotX - rx) / denominator;
                backRightPower = (rotY + rotX - rx) / denominator;
            }

            lastShareGamepad1 = currentShareStateGamepad1;
            lastShareGamepad2 = currentShareStateGamepad2;

            robotHardware.frontLeftMotor.setPower(frontLeftPower);
            robotHardware.backLeftMotor.setPower(backLeftPower);
            robotHardware.frontRightMotor.setPower(frontRightPower);
            robotHardware.backRightMotor.setPower(backRightPower);

            Utils.displayMotorPowers(telemetry, frontLeftPower, backLeftPower, frontRightPower, backRightPower);
            Utils.displayCodeVersion(telemetry, "7.15.25.5.51");
            telemetry.update();
        }

    }
}