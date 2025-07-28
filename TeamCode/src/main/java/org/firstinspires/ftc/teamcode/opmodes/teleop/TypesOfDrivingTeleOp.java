package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.config.enums.RobotInitialization;
import org.firstinspires.ftc.teamcode.utils.TelemetryMethods;
import org.firstinspires.ftc.teamcode.systems.arm.Positions;
import org.firstinspires.ftc.teamcode.systems.robotHardware.Hardware;
import org.firstinspires.ftc.teamcode.utils.Gamepads;

@TeleOp(name = "DriveBase-typesOfDriving", group = "Dev-Teleops")
@Disabled
public class TypesOfDrivingTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // SHARE button state tracking for toggling drive modes
        boolean currentShareStateGamepad1;
        boolean lastShareGamepad1 = false;
        boolean currentShareStateGamepad2;
        boolean lastShareGamepad2 = false;

        // Drive mode tracking: 1 = normal, 2 = field-centric
        byte driveModeGamepad1 = 1, driveModeGamepad2 = 1;

        // Create and initialize hardware instance
        final Hardware robotHardware = new Hardware();
        robotHardware.init(hardwareMap, RobotInitialization.WithoutRoadRunner);

        // Speed scaling coefficients for both gamepads
        double coefXGamepad1 = 1.0;
        double coefYGamepad1 = 1.0;
        double coefRxGamepad1 = 1.0;

        double coefXGamepad2 = 1.0;
        double coefYGamepad2 = 1.0;
        double coefRxGamepad2 = 1.0;

        // Movement variables
        double x, y, rx, denominator;
        double frontLeftPower = 0, backLeftPower = 0, frontRightPower = 0, backRightPower = 0;
        double botHeading, rotX, rotY;

        // Initialize IMU (Inertial Measurement Unit) with hub orientation
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN));
        imu.initialize(parameters);

        Positions.ArmPosition basketPos = Positions.getBasket();
        Positions.ArmPosition perimeterPos = Positions.getPerimeter();
        Positions.ArmPosition specimenUpPos = Positions.getSpecimenUp();
        Positions.ArmPosition specimenDownPos = Positions.getSpecimenDown();
        Positions.ArmPosition submersibil1 = Positions.getSubmersibil1();
        Positions.ArmPosition submersibil2 = Positions.getSubmersibil2();
        Positions.ArmPosition basket3 = Positions.getBasket3();
        Positions.ArmPosition perimeterUp = Positions.getPerimeterUP();

        // Wait for the start button
        waitForStart();

        // Main control loop
        while (opModeIsActive()) {

            // === Update speed coefficients based on Gamepad1 bumper input ===
            coefXGamepad1 = 1.0;
            coefYGamepad1 = 1.0;
            coefRxGamepad1 = 1.0;

            if (gamepad1.right_bumper) {
                // 50% speed mode
                Gamepads.rightBumperRumble(gamepad1);
                coefXGamepad1 = 0.5;
                coefYGamepad1 = 0.5;
                coefRxGamepad1 = 0.5;
            } else if (gamepad1.left_bumper) {
                Gamepads.leftBumperRumble(gamepad1);
                // 25% precision mode
                coefXGamepad1 = 0.25;
                coefYGamepad1 = 0.25;
                coefRxGamepad1 = 0.25;
            }

            // === Same logic for Gamepad2 bumpers ===
            coefXGamepad2 = 1.0;
            coefYGamepad2 = 1.0;
            coefRxGamepad2 = 1.0;

            if (gamepad2.right_bumper) {
                Gamepads.rightBumperRumble(gamepad2);
                coefXGamepad2 = 0.5;
                coefYGamepad2 = 0.5;
                coefRxGamepad2 = 0.5;
            } else if (gamepad2.left_bumper) {
                Gamepads.leftBumperRumble(gamepad2);
                coefXGamepad2 = 0.25;
                coefYGamepad2 = 0.25;
                coefRxGamepad2 = 0.25;
            }

            // === Detect drive mode toggle via SHARE button press ===
            currentShareStateGamepad1 = gamepad1.share;
            currentShareStateGamepad2 = gamepad2.share;

            // Toggle Gamepad1 mode: 1 ↔ 2
            if (currentShareStateGamepad1 && !lastShareGamepad1) {
                if (driveModeGamepad1 == 1) {
                    driveModeGamepad1 = 2;
                } else {
                    driveModeGamepad1 = 1;
                }
            }

            // Toggle Gamepad2 mode: 1 ↔ 2
            if (currentShareStateGamepad2 && !lastShareGamepad2) {
                if (driveModeGamepad2 == 1) {
                    driveModeGamepad2 = 2;
                } else {
                    driveModeGamepad2 = 1;
                }
            }

            // === Prioritize Gamepad2 control if any joystick input is detected ===
            boolean gamepad2Active = Math.abs(gamepad2.left_stick_x) > 0.05 ||
                    Math.abs(gamepad2.left_stick_y) > 0.05 ||
                    Math.abs(gamepad2.right_stick_x) > 0.05;

            if (gamepad2Active) {
                // === Gamepad2 controls drive ===
                x = -gamepad2.left_stick_x * coefXGamepad2;
                y = gamepad2.left_stick_y * coefYGamepad2;
                rx = -gamepad2.right_stick_x * coefRxGamepad2;

                if (driveModeGamepad2 == 1) {
                    // Standard (robot-centric) driving
                    denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                    frontLeftPower = (y + x + rx) / denominator;
                    backLeftPower = (y - x + rx) / denominator;
                    frontRightPower = (y - x - rx) / denominator;
                    backRightPower = (y + x - rx) / denominator;

                } else {
                    // Field-centric driving using IMU heading
                    botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                    rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                    rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                    denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                    frontLeftPower = (rotY + rotX + rx) / denominator;
                    backLeftPower = (rotY - rotX + rx) / denominator;
                    frontRightPower = (rotY - rotX - rx) / denominator;
                    backRightPower = (rotY + rotX - rx) / denominator;
                }
            } else {
                // === Gamepad1 takes over if Gamepad2 is idle ===
                x = -gamepad1.left_stick_x * coefXGamepad1;
                y = gamepad1.left_stick_y * coefYGamepad1;
                rx = -gamepad1.right_stick_x * coefRxGamepad1;

                if (driveModeGamepad1 == 1) {
                    // Robot-centric
                    denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                    frontLeftPower = (y + x + rx) / denominator;
                    backLeftPower = (y - x + rx) / denominator;
                    frontRightPower = (y - x - rx) / denominator;
                    backRightPower = (y + x - rx) / denominator;

                } else {
                    // Field-centric
                    botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                    rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                    rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                    denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                    frontLeftPower = (rotY + rotX + rx) / denominator;
                    backLeftPower = (rotY - rotX + rx) / denominator;
                    frontRightPower = (rotY - rotX - rx) / denominator;
                    backRightPower = (rotY + rotX - rx) / denominator;
                }
            }

            // === Update last SHARE button states for edge detection ===
            lastShareGamepad1 = currentShareStateGamepad1;
            lastShareGamepad2 = currentShareStateGamepad2;

            // === Set calculated powers to motors ===
            robotHardware.frontLeftMotor.setPower(frontLeftPower);
            robotHardware.backLeftMotor.setPower(backLeftPower);
            robotHardware.frontRightMotor.setPower(frontRightPower);
            robotHardware.backRightMotor.setPower(backRightPower);

            // === Display telemetry for debugging and monitoring ===
            TelemetryMethods.displayMotorPowers(telemetry, frontLeftPower, backLeftPower, frontRightPower, backRightPower);
            TelemetryMethods.displayCodeVersion(telemetry, "7.15.25.5.51");
            telemetry.update();
        }
    }
}