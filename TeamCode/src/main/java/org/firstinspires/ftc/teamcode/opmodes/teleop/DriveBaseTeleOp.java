package org.firstinspires.ftc.teamcode.opmodes.teleop;

//==============================Robot Core=============================
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

//==============================Road Runner============================
import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.teamcode.config.ColorConfig;
import org.firstinspires.ftc.teamcode.roadRunner.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunner.localizer.ThreeDeadWheelLocalizer;

//=================================Utils===============================
import org.firstinspires.ftc.teamcode.Utils.TelemetryMethods;

//=============================Configurations==========================
import org.firstinspires.ftc.teamcode.config.enums.DriveType;
import org.firstinspires.ftc.teamcode.config.GamepadsCoefficients;

//=============================File reading============================
import java.io.FileReader;
import java.io.IOException;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import java.io.File;

@TeleOp(name = "DriveBase-typesOfDriving", group = "Dev-Teleops")
public class DriveBaseTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //=============================================================
        //===================VARIABLE INITIALIZATION===================
        //=============================================================

        //=====================Share button states=====================
        boolean currentShareStateGamepad1;
        boolean lastShareGamepad1 = false;
        boolean currentShareStateGamepad2;
        boolean lastShareGamepad2 = false;

        //=========================Drive types=========================
        DriveType driveModeGamepad1 = DriveType.ROBOTCENTRIC;
        DriveType driveModeGamepad2 = DriveType.ROBOTCENTRIC;

        //=========================Coefficients=========================
        double coefXGamepad1;
        double coefYGamepad1;
        double coefRxGamepad1;

        double coefXGamepad2;
        double coefYGamepad2;
        double coefRxGamepad2;

        //==================Motors and gamepad values===================
        double x, y, rx, denominator;
        double leftFrontPower = 0, leftBackPower = 0, rightFrontPower = 0, rightBackPower = 0;
        double botHeading, rotX, rotY;
        double servoPower;

        //=============================================================
        //==================ROAD RUNNER INITIALIZATION=================
        //=============================================================

        //===================Setting the robot pose====================
        Pose2d startPose;
        Pose2d currentPose;

        File file = AppUtil.getInstance().getSettingsFile("robotPosition.json");

        try (FileReader reader = new FileReader(file)) {
            JsonParser parser = new JsonParser();
            JsonObject json = parser.parse(reader).getAsJsonObject();

            double startPoseX = json.get("x").getAsDouble();
            double startPoseY = json.get("y").getAsDouble();
            double startPoseHeading = json.get("heading").getAsDouble();

            startPose = new Pose2d(startPoseX, startPoseY, startPoseHeading);
        } catch (IOException e) {
            startPose = new Pose2d(0, 0, 0);
        }

        //=============Drive and localizer initialization==============
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        ThreeDeadWheelLocalizer driveLocalizer = new ThreeDeadWheelLocalizer(hardwareMap, MecanumDrive.PARAMS.inPerTick, startPose);

        //=====================IMU initialization======================
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN));
        imu.initialize(parameters);

        // Wait for the start button
        waitForStart();

        // Main control loop
        while(opModeIsActive()) {

            //=============================================================
            //=======================DRIVE MOVEMENT========================
            //=============================================================

            //==================First gamepad coefficients==================
            coefXGamepad1 = GamepadsCoefficients.coefXGamepad1;
            coefYGamepad1 = GamepadsCoefficients.coefYGamepad1;
            coefRxGamepad1 = GamepadsCoefficients.coefRxGamepad1;

            if (gamepad1.right_trigger > 0.1) {
                // 50% motor power gamepad1
                coefXGamepad1 = 0.5;
                coefYGamepad1 = 0.5;
                coefRxGamepad1 = 0.5;
            } else if (gamepad1.left_trigger > 0.1) {
                // 25% motor power gamepad1
                coefXGamepad1 = 0.25;
                coefYGamepad1 = 0.25;
                coefRxGamepad1 = 0.25;
            }

            //==================Second gamepad coefficients==================
            coefXGamepad2 = GamepadsCoefficients.coefXGamepad2;
            coefYGamepad2 = GamepadsCoefficients.coefYGamepad2;
            coefRxGamepad2 = GamepadsCoefficients.coefRxGamepad2;

            if (gamepad2.right_trigger > 0.1) {
                // 50% motor power gamepad2
                coefXGamepad2 = 0.5;
                coefYGamepad2 = 0.5;
                coefRxGamepad2 = 0.5;
            } else if (gamepad2.left_trigger > 0.1) {
                // 25% motor power gamepad2
                coefXGamepad2 = 0.25;
                coefYGamepad2 = 0.25;
                coefRxGamepad2 = 0.25;
            }

            //==================Gamepads drive types selection==================
            currentShareStateGamepad1 = gamepad1.share;
            currentShareStateGamepad2 = gamepad2.share;

            if (currentShareStateGamepad1 && !lastShareGamepad1) {
                if (driveModeGamepad1 == DriveType.ROBOTCENTRIC) {
                    driveModeGamepad1 = DriveType.FIELDCENTRIC;
                } else {
                    driveModeGamepad1 = DriveType.ROBOTCENTRIC;
                }
            }

            if (currentShareStateGamepad2 && !lastShareGamepad2) {
                if (driveModeGamepad2 == DriveType.ROBOTCENTRIC) {
                    driveModeGamepad2 = DriveType.FIELDCENTRIC;
                } else {
                    driveModeGamepad2 = DriveType.ROBOTCENTRIC;
                }
            }

            boolean gamepad2Active = Math.abs(gamepad2.left_stick_x) > 0.05 ||
                    Math.abs(gamepad2.left_stick_y) > 0.05 ||
                    Math.abs(gamepad2.right_stick_x) > 0.05;

            if (gamepad2Active) {
                //======================Gamepad2 drivebase======================
                x = gamepad2.left_stick_x * coefXGamepad2;
                y = -gamepad2.left_stick_y * coefYGamepad2;
                rx = gamepad2.right_stick_x * coefRxGamepad2;

                if (driveModeGamepad2 == DriveType.ROBOTCENTRIC) {
                    denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                    leftFrontPower = (y + x + rx) / denominator;
                    leftBackPower = (y - x + rx) / denominator;
                    rightFrontPower = (y - x - rx) / denominator;
                    rightBackPower = (y + x - rx) / denominator;

                } else if (driveModeGamepad2 == DriveType.FIELDCENTRIC) {
                    botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                    rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                    rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                    denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                    leftFrontPower = (rotY + rotX + rx) / denominator;
                    leftBackPower = (rotY - rotX + rx) / denominator;
                    rightFrontPower = (rotY - rotX - rx) / denominator;
                    rightBackPower = (rotY + rotX - rx) / denominator;
                }
            } else {
                x = gamepad1.left_stick_x * coefXGamepad1;
                y =  -gamepad1.left_stick_y * coefYGamepad1;
                rx = gamepad1.right_stick_x * coefRxGamepad1;

                if (driveModeGamepad1 == DriveType.ROBOTCENTRIC) {
                    //======================Gamepad1 drivebase======================
                    denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                    leftFrontPower = (y + x + rx) / denominator;
                    leftBackPower = (y - x + rx) / denominator;
                    rightFrontPower = (y - x - rx) / denominator;
                    rightBackPower = (y + x - rx) / denominator;

                } else if (driveModeGamepad1 == DriveType.FIELDCENTRIC) {
                    botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                    rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                    rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                    denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                    leftFrontPower = (rotY + rotX + rx) / denominator;
                    leftBackPower = (rotY - rotX + rx) / denominator;
                    rightFrontPower = (rotY - rotX - rx) / denominator;
                    rightBackPower = (rotY + rotX - rx) / denominator;
                }
            }

            lastShareGamepad1 = currentShareStateGamepad1;
            lastShareGamepad2 = currentShareStateGamepad2;

            //======================Apllying powers=======================
            drive.leftFront.setPower(leftFrontPower);
            drive.leftBack.setPower(leftBackPower);
            drive.rightFront.setPower(rightFrontPower);
            drive.rightBack.setPower(rightBackPower);

            //=============================================================
            //==========================TELEMETRY==========================
            //=============================================================

            driveLocalizer.update();
            currentPose = driveLocalizer.getPose();

            TelemetryMethods.displayMotorPowers(telemetry, leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
            TelemetryMethods.displayPostion(telemetry, currentPose);
            TelemetryMethods.displayDriveModes(telemetry, driveModeGamepad1, driveModeGamepad2);
            TelemetryMethods.displayAlliance(telemetry, ColorConfig.alliance);
            TelemetryMethods.displayCodeVersion(telemetry, "7.29.25.2.33");
            telemetry.addLine("-----------------------------");
            telemetry.update();
        }
    }
}