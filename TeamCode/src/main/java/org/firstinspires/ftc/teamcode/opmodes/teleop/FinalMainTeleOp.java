package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.systems.colorSensor.ColorConfig;
import org.firstinspires.ftc.teamcode.utils.TelemetryMethods;
import org.firstinspires.ftc.teamcode.systems.arm.JacobianArm;
import org.firstinspires.ftc.teamcode.systems.arm.Positions;
import org.firstinspires.ftc.teamcode.utils.Gamepads;
import org.firstinspires.ftc.teamcode.systems.robotHardware.Hardware;
import org.firstinspires.ftc.teamcode.systems.colorSensor.SampleDetection;

import org.firstinspires.ftc.teamcode.enums.DriveType;
import org.firstinspires.ftc.teamcode.enums.RobotInitialization;

@TeleOp(name = "FinalMain-TeleOp", group = "Use")
public class FinalMainTeleOp extends LinearOpMode {

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

        //=========================Coeficients=========================
        double coefXGamepad1 = 1.0;
        double coefYGamepad1 = 1.0;
        double coefRxGamepad1 = 1.0;

        double coefXGamepad2 = 1.0;
        double coefYGamepad2 = 1.0;
        double coefRxGamepad2 = 1.0;

        //==================Motor and gamepad values===================
        double x, y, rx, denominator;
        double frontLeftPower = 0, backLeftPower = 0, frontRightPower = 0, backRightPower = 0;
        double botHeading, rotX, rotY;

        //=============================================================
        //===================HARDWARE INITIALIZATION===================
        //=============================================================


        //===================Drivebase initialization==================
        final Hardware robotHardware = new Hardware();
        robotHardware.init(hardwareMap, RobotInitialization.WithoutRoadRunner); // (1)-When not using Road Runner; (2)-When using Road Runner.

        //=====================IMU initialization======================
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN));
        imu.initialize(parameters);

        //================Arm and intake initialization=================
        JacobianArm arm = new JacobianArm(hardwareMap);

        CRServo intake = hardwareMap.get(CRServo.class, "intake");

        SampleDetection sampleDetector = new SampleDetection(hardwareMap);

        waitForStart();

        //========================Arm positions=========================
        Positions.ArmPosition basketPos = Positions.getBasket();
        Positions.ArmPosition perimeterPos = Positions.getPerimeter();
        Positions.ArmPosition specimenUpPos = Positions.getSpecimenUp();
        Positions.ArmPosition specimenDownPos = Positions.getSpecimenDown();
        Positions.ArmPosition submersibil1 = Positions.getSubmersibil1();
        Positions.ArmPosition submersibil2 = Positions.getSubmersibil2();
        Positions.ArmPosition basket3 = Positions.getBasket3();
        Positions.ArmPosition perimeterUp = Positions.getPerimeterUP();

        while(opModeIsActive()) {

            //=============================================================
            //======================CHASSIS MOVEMENT=======================
            //=============================================================

            //==================First gamepad coeficients==================
            coefXGamepad1 = 1.0;
            coefYGamepad1 = 1.0;
            coefRxGamepad1 = 1.0;

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

            //==================Second gamepad coeficients==================
            coefXGamepad2 = 1.0;
            coefYGamepad2 = 1.0;
            coefRxGamepad2 = 1.0;

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
                x = -gamepad2.left_stick_x * coefXGamepad2;
                y = gamepad2.left_stick_y * coefYGamepad2;
                rx = -gamepad2.right_stick_x * coefRxGamepad2;

                if (driveModeGamepad2 == DriveType.ROBOTCENTRIC) {
                    denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                    frontLeftPower = (y + x + rx) / denominator;
                    backLeftPower = (y - x + rx) / denominator;
                    frontRightPower = (y - x - rx) / denominator;
                    backRightPower = (y + x - rx) / denominator;

                } else {
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
                x = -gamepad1.left_stick_x * coefXGamepad1;
                y = gamepad1.left_stick_y * coefYGamepad1;
                rx = -gamepad1.right_stick_x * coefRxGamepad1;

                if (driveModeGamepad1 == DriveType.ROBOTCENTRIC) {
                    //======================Gamepad1 drivebase======================
                    denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                    frontLeftPower = (y + x + rx) / denominator;
                    backLeftPower = (y - x + rx) / denominator;
                    frontRightPower = (y - x - rx) / denominator;
                    backRightPower = (y + x - rx) / denominator;

                } else {
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

            lastShareGamepad1 = currentShareStateGamepad1;
            lastShareGamepad2 = currentShareStateGamepad2;

            //======================Apllying powers=======================
            robotHardware.frontLeftMotor.setPower(frontLeftPower);
            robotHardware.backLeftMotor.setPower(backLeftPower);
            robotHardware.frontRightMotor.setPower(frontRightPower);
            robotHardware.backRightMotor.setPower(backRightPower);

            //=============================================================
            //========================ARM MOVEMENT=========================
            //=============================================================

            //======================Arm poseitions gamepad1================
            if (gamepad1.dpad_up) {
                arm.ArmGoto(basketPos.x, basketPos.y, basketPos.elbowUp);
            } else if (gamepad1.dpad_down) {
                arm.ArmGoto(submersibil1.x, submersibil1.y, submersibil1.elbowUp);
            } else if (gamepad1.dpad_right) {
                arm.ArmGoto(submersibil2.x, submersibil2.y, submersibil2.elbowUp);
            } else if (gamepad1.dpad_left) {
                arm.ArmGoto(basket3.x, basket3.y, basket3.elbowUp);
            }

            //======================Arm poseitions gamepad2================
            if (gamepad2.dpad_down) {
                arm.ArmGoto(perimeterPos.x, perimeterPos.y, perimeterPos.elbowUp);
            } else if (gamepad2.dpad_right) {
                arm.ArmGoto(specimenUpPos.x, specimenUpPos.y, specimenUpPos.elbowUp);
            } else if (gamepad2.dpad_left) {
                arm.ArmGoto(specimenDownPos.x, specimenDownPos.y, specimenDownPos.elbowUp);
            } else if (gamepad2.dpad_up) {
                arm.ArmGoto(perimeterUp.x, perimeterUp.y, perimeterUp.elbowUp);
            }

            //=============================================================
            //========================INTAKE ACTIONS=======================
            //=============================================================

            //========================Servo control========================
            if (gamepad1.left_bumper) {
                intake.setPower(-1.0); // Pulls
            } else if (gamepad1.right_bumper) {
                intake.setPower(1.0); // Push
            } else {
                intake.setPower(0.0); // Steady
            }

            if (!sampleDetector.checkColor()) {
                Gamepads.wrongSampleTypeRumble(gamepad1); // If we are trying to get a wrong colored sample, the controller will vibrate.
            }

            //=============================================================
            //==========================TELEMETRY==========================
            //=============================================================

            TelemetryMethods.displayMotorPowers(telemetry,
                                                robotHardware.frontLeftMotor.getPower(),
                                                robotHardware.backLeftMotor.getPower(),
                                                robotHardware.frontRightMotor.getPower(),
                                                robotHardware.frontRightMotor.getPower()
            );
            TelemetryMethods.displayDriveModes(telemetry, driveModeGamepad1, driveModeGamepad2);
            TelemetryMethods.displayAlliance(telemetry, ColorConfig.alliance);
            TelemetryMethods.displaySampleValidation(telemetry, sampleDetector.checkColor());
            TelemetryMethods.displayCodeVersion(telemetry, "7.25.25.6.43");
            telemetry.addLine("-----------------------------");
            telemetry.update();
        }
    }
}
