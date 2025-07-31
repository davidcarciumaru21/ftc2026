package org.firstinspires.ftc.teamcode.opmodes.teleop;

//==============================Robot Core=============================
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

//==============================Road Runner============================
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.teamcode.config.enums.AllianceColor;
import org.firstinspires.ftc.teamcode.roadRunner.drives.MecanumDrive;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;

//=============================Robot Systems===========================
import org.firstinspires.ftc.teamcode.config.ColorConfig;
import org.firstinspires.ftc.teamcode.systems.arm.JacobianArm;
import org.firstinspires.ftc.teamcode.systems.arm.Positions;
import org.firstinspires.ftc.teamcode.systems.arm.ArmAction;
import org.firstinspires.ftc.teamcode.systems.colorSensor.SampleDetection;
import org.firstinspires.ftc.teamcode.systems.servo.ServoAction;

//=================================Utils===============================
import org.firstinspires.ftc.teamcode.Utils.Gamepads;
import org.firstinspires.ftc.teamcode.Utils.TelemetryMethods;
import org.firstinspires.ftc.teamcode.Utils.WaitAction;

//=============================Configurations==========================
import org.firstinspires.ftc.teamcode.config.enums.DriveType;
import org.firstinspires.ftc.teamcode.config.PresetsPositions;
import org.firstinspires.ftc.teamcode.config.GamepadsCoefficients;

//=============================File reading============================
import java.io.FileReader;
import java.io.IOException;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import java.io.File;

@TeleOp(name = "MainTeleOp", group = "A(Match usage)")
public class MainTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {

        //=============================================================
        //===================VARIABLE INITIALIZATION===================
        //=============================================================

        //===================Gamepad1 button states====================
        boolean currentShareStateGamepad1;
        boolean lastShareGamepad1 = false;

        boolean currentTriangleStateGamepad1;
        boolean lastTriangleStateGamepad1 = false;

        boolean currentCircleStateGamepad1;
        boolean lastCircleStateGamepad1 = false;

        boolean currentCrossStateGamepad1;
        boolean lastCrossStateGamepad1 = false;

        boolean currentSquareStateGamepad1;
        boolean lastSquareStateGamepad1 = false;

        //===================Gamepad2 button states====================
        boolean currentShareStateGamepad2;
        boolean lastShareGamepad2 = false;

        boolean currentTriangleStateGamepad2;
        boolean lastTriangleStateGamepad2 = false;

        boolean currentCircleStateGamepad2;
        boolean lastCircleStateGamepad2 = false;

        boolean currentCrossStateGamepad2;
        boolean lastCrossStateGamepad2 = false;

        boolean currentSquareStateGamepad2;
        boolean lastSquareStateGamepad2 = false;

        boolean currentPsStateGamepad2;
        boolean lastPsStateGamepad2 = false;

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

        //=============================================================
        //==================ROAD RUNNER INITIALIZATION=================
        //=============================================================

        //====================Setting the position=====================
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

        MecanumDrive.PARAMS.maxWheelVel = 500;
        MecanumDrive.PARAMS.maxProfileAccel = 250;
        MecanumDrive.PARAMS.minProfileAccel = -250;
        MecanumDrive.PARAMS.maxAngVel = 500;
        MecanumDrive.PARAMS.maxAngAccel = 250;
        MecanumDrive speedDrive = new MecanumDrive(hardwareMap, startPose);

        //=====================IMU initialization======================
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN));
        imu.initialize(parameters);

        //================Arm and intake initialization=================
        JacobianArm arm = new JacobianArm(hardwareMap);

        double servoPower;
        CRServo intake = hardwareMap.get(CRServo.class, "intake");
        SampleDetection sampleDetector = new SampleDetection(hardwareMap);

        // Wait for the start button
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

        //=======================Drive positions========================
        Pose2d blueSubmersiblePose = PresetsPositions.getBlueSubmersiblePose();
        Pose2d forwardBlueSubmersiblePose = PresetsPositions.getForwardBlueSubmersiblePose();

        Pose2d redSubmersiblePose = PresetsPositions.getRedSubmersiblePose();
        Pose2d forwardRedSubmersiblePose = PresetsPositions.getForwardRedSubmersiblePose();

        Pose2d blueBasePose = PresetsPositions.getBlueBasePose();
        Pose2d redBasePose = PresetsPositions.getRedBasePose();

        Pose2d blueScoringSubmersiblePose = PresetsPositions.getBlueScoringSubmersiblePose();
        Pose2d redScoringSubmersiblePose = PresetsPositions.getRedScoringSubmersiblePose();

        //===========================Actions============================
        ArmAction armAction = new ArmAction(arm);
        Action basket = armAction.ArmGoto(basketPos.x, basketPos.y, basketPos.elbowUp); // Move arm to scoring position
        ServoAction servo = new ServoAction(hardwareMap);
        Action servoOut = servo.setPower(-1.0);  // Activate servo to take game element
        Action servoStop = servo.setPower(0.0); // Stop the servo after ejection
        Action servoIn = servo.setPower(1.0); // Activate servo to eject game element

        // Main control loop
        while(opModeIsActive()) {

            //=============================================================
            //=======================BUTTON STATES=========================
            //=============================================================

            //===================Gamepad1 button states====================
            currentShareStateGamepad1 = gamepad1.share;
            currentTriangleStateGamepad1 = gamepad1.triangle;
            currentCircleStateGamepad1 = gamepad1.circle;

            //===================Gamepad2 button states====================
            currentShareStateGamepad2 = gamepad2.share;
            currentTriangleStateGamepad2 = gamepad2.triangle;
            currentCircleStateGamepad2 = gamepad2.circle;
            currentCrossStateGamepad2 = gamepad2.cross;
            currentSquareStateGamepad2 = gamepad2.square;

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
            //========================ARM MOVEMENT=========================
            //=============================================================

            //=====================Arm positions gamepad1==================

            if (gamepad1.dpad_up) {
                arm.ArmGoto(basketPos.x, basketPos.y, basketPos.elbowUp);
            } else if (gamepad1.dpad_down) {
                arm.ArmGoto(submersibil1.x, submersibil1.y, submersibil1.elbowUp);
            } else if (gamepad1.dpad_right) {
                arm.ArmGoto(submersibil2.x, submersibil2.y, submersibil2.elbowUp);
            } else if (gamepad1.dpad_left) {
                arm.ArmGoto(basket3.x, basket3.y, basket3.elbowUp);
            }

            //=====================Arm positions gamepad2==================
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

            servoPower = 0.0;

            if (gamepad1.left_bumper) {
                servoPower = -1.0;
            } else if (gamepad1.right_bumper) {
                servoPower = 1.0;
            }
            if (gamepad2.left_bumper) {
                servoPower = -1.0;
            } else if (gamepad2.right_bumper) {
                servoPower = 1.0;
            }

            intake.setPower(servoPower);

            if (!sampleDetector.checkColor()) {
                Gamepads.wrongSampleTypeRumble(gamepad1); // If we are trying to get a wrong colored sample, the controller will vibrate.
            }

            //=============================================================
            //=====================ROAD RUNNER PRESETS=====================
            //=============================================================

            //=====================Presets for gamepad1====================

            if (currentCircleStateGamepad1 && !lastCircleStateGamepad1) {
                Actions.runBlocking(new SequentialAction(
                        basket,
                        servoOut,
                        new WaitAction(0.3),
                        servoStop
                ));
            }

            //=====================Presets for gamepad2====================

            //===================From base to submersible==================
            if (currentTriangleStateGamepad2 && !lastTriangleStateGamepad2) {

                Action strafeToSubmersible;
                Action forwardToSubmersible;
                if (ColorConfig.alliance == AllianceColor.BLUE) {

                    speedDrive.stopDrive();
                    speedDrive.localizer.update();
                    currentPose = speedDrive.localizer.getPose();
                    speedDrive.localizer.setPose(currentPose);
                    strafeToSubmersible = speedDrive.actionBuilder(currentPose)
                            .strafeToLinearHeading(new Vector2d(
                                            blueSubmersiblePose.position.x,
                                            blueSubmersiblePose.position.y),
                                    blueSubmersiblePose.heading.toDouble())
                            .build();

                    speedDrive.stopDrive();
                    Actions.runBlocking(strafeToSubmersible);

                    speedDrive.stopDrive();
                    speedDrive.localizer.update();
                    currentPose = speedDrive.localizer.getPose();
                    speedDrive.localizer.setPose(currentPose);
                    forwardToSubmersible = speedDrive.actionBuilder(currentPose)
                            .strafeToLinearHeading(new Vector2d(
                                            forwardBlueSubmersiblePose.position.x,
                                            forwardBlueSubmersiblePose.position.y),
                                    forwardBlueSubmersiblePose.heading.toDouble())
                            .build();

                    speedDrive.stopDrive();
                    Actions.runBlocking(forwardToSubmersible);
                    speedDrive.stopDrive();
                } else {

                    speedDrive.stopDrive();
                    speedDrive.localizer.update();
                    currentPose = speedDrive.localizer.getPose();
                    speedDrive.localizer.setPose(currentPose);
                    strafeToSubmersible = speedDrive.actionBuilder(currentPose)
                            .strafeToLinearHeading(new Vector2d(
                                            redSubmersiblePose.position.x,
                                            redSubmersiblePose.position.y),
                                    redSubmersiblePose.heading.toDouble())
                            .build();

                    speedDrive.stopDrive();
                    Actions.runBlocking(strafeToSubmersible);

                    speedDrive.stopDrive();
                    speedDrive.localizer.update();
                    currentPose = speedDrive.localizer.getPose();
                    speedDrive.localizer.setPose(currentPose);
                    forwardToSubmersible = speedDrive.actionBuilder(currentPose)
                            .strafeToLinearHeading(new Vector2d(
                                                forwardRedSubmersiblePose.position.x,
                                                forwardRedSubmersiblePose.position.y),
                                    forwardRedSubmersiblePose.heading.toDouble())
                            .build();

                    speedDrive.stopDrive();
                    Actions.runBlocking(forwardToSubmersible);
                    speedDrive.stopDrive();

                }
            }

            //===================From submersible to base==================
            if (currentCircleStateGamepad2 && !lastCircleStateGamepad2) {

                Action backFromSubmersible;
                Action strafeToBase;
                if (ColorConfig.alliance == AllianceColor.BLUE) {

                    speedDrive.stopDrive();
                    speedDrive.localizer.update();
                    currentPose = speedDrive.localizer.getPose();
                    speedDrive.localizer.setPose(currentPose);
                    backFromSubmersible = speedDrive.actionBuilder(currentPose)
                            .strafeToLinearHeading(new Vector2d(
                                            blueSubmersiblePose.position.x,
                                            blueSubmersiblePose.position.y),
                                    blueSubmersiblePose.heading.toDouble())
                            .build();

                    speedDrive.stopDrive();
                    Actions.runBlocking(backFromSubmersible);

                    speedDrive.stopDrive();
                    speedDrive.localizer.update();
                    currentPose = speedDrive.localizer.getPose();
                    speedDrive.localizer.setPose(currentPose);
                    strafeToBase = speedDrive.actionBuilder(currentPose)
                            .strafeToLinearHeading(new Vector2d(
                                            blueBasePose.position.x,
                                            blueBasePose.position.y),
                                    blueBasePose.heading.toDouble())
                            .build();

                    speedDrive.stopDrive();
                    Actions.runBlocking(strafeToBase);
                    speedDrive.stopDrive();
                } else {

                    speedDrive.stopDrive();
                    speedDrive.localizer.update();
                    currentPose = speedDrive.localizer.getPose();
                    speedDrive.localizer.setPose(currentPose);
                    backFromSubmersible = speedDrive.actionBuilder(currentPose)
                            .strafeToLinearHeading(new Vector2d(
                                                redSubmersiblePose.position.x,
                                                redSubmersiblePose.position.y),
                                    redSubmersiblePose.heading.toDouble())
                            .build();

                    speedDrive.stopDrive();
                    Actions.runBlocking(backFromSubmersible);

                    speedDrive.stopDrive();
                    speedDrive.localizer.update();
                    currentPose = speedDrive.localizer.getPose();
                    speedDrive.localizer.setPose(currentPose);
                    strafeToBase = speedDrive.actionBuilder(currentPose)
                            .strafeToLinearHeading(new Vector2d(
                                            redBasePose.position.x,
                                            redBasePose.position.y),
                                    redBasePose.heading.toDouble())
                            .build();

                    speedDrive.stopDrive();
                    Actions.runBlocking(strafeToBase);
                    speedDrive.stopDrive();
                }
            }

            //===============From base to submersible scoring==============
            if (currentCrossStateGamepad2 && !lastCrossStateGamepad2) {

                Action strafeFromBaseToScoringSubmersible;
                if (ColorConfig.alliance == AllianceColor.BLUE) {

                    speedDrive.stopDrive();
                    speedDrive.localizer.update();
                    currentPose = speedDrive.localizer.getPose();
                    speedDrive.localizer.setPose(currentPose);
                    strafeFromBaseToScoringSubmersible = speedDrive.actionBuilder(currentPose)
                            .strafeToLinearHeading(new Vector2d(
                                            blueScoringSubmersiblePose.position.x,
                                            blueScoringSubmersiblePose.position.y),
                                    blueScoringSubmersiblePose.heading.toDouble())
                            .build();

                    speedDrive.stopDrive();
                    Actions.runBlocking(strafeFromBaseToScoringSubmersible);
                    speedDrive.stopDrive();
                } else {

                    speedDrive.stopDrive();
                    speedDrive.localizer.update();
                    currentPose = speedDrive.localizer.getPose();
                    speedDrive.localizer.setPose(currentPose);
                    strafeFromBaseToScoringSubmersible = speedDrive.actionBuilder(currentPose)
                            .strafeToLinearHeading(new Vector2d(
                                            redScoringSubmersiblePose.position.x,
                                            redScoringSubmersiblePose.position.y),
                                    redScoringSubmersiblePose.heading.toDouble())
                            .build();

                    speedDrive.stopDrive();
                    Actions.runBlocking(strafeFromBaseToScoringSubmersible);
                    speedDrive.stopDrive();
                }
            }

            //===============From submersible scoring to base==============
            if (currentSquareStateGamepad2 && !lastSquareStateGamepad2) {

                Action strafeFromScoringSubmersibleToBase;
                if (ColorConfig.alliance == AllianceColor.BLUE) {

                    speedDrive.stopDrive();
                    speedDrive.localizer.update();
                    currentPose = speedDrive.localizer.getPose();
                    speedDrive.localizer.setPose(currentPose);
                    strafeFromScoringSubmersibleToBase = speedDrive.actionBuilder(currentPose)
                            .strafeToLinearHeading(new Vector2d(
                                            blueBasePose.position.x,
                                            blueBasePose.position.y),
                                    blueBasePose.heading.toDouble())
                            .build();

                    speedDrive.stopDrive();
                    Actions.runBlocking(strafeFromScoringSubmersibleToBase);
                    speedDrive.stopDrive();
                } else {

                    speedDrive.stopDrive();
                    speedDrive.localizer.update();
                    currentPose = speedDrive.localizer.getPose();
                    speedDrive.localizer.setPose(currentPose);
                    strafeFromScoringSubmersibleToBase = speedDrive.actionBuilder(currentPose)
                            .strafeToLinearHeading(new Vector2d(
                                            redBasePose.position.x,
                                            redBasePose.position.y),
                                    redBasePose.heading.toDouble())
                            .build();

                    speedDrive.stopDrive();
                    Actions.runBlocking(strafeFromScoringSubmersibleToBase);
                    speedDrive.stopDrive();
                }
            }

            lastTriangleStateGamepad2 = currentTriangleStateGamepad2;
            lastCircleStateGamepad2 = currentCircleStateGamepad2;
            lastCrossStateGamepad2 = currentCrossStateGamepad2;
            lastSquareStateGamepad2 = currentSquareStateGamepad2;

            lastCircleStateGamepad1 = currentCircleStateGamepad1;

            //=============================================================
            //==========================TELEMETRY==========================
            //=============================================================

            drive.localizer.update();
            currentPose = drive.localizer.getPose();

            TelemetryMethods.displayMotorPowers(telemetry, leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
            TelemetryMethods.displayPostion(telemetry, currentPose);
            TelemetryMethods.displayDriveModes(telemetry, driveModeGamepad1, driveModeGamepad2);
            TelemetryMethods.displayAlliance(telemetry, ColorConfig.alliance);
            TelemetryMethods.displaySampleValidation(telemetry, sampleDetector.checkColor());
            TelemetryMethods.displayCodeVersion(telemetry, "7.29.25.2.33");
            telemetry.addLine("-----------------------------");
            telemetry.update();
        }
    }
}
