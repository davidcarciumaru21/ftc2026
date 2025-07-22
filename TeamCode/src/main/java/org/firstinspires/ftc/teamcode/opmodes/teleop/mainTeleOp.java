package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.trajectories.roadRunnerConfigurations.MecanumDrive.PARAMS;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Utils.MeasurementUnit;
import org.firstinspires.ftc.teamcode.Utils.Telemetry;
import org.firstinspires.ftc.teamcode.presets.Node;
import org.firstinspires.ftc.teamcode.presets.Table;
import org.firstinspires.ftc.teamcode.presets.Robot;
import org.firstinspires.ftc.teamcode.presets.PointToPoint;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.systems.arm.JacobianArm;
import org.firstinspires.ftc.teamcode.systems.arm.Positions;
import org.firstinspires.ftc.teamcode.systems.gamepad.Gamepads;
import org.firstinspires.ftc.teamcode.systems.robotHardware.Hardware;
import org.firstinspires.ftc.teamcode.trajectories.roadRunnerConfigurations.MecanumDrive;
import org.firstinspires.ftc.teamcode.trajectories.roadRunnerConfigurations.ThreeDeadWheelLocalizer;

import java.util.List;

@TeleOp(name = "Main-TeleOp", group = "Use")
public class mainTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Create a 40x40 grid map
        Table tableObj = new Table(144, 144, 1);

        // Define the robot with specific dimensions (converted from cm to inches)
        Robot robotPoinToPoint = new Robot((int) MeasurementUnit.cmToInches(43.0), (int) MeasurementUnit.cmToInches(45.0));

        // Set robot's starting pose
        double startX = tableObj.columns - robotPoinToPoint.height / 2.0, startY = tableObj.rows - robotPoinToPoint.width / 2.0;
        double startHeading = Math.toDegrees(0);  // Facing forward
        Pose2d startPose = new Pose2d(startX, startY, startHeading);

        // Set up Road Runner drive and localization
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        ThreeDeadWheelLocalizer driveLocalizer = new ThreeDeadWheelLocalizer(hardwareMap, PARAMS.inPerTick, startPose);

        // Configure table (obstacles, boundaries, etc.)
        tableObj.drawBorder(robotPoinToPoint);
        tableObj.markUnableToReachNodes(robotPoinToPoint);

        // Gamepad button state tracking
        boolean lastBState = false;
        boolean currentBState;
        boolean lastCircle = false;

        // SHARE button state tracking for toggling drive modes
        boolean currentShareStateGamepad1;
        boolean lastShareGamepad1 = false;
        boolean currentShareStateGamepad2;
        boolean lastShareGamepad2 = false;

        // Drive mode tracking: 1 = normal, 2 = field-centric
        byte driveModeGamepad1 = 1, driveModeGamepad2 = 1;

        // Create and initialize hardware (motors, etc.)
        final Hardware robotHardware = new Hardware();
        robotHardware.init(hardwareMap, (byte) 2);

        // Variables for path planning and control
        int xNode, yNode;
        Node currentNode;
        boolean buzy = false; // Flag to prevent manual drive during autonomous path-following

        // Set a target node on the grid
        Node endNode = tableObj.table[15][15];
        List<Node> path;

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

        // Initialize the JacobianArm system using the hardware map
        JacobianArm arm = new JacobianArm(hardwareMap);

        // Get reference to CRServo used for intake mechanism
        CRServo intake = hardwareMap.get(CRServo.class, "intake");

        waitForStart();

        // Load predefined arm positions from Positions class
        Positions.ArmPosition basketPos = Positions.getBasket();
        Positions.ArmPosition perimeterPos = Positions.getPerimeter();
        Positions.ArmPosition specimenUpPos = Positions.getSpecimenUp();
        Positions.ArmPosition specimenDownPos = Positions.getSpecimenDown();
        Positions.ArmPosition submersibil1 = Positions.getSubmersibil1();
        Positions.ArmPosition submersibil2 = Positions.getSubmersibil2();
        Positions.ArmPosition basket3 = Positions.getBasket3();
        Positions.ArmPosition perimeterUp = Positions.getPerimeterUP();

        while(opModeIsActive()) {

            // Update position using localization
            driveLocalizer.update();
            Pose2d currentPose = driveLocalizer.getPose();

            // Read circle button press (to trigger path planning)
            currentBState = gamepad1.circle;

            /*
            // Detect rising edge of button press to start autonomous move
            if (currentBState && !lastBState) {
                buzy = true; // Disable manual control while robot is moving

                try {
                    // Convert current pose into grid coordinates (for pathfinding)
                    yNode = 2 * (int) startX - (int) Math.round(currentPose.position.x);
                    xNode = (int) Math.round(currentPose.position.y);
                    currentNode = tableObj.table[yNode][xNode];

                    // Mark start and end for pathfinding
                    currentNode.isStart = true;
                    endNode.isEnd = true;

                    // Reset and update the table map
                    tableObj.resetNodeStates();
                    tableObj.drawBorder(robotPoinToPoint);
                    tableObj.markUnableToReachNodes(robotPoinToPoint);
                    tableObj.generateRobot(robotPoinToPoint, new int[] {yNode, xNode});

                    // Run A* pathfinding
                    path = PointToPoint.aStar(tableObj, currentNode, endNode, robotPoinToPoint);
                    tableObj.applyPathToTable(path);

                    // Convert path to movement commands
                    List<String> moves = PointToPoint.pathToCommands(path, tableObj);
                    List<String> compressedMoves = PointToPoint.compressPath(moves);

                    // Follow the path using Road Runner drive
                    PointToPoint.executePath(compressedMoves, drive, currentPose);

                } catch(Exception e) {
                    // If path planning fails
                    telemetry.addLine("Unable to get to the point");
                    telemetry.update();
                }

                buzy = false; // Re-enable manual control
            }
            */

            // Manual driving (if not currently executing a path)
            if (!buzy) {
                // === Update speed coefficients based on Gamepad1 bumper input ===
                coefXGamepad1 = 1.0;
                coefYGamepad1 = 1.0;
                coefRxGamepad1 = 1.0;

                if (gamepad1.right_trigger > 0.1) {
                    // 50% speed mode
                    coefXGamepad1 = 0.5;
                    coefYGamepad1 = 0.5;
                    coefRxGamepad1 = 0.5;
                } else if (gamepad1.left_trigger > 0.1) {
                    // 25% precision mode
                    coefXGamepad1 = 0.25;
                    coefYGamepad1 = 0.25;
                    coefRxGamepad1 = 0.25;
                }

                // === Same logic for Gamepad2 bumpers ===
                coefXGamepad2 = 1.0;
                coefYGamepad2 = 1.0;
                coefRxGamepad2 = 1.0;

                if (gamepad2.right_trigger > 0.1) {
                    coefXGamepad2 = 0.5;
                    coefYGamepad2 = 0.5;
                    coefRxGamepad2 = 0.5;
                } else if (gamepad2.left_trigger > 0.1) {
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
                    x = gamepad2.left_stick_x * coefXGamepad2;
                    y = -gamepad2.left_stick_y * coefYGamepad2;
                    rx = gamepad2.right_stick_x * coefRxGamepad2;

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
                    x = gamepad1.left_stick_x * coefXGamepad1;
                    y = -gamepad1.left_stick_y * coefYGamepad1;
                    rx = gamepad1.right_stick_x * coefRxGamepad1;

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

                // GAMEPAD 1: Control arm positions with D-Pad
                if (gamepad1.dpad_up) {
                    arm.ArmGoto(basketPos.x, basketPos.y, basketPos.elbowUp);  // Go to basket position
                } else if (gamepad1.dpad_down) {
                    arm.ArmGoto(submersibil1.x, submersibil1.y, submersibil1.elbowUp); // Submersible 1
                } else if (gamepad1.dpad_right) {
                    arm.ArmGoto(submersibil2.x, submersibil2.y, submersibil2.elbowUp); // Submersible 2
                } else if (gamepad1.dpad_left) {
                    arm.ArmGoto(basket3.x, basket3.y, basket3.elbowUp); // Alternate basket position
                }

                // GAMEPAD 2: Secondary operator arm controls
                if (gamepad2.dpad_down) {
                    arm.ArmGoto(perimeterPos.x, perimeterPos.y, perimeterPos.elbowUp); // Perimeter low
                } else if (gamepad2.dpad_right) {
                    arm.ArmGoto(specimenUpPos.x, specimenUpPos.y, specimenUpPos.elbowUp); // Specimen up
                } else if (gamepad2.dpad_left) {
                    arm.ArmGoto(specimenDownPos.x, specimenDownPos.y, specimenDownPos.elbowUp); // Specimen down
                } else if (gamepad2.dpad_up) {
                    arm.ArmGoto(perimeterUp.x, perimeterUp.y, perimeterUp.elbowUp); // Perimeter high
                }

                // GAMEPAD 1 bumpers: Intake control
                if (gamepad1.left_bumper) {
                    intake.setPower(1.0); // Intake in
                } else if (gamepad1.right_bumper) {
                    intake.setPower(-1.0); // Intake out
                } else {
                    intake.setPower(0.0); // Stop intake
                }
            }

            // Update button state tracker
            lastBState = currentBState;
        }
    }
}
