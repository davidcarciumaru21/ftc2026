package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.trajectories.roadRunnerConfigurations.MecanumDrive.PARAMS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utils.MeasurementUnit;
import org.firstinspires.ftc.teamcode.Utils.Telemetry;
import org.firstinspires.ftc.teamcode.presets.Node;
import org.firstinspires.ftc.teamcode.presets.Table;
import org.firstinspires.ftc.teamcode.presets.Robot;
import org.firstinspires.ftc.teamcode.presets.PointToPoint;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.systems.robotHardware.Hardware;
import org.firstinspires.ftc.teamcode.trajectories.roadRunnerConfigurations.MecanumDrive;
import org.firstinspires.ftc.teamcode.trajectories.roadRunnerConfigurations.ThreeDeadWheelLocalizer;

import java.util.List;

@TeleOp(name = "DriveBase-TeleOpPointToPoint", group = "Dev-Teleops")
public class DriveBaseTeleopPointToPoint extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Create a 40x40 grid map
        Table tableObj = new Table(40, 40, 1);

        // Define the robot with specific dimensions (converted from cm to inches)
        Robot robotPoinToPoint = new Robot((int) MeasurementUnit.cmToInches(35.0), (int) MeasurementUnit.cmToInches(43.0));

        // Set robot's starting pose
        double startX = robotPoinToPoint.height / 2, startY = robotPoinToPoint.width / 2;
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

        // Create and initialize hardware (motors, etc.)
        final Hardware robotHardware = new Hardware();
        robotHardware.init(hardwareMap, (byte) 2);

        // Control sensitivity coefficients
        double coefX = 1.1;
        double coefY = 1.0;
        double coefRx = 1.0;

        // Variables for path planning and control
        int xNode, yNode;
        Node currentNode;
        boolean buzy = false; // Flag to prevent manual drive during autonomous path-following

        // Set a target node on the grid
        Node endNode = tableObj.table[15][15];
        List<Node> path;

        // Joystick input variables
        double x, y, rx;
        double denominator = 0;
        double frontLeftPower, backLeftPower, frontRightPower, backRightPower;

        waitForStart();

        while(opModeIsActive()) {

            // Update position using localization
            driveLocalizer.update();
            Pose2d currentPose = driveLocalizer.getPose();

            // Convert current pose into grid coordinates (for pathfinding)
            yNode = 2 * (int) startX - (int) Math.round(currentPose.position.x);
            xNode = 2 * (int) startY - (int) Math.round(currentPose.position.y);
            currentNode = tableObj.table[yNode][xNode];

            // Read circle button press (to trigger path planning)
            currentBState = gamepad1.circle;

            // Detect rising edge of button press to start autonomous move
            if (currentBState && !lastBState) {
                buzy = true; // Disable manual control while robot is moving

                try {
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

            // Manual driving (if not currently executing a path)
            if (!buzy) {
                // Read gamepad joystick inputs
                x = -gamepad1.left_stick_x * coefX;  // Strafing
                y = gamepad1.left_stick_y * coefY;   // Forward/backward
                rx = -gamepad1.right_stick_x * coefRx; // Rotation

                // Normalize input to prevent overpowering motors
                denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

                // Calculate motor powers using standard mecanum drive formulas
                frontLeftPower = (y + x + rx) / denominator;
                backLeftPower = (y - x + rx) / denominator;
                frontRightPower = (y - x - rx) / denominator;
                backRightPower = (y + x - rx) / denominator;

                // Apply power to motors
                robotHardware.frontLeftMotor.setPower(frontLeftPower);
                robotHardware.backLeftMotor.setPower(backLeftPower);
                robotHardware.frontRightMotor.setPower(frontRightPower);
                robotHardware.backRightMotor.setPower(backRightPower);
            }

            // Debug/telemetry output
            telemetry.addData("x", currentPose.position.x);
            telemetry.addData("y", currentPose.position.y);
            telemetry.addData("NodeX", xNode);
            telemetry.addData("Nodey", yNode);
            telemetry.update();

            // Update button state tracker
            lastBState = currentBState;
        }
    }
}
