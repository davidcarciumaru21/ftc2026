package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.trajectories.MecanumDrive.PARAMS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utils.Utils;
import org.firstinspires.ftc.teamcode.presets.Node;
import org.firstinspires.ftc.teamcode.presets.Table;
import org.firstinspires.ftc.teamcode.presets.Robot;
import org.firstinspires.ftc.teamcode.presets.PointToPoint;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.robotHardware.Hardware;
import org.firstinspires.ftc.teamcode.trajectories.MecanumDrive;
import org.firstinspires.ftc.teamcode.trajectories.ThreeDeadWheelLocalizer;

import java.util.List;

@TeleOp(name = "DriveBase-TeleOpPointToPoint", group = "Dev-Teleops")
public class DriveBaseTeleopPointToPoint extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robotPointToPoint = new Robot((int) Utils.cmToInches(35.0), (int) Utils.cmToInches(43.0));
        double startX = robotPointToPoint.width / 2;
        double startY = robotPointToPoint.height / 2;

        Pose2d startPose = new Pose2d(startX, startY, 0);

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Table tableObj = new Table(40, 40, 1);
        tableObj.markUnableToReachNodes(robotPointToPoint);
        tableObj.drawBorder(robotPointToPoint);

        Node endNode = tableObj.table[8][7];
        ThreeDeadWheelLocalizer driveLocalizer = new ThreeDeadWheelLocalizer(hardwareMap, PARAMS.inPerTick, startPose);

        boolean lastCircle = false;
        boolean busy = false;

        Hardware robotHardware = new Hardware();
        robotHardware.init(hardwareMap, 2);

        double coefX = 1.1;
        double coefY = 1.0;
        double coefRx = 1.0;

        waitForStart();

        while (opModeIsActive()) {

            driveLocalizer.update();
            Pose2d currentPose = driveLocalizer.getPose();

            Vector2d poseCoordinates = new Vector2d(currentPose.position.x, currentPose.position.y);
            Vector2d nodeCoordinates = Utils.robotNode(poseCoordinates);

            int nodeX = (int) nodeCoordinates.x;
            int nodeY = 2 * (int) startY - (int)Math.round(nodeCoordinates.y);

            telemetry.addData("Robot Pose X", currentPose.position.x);
            telemetry.addData("Robot Pose Y", currentPose.position.y);
            telemetry.addData("Robot Heading (rad)", Math.toDegrees(currentPose.heading.toDouble()));

            telemetry.addData("Mapped Node X", nodeX);
            telemetry.addData("Mapped Node Y", nodeY);

            boolean currentCircle = gamepad1.circle;

            telemetry.addData("Circle Button Pressed", currentCircle);
            telemetry.addData("Busy Flag", busy);

            if (currentCircle && !lastCircle && !busy) {
                busy = true;

                telemetry.addLine("Starting pathfinding sequence...");
                telemetry.update();

                // Reset node states for fresh pathfinding
                tableObj.resetNodeStates();
                telemetry.addLine("Node states reset.");

                // Validate node bounds
                boolean nodeInBounds = nodeX >= 0 && nodeX < tableObj.columns && nodeY >= 0 && nodeY < tableObj.rows;

                telemetry.addData("Node in Bounds?", nodeInBounds);

                if (!nodeInBounds) {
                    telemetry.addLine("Invalid node index for A*");
                    telemetry.addData("X", nodeX);
                    telemetry.addData("Y", nodeY);
                    telemetry.update();
                    busy = false;
                    lastCircle = currentCircle;
                    continue;
                }

                // Update robot position on the grid before pathfinding
                tableObj.generateRobot(robotPointToPoint, new int[]{nodeY, nodeX});
                telemetry.addLine("Robot position updated on grid.");

                try {
                    Node startNode = tableObj.table[nodeY][nodeX];
                    Node goalNode = tableObj.table[8][7];

                    startNode.isStart = true;
                    goalNode.isEnd = true;

                    telemetry.addData("Start Node", "[" + startNode.y + "," + startNode.x + "]");
                    telemetry.addData("Goal Node", "[" + goalNode.y + "," + goalNode.x + "]");

                    List<Node> path = PointToPoint.aStar(tableObj, startNode, goalNode, robotPointToPoint);

                    if (path == null) {
                        telemetry.addLine("No path found.");
                    } else {
                        telemetry.addLine("Path found with length: " + path.size());
                        tableObj.applyPathToTable(path);
                        tableObj.generateRobot(robotPointToPoint, new int[]{nodeY, nodeX});
                        tableObj.printMatrix();

                        List<String> moves = PointToPoint.pathToCommands(path, tableObj);
                        List<String> compressedMoves = PointToPoint.compressPath(moves);

                        telemetry.addLine("Commands to execute:");
                        for (String command : compressedMoves) {
                            telemetry.addLine(" - " + command);
                        }

                        telemetry.addLine("Executing path...");
                        telemetry.update();

                        // Update pose again just before execution for accuracy
                        driveLocalizer.update();
                        Pose2d updatedPose = driveLocalizer.getPose();

                        telemetry.addData("Pose before execution", updatedPose);

                        PointToPoint.executePath(compressedMoves, drive, updatedPose);

                        telemetry.addLine("Path execution complete.");
                    }
                } catch (Exception e) {
                    telemetry.addLine("Exception during A* pathing:");
                    telemetry.addData("Message", e.getMessage());
                    telemetry.update();
                    e.printStackTrace();
                }

                busy = false;
            }

            lastCircle = currentCircle;

            if (!busy) {
                // Manual driving controls
                double x = gamepad1.left_stick_x * coefX;
                double y = gamepad1.left_stick_y * coefY;
                double rx = -gamepad1.right_stick_x * coefRx;

                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

                double frontLeftPower = (y + x + rx) / denominator;
                double backLeftPower = (y - x + rx) / denominator;
                double frontRightPower = (y - x - rx) / denominator;
                double backRightPower = (y + x - rx) / denominator;

                robotHardware.frontLeftMotor.setPower(frontLeftPower);
                robotHardware.backLeftMotor.setPower(backLeftPower);
                robotHardware.frontRightMotor.setPower(frontRightPower);
                robotHardware.backRightMotor.setPower(backRightPower);
            }

            telemetry.update();
        }
    }
}
