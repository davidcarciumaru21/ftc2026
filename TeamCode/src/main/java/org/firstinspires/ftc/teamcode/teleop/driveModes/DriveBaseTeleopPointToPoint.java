package org.firstinspires.ftc.teamcode.teleop.driveModes;

import static org.firstinspires.ftc.teamcode.MecanumDrive.PARAMS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.Hardware;
import org.firstinspires.ftc.teamcode.teleop.Utils;
import org.firstinspires.ftc.teamcode.teleop.presets.pointToPoint.Node;
import org.firstinspires.ftc.teamcode.teleop.presets.pointToPoint.Table;
import org.firstinspires.ftc.teamcode.teleop.presets.pointToPoint.Robot;
import org.firstinspires.ftc.teamcode.teleop.presets.pointToPoint.PointToPoint;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;

import java.util.List;

@TeleOp(name = "DriveBase-TeleOpPointToPoint", group = "Dev-Teleops")
public class DriveBaseTeleopPointToPoint extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robotPointToPoint = new Robot((int) Utils.cmToInches(35.0), (int) Utils.cmToInches(43.0));
        double startX = robotPointToPoint.width / 2;
        double startY = robotPointToPoint.height / 2;
        boolean buzy = false;
        int startHeading = 0;
        Pose2d startPose = new Pose2d(startX, startY, Math.toRadians(startHeading));

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Table tableObj = new Table(40, 40, 1);
        tableObj.markUnableToReachNodes(robotPointToPoint);
        tableObj.drawBorder(robotPointToPoint);

        Node endNode = tableObj.table[8][7];
        ThreeDeadWheelLocalizer driveLocalizer = new ThreeDeadWheelLocalizer(hardwareMap, PARAMS.inPerTick, startPose);

        boolean lastCircle = false;

        // Create Hardware instance
        final Hardware robotHardware = new Hardware();

        // Initialize hardware map
        robotHardware.init(hardwareMap, 2);

        // Coefficients for joystick inputs
        double coefX = 1.1;
        double coefY = 1.0;
        double coefRx = 1.0;

        waitForStart();

        while (opModeIsActive()) {

            driveLocalizer.update();
            Pose2d pose = driveLocalizer.getPose();

            Vector2d poseCoordinates = new Vector2d(pose.position.x, pose.position.y);
            Vector2d NodeCoordinates = Utils.robotNode(poseCoordinates);

            int nodeX = (int) NodeCoordinates.x;
            int nodeY = 2 * (int) startY - (int) NodeCoordinates.y;

            telemetry.addData("xNode", nodeX);
            telemetry.addData("yNode", nodeY);
            telemetry.update();

            boolean currentCircle = gamepad1.circle;

            if (currentCircle && !lastCircle) {
                buzy = true;
                // Valid bounds check
                boolean nodeInBounds = nodeX >= 0 && nodeX < tableObj.columns && nodeY >= 0 && nodeY < tableObj.rows;

                if (!nodeInBounds) {
                    telemetry.addLine("Invalid node index for A*");
                    telemetry.addData("X", nodeX);
                    telemetry.addData("Y", nodeY);
                    telemetry.update();
                } else {
                    try {
                        Node startNode = tableObj.table[nodeY][nodeX];
                        startNode.isStart = true;
                        endNode.isEnd = true;

                        List<Node> path = PointToPoint.aStar(tableObj, startNode, endNode, robotPointToPoint);

                        if (path == null) {
                            telemetry.addLine("No path found.");
                        } else {
                            tableObj.applyPathToTable(path);
                            tableObj.generateRobot(robotPointToPoint, new int[]{startNode.y, startNode.x});

                            List<String> moves = PointToPoint.pathToCommands(path, tableObj);
                            List<String> compressedMoves = PointToPoint.compressPath(moves);

                            for (String command : compressedMoves) {
                                telemetry.addLine(command);
                            }
                            telemetry.addLine("Executing path...");
                            PointToPoint.executePath(compressedMoves, drive, pose);
                        }
                    } catch (Exception e) {
                        telemetry.addLine("Exception during A* pathing");
                        telemetry.addData("Message", e.getMessage());
                        e.printStackTrace();
                    }

                    telemetry.update();
                }
                buzy = false;
            }

            lastCircle = currentCircle;
            if(buzy == false) {
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
                robotHardware.frontLeftMotor.setPower(frontLeftPower);
                robotHardware.backLeftMotor.setPower(backLeftPower);
                robotHardware.frontRightMotor.setPower(frontRightPower);
                robotHardware.backRightMotor.setPower(backRightPower);
            }
        }
    }
}
