package org.firstinspires.ftc.teamcode.teleop.driveModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.Utils;
import org.firstinspires.ftc.teamcode.teleop.presets.pointToPoint.Node;
import org.firstinspires.ftc.teamcode.teleop.presets.pointToPoint.Table;
import org.firstinspires.ftc.teamcode.teleop.presets.pointToPoint.Robot;
import org.firstinspires.ftc.teamcode.teleop.presets.pointToPoint.PointToPoint;
import org.firstinspires.ftc.teamcode.teleop.Hardware;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.List;

@TeleOp(name = "DriveBase-TeleOpPointToPoint", group = "Dev-Teleops")
public class DriveBaseTeleopPointToPoint extends LinearOpMode {

    private final Hardware robot = new Hardware();

    // Robot size in grid units
    private final Robot robotPreset = new Robot(35, 43);

    private List<Node> path;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing hardware...");
        telemetry.update();

        robot.init(hardwareMap, 2);

        Pose2d startPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        MecanumDrive.DriveLocalizer driveLocalizer = drive.new DriveLocalizer(startPose);

        Table tableObj = new Table(280, 140, 10);

        telemetry.addLine("Precomputing table data...");
        telemetry.update();
        tableObj.mark_unable_to_reach_nodes(robotPreset);
        tableObj.draw_border(robotPreset);

        telemetry.addLine("Ready - Press Play");
        telemetry.update();
        waitForStart();

        double coefX = 1.1, coefY = 1.0, coefRx = 1.0;

        while (opModeIsActive()) {

            // FIRST: update pose estimate
            driveLocalizer.update();

            // THEN: get pose after update
            Vector2d robotCoordinates = driveLocalizer.getPose().position;
            double headingDegrees = Math.toDegrees(drive.localizer.getPose().heading.toDouble());

            // Get joystick inputs
            double x = gamepad1.left_stick_x * coefX;
            double y = gamepad1.left_stick_y * coefY;
            double rx = -gamepad1.right_stick_x * coefRx;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            robot.frontLeftMotor.setPower(frontLeftPower);
            robot.backLeftMotor.setPower(backLeftPower);
            robot.frontRightMotor.setPower(frontRightPower);
            robot.backRightMotor.setPower(backRightPower);

            if (gamepad1.square) {
                telemetry.addLine("Square pressed. Starting pathfinding...");
                telemetry.update();

                int startX = (int) (robotCoordinates.x + Math.ceil(robotPreset.width / 2.0));
                int startY = (int) (robotCoordinates.y + Math.ceil(robotPreset.height / 2.0));
                Node start = new Node(startX, startY);
                start.isStart = true;

                Node end = tableObj.table[100][100];
                end.isEnd = true;

                telemetry.addLine("Running A*...");
                telemetry.update();
                path = PointToPoint.aStar(tableObj, start, end, robotPreset);

                if (path != null && !path.isEmpty()) {
                    telemetry.addLine("Path found. Executing...");
                    telemetry.update();

                    tableObj.apply_path_to_table(path);
                    tableObj.generate_robot(robotPreset, new int[]{start.y, start.x});
                    tableObj.print_matrix();

                    List<String> moves = PointToPoint.path_to_commands(path, tableObj);
                    List<String> compressedMoves = PointToPoint.compress_path(moves);
                    PointToPoint.execute_path(compressedMoves, drive, driveLocalizer.getPose());

                    telemetry.addLine("Path executed.");
                } else {
                    telemetry.addLine("No path found.");
                }
                telemetry.update();
            }

            // Telemetry
            Utils.displayMotorPowers(telemetry, frontLeftPower, backLeftPower, frontRightPower, backRightPower);
            Utils.displayCodeVersion(telemetry, "7.8.25.09.50");
            Utils.displayCoordonates(telemetry, robotCoordinates.x, robotCoordinates.y, headingDegrees);
            telemetry.update();
        }
    }
}
