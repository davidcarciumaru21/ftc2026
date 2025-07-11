package org.firstinspires.ftc.teamcode.teleop.driveModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.Utils;
import org.firstinspires.ftc.teamcode.teleop.presets.pointToPoint.Node;
import org.firstinspires.ftc.teamcode.teleop.presets.pointToPoint.Table;
import org.firstinspires.ftc.teamcode.teleop.presets.pointToPoint.Robot;
import org.firstinspires.ftc.teamcode.teleop.presets.pointToPoint.PointToPoint;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.List;

@TeleOp(name = "DriveBase-TeleOpPointToPoint", group = "Dev-Teleops")
public class DriveBaseTeleopPointToPoint extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robotPointToPoint = new Robot(35, 43);
        int startX = (int)(Utils.cmToInches((double) robotPointToPoint.width / 2) + 1), startY = (int)(Utils.cmToInches((double) robotPointToPoint.height) / 2 + 1);
        int startHeading = 0; // in degrees
        Pose2d startPose = new Pose2d(startX, startY, Math.toRadians(startHeading));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        MecanumDrive.DriveLocalizer driveLocalizer = drive.new DriveLocalizer(startPose);
        Table tableObj = new Table(100, 100, 1);
        tableObj.markUnableToReachNodes(robotPointToPoint);
        tableObj.drawBorder(robotPointToPoint);
        Node endNode = tableObj.table[50][50];

        boolean lastCircle = false;

        waitForStart();

        while(opModeIsActive()) {
            Pose2d pose = driveLocalizer.getPose();

            boolean currentCircle = gamepad1.circle;
            if (currentCircle && !lastCircle) {
                Vector2d poseCoordinates = new Vector2d(pose.position.x, pose.position.y);
                Vector2d NodeCoordinates = Utils.robotNode(poseCoordinates);

                int nodeX = (int) -NodeCoordinates.x;
                int nodeY = (int) NodeCoordinates.y;

                if (nodeX < 0 || nodeX >= tableObj.columns || nodeY < 0 || nodeY >= tableObj.rows) {
                    telemetry.addLine("Invalid node index for A*!");
                    telemetry.addData("X", nodeX);
                    telemetry.addData("Y", nodeY);
                    telemetry.update();
                    return;
                }

                Node startNode = tableObj.table[(int)NodeCoordinates.y][(int)-NodeCoordinates.x];
                startNode.isStart = true;
                endNode.isEnd = true;

                telemetry.addLine("a*");
                telemetry.addData("y grid", (int)NodeCoordinates.y);
                telemetry.addData("x grid", (int)-NodeCoordinates.x);
                telemetry.update();
                List<Node> path = null;
                try {
                    path = PointToPoint.aStar(tableObj, startNode, endNode, robotPointToPoint);
                    if (path == null) {
                        return;
                    }
                } catch (Exception e) {
                    telemetry.addLine("Exception during A* pathing");
                    telemetry.addData("Message", e.getMessage());
                    telemetry.update();
                    e.printStackTrace(); // Also shows in logcat
                }

                tableObj.applyPathToTable(path);
                telemetry.addLine("apply");
                tableObj.generateRobot(robotPointToPoint, new int[]{startNode.y, startNode.x});
                telemetry.addLine("generate map");
                telemetry.update();
                List<String> moves = PointToPoint.pathToCommands(path, tableObj);
                telemetry.update();
                List<String> compressedMoves = PointToPoint.compressPath(moves);
                for (String command : compressedMoves) {
                    telemetry.addLine(command);
                }
                telemetry.update();
                PointToPoint.executePath(compressedMoves, drive, pose);
            }
            lastCircle = currentCircle;

            Utils.displayCodeVersion(telemetry,"7.9.25.18.15");
            Utils.displayCoordonates(telemetry, pose.position.x, pose.position.y, pose.heading.toDouble());
            telemetry.update();
            driveLocalizer.update();
        }
    }
}