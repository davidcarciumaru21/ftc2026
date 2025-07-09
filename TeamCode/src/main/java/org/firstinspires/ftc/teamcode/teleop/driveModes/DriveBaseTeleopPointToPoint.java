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
        int startX = (int)(Utils.cmToInches((double) robotPointToPoint.width / 2)), startY = (int)(Utils.cmToInches((double) robotPointToPoint.height) / 2);
        int startHeading = 0; // in degrees
        Pose2d startPose = new Pose2d(startX, startY, Math.toRadians(startHeading));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        MecanumDrive.DriveLocalizer driveLocalizer = drive.new DriveLocalizer(startPose);
        Table tableObj = new Table(360, 360, 10);
        tableObj.markUnableToReachNodes(robotPointToPoint);
        tableObj.drawBorder(robotPointToPoint);

        waitForStart();

        while(opModeIsActive()) {
            Pose2d pose = driveLocalizer.getPose();

            if (gamepad1.circleWasPressed()) {
                Vector2d poseCoordinates = new Vector2d(pose.position.x, pose.position.y);
                Vector2d NodeCoordinates = Utils.robotNode(poseCoordinates, drive.PARAMS.inPerTick, tableObj.mu);

                Node startNode = tableObj.table[(int)NodeCoordinates.y][(int)NodeCoordinates.x];
                startNode.isStart = true;
                Node endNode = tableObj.table[200][200];
                endNode.isEnd = true;

                List<Node> path = PointToPoint.aStar(tableObj, startNode, endNode, robotPointToPoint);
                assert path != null;
                tableObj.applyPathToTable(path);
                tableObj.generateRobot(robotPointToPoint, new int[]{startNode.y, startNode.x});
                tableObj.printMatrix();
                List<String> moves = PointToPoint.pathToCommands(path, tableObj);
                List<String> compressedMoves = PointToPoint.compressPath(moves);
                PointToPoint.executePath(compressedMoves, drive, pose);
            }

            Utils.displayCodeVersion(telemetry,"7.9.25.18.15");
            Utils.displayCoordonates(telemetry, pose.position.x, pose.position.y, pose.heading.toDouble());
            telemetry.update();
            driveLocalizer.update();
        }
    }
}