package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.trajectories.MecanumDrive.PARAMS;

import android.graphics.Point;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utils.Utils;
import org.firstinspires.ftc.teamcode.presets.Node;
import org.firstinspires.ftc.teamcode.presets.Table;
import org.firstinspires.ftc.teamcode.presets.Robot;
import org.firstinspires.ftc.teamcode.presets.PointToPoint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.robotHardware.Hardware;
import org.firstinspires.ftc.teamcode.trajectories.MecanumDrive;
import org.firstinspires.ftc.teamcode.trajectories.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.presets.Movement;
import java.util.List;

@TeleOp(name = "DriveBase-TeleOpPointToPoint", group = "Dev-Teleops")
public class DriveBaseTeleopPointToPoint extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Table tableObj = new Table(40, 40, 1);
        Robot robotPoinToPoint = new Robot((int)Utils.cmToInches(35.0), (int)Utils.cmToInches(43.0));
        double startX = robotPoinToPoint.height / 2, startY = robotPoinToPoint.width / 2;
        double startHeading = Math.toDegrees(0);
        Pose2d startPose = new Pose2d(startX, startY, startHeading);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        ThreeDeadWheelLocalizer driveLocalizer = new ThreeDeadWheelLocalizer(hardwareMap, PARAMS.inPerTick, startPose);

        tableObj.drawBorder(robotPoinToPoint);
        tableObj.markUnableToReachNodes(robotPoinToPoint);

        boolean lastBState = false;
        boolean currentBState;

        int xNode, yNode;
        Node currentNode;

        Node endNode = tableObj.table[20][20];

        List<Node> path;

        waitForStart();

        while(opModeIsActive()) {

            driveLocalizer.update();
            Pose2d currentPose = driveLocalizer.getPose();

            yNode = 2 * (int) startX - (int) Math.round(currentPose.position.x);
            xNode = (int) Math.round(currentPose.position.y);
            currentNode = tableObj.table[yNode][xNode];

            currentBState = gamepad1.circle;

            if (currentBState && !lastBState) {
                telemetry.addLine("hello");
                telemetry.update();
                try {
                    currentNode.isStart = true;
                    endNode.isEnd = true;
                    tableObj.resetNodeStates();
                    tableObj.drawBorder(robotPoinToPoint);
                    tableObj.markUnableToReachNodes(robotPoinToPoint);
                    tableObj.generateRobot(robotPoinToPoint, new int[] {yNode, xNode});

                    path = PointToPoint.aStar(tableObj, currentNode, endNode, robotPoinToPoint);
                    tableObj.applyPathToTable(path);
                    List<String> moves = PointToPoint.pathToCommands(path, tableObj);
                    List<String> compressedMoves = PointToPoint.compressPath(moves);
                    PointToPoint.executePath(compressedMoves, drive, currentPose);

                } catch(Exception e) {
                    telemetry.addLine("Unable to get to the point");
                    telemetry.update();
                }
            }
            telemetry.addData("x", currentPose.position.x);
            telemetry.addData("y", currentPose.position.y);
            telemetry.addData("NodeX", xNode);
            telemetry.addData("Nodey", yNode);
            telemetry.update();
            lastBState = currentBState;
        }
    }
}