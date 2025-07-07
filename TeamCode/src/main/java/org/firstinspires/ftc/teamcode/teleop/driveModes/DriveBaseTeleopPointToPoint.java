package org.firstinspires.ftc.teamcode.teleop.driveModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
    private Robot robotPreset = new Robot(6, 6);
    private Table tableObj = new Table(120, 120, 1);
    private List<Node> path;
    Pose2d startPose = new Pose2d(0, 0, 0);
    MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
    MecanumDrive.DriveLocalizer driveLocalizer = drive.new DriveLocalizer(startPose);

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        for (int y = 30; y < 90; y++) {
            for (int x = 40; x < 80; x++) {
                tableObj.table[y][x].isObstacle = true;
            }
        }

        tableObj.mark_unable_to_reach_nodes(robotPreset);
        tableObj.draw_border(robotPreset);

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.circle){
                Vector2d robotCoordinates = driveLocalizer.getPose().position;

                Node start = new Node((int)robotCoordinates.x, (int)robotCoordinates.y);
                start.isStart = true;

                Node end = tableObj.table[3][3];
                end.isEnd = true;

                path = PointToPoint.aStar(tableObj, start, end, robotPreset);
                tableObj.apply_path_to_table(path);
                tableObj.generate_robot(robotPreset, new int[]{start.y, start.x});
                tableObj.print_matrix();

                if (path != null) {
                    List<String> moves = PointToPoint.path_to_commands(path, tableObj);
                    List<String> compressedMoves =  PointToPoint.compress_path(moves);
                    PointToPoint.execute_path(compressedMoves, drive, driveLocalizer.getPose());
                } else {
                    System.out.println("\u001B[31mNo path found!\u001B[0m");  // Red text
                }
            }
        }
    }
}
