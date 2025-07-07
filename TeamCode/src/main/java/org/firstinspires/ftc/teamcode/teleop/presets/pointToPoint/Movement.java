package org.firstinspires.ftc.teamcode.teleop.presets.pointToPoint;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Movement {

    public static Action right(double length, MecanumDrive drive, Pose2d pose) {
        return drive.actionBuilder(pose)
                .lineToX(pose.position.x + length)
                .build();
    }

    public static Action rightDown(double length, MecanumDrive drive, Pose2d pose) {
        return drive.actionBuilder(pose)
                .splineTo(new Vector2d(pose.position.x + length, pose.position.y - length), pose.heading)
                .build();
    }

    public static Action down(double length, MecanumDrive drive, Pose2d pose) {
        return drive.actionBuilder(pose)
                .lineToY(pose.position.y - length)
                .build();
    }

    public static Action leftDown(double length, MecanumDrive drive, Pose2d pose) {
        return drive.actionBuilder(pose)
                .splineTo(new Vector2d(pose.position.x - length, pose.position.y - length), pose.heading)
                .build();
    }

    public static Action left(double length, MecanumDrive drive, Pose2d pose) {
        return drive.actionBuilder(pose)
                .lineToX(pose.position.x - length)
                .build();
    }

    public static Action leftUp(double length, MecanumDrive drive, Pose2d pose) {
        return drive.actionBuilder(pose)
                .splineTo(new Vector2d(pose.position.x - length, pose.position.y + length), pose.heading)
                .build();
    }

    public static Action up(double length, MecanumDrive drive, Pose2d pose) {
        return drive.actionBuilder(pose)
                .lineToY(pose.position.y + length)
                .build();
    }

    public static Action rightUp(double length, MecanumDrive drive, Pose2d pose) {
        return drive.actionBuilder(pose)
                .splineTo(new Vector2d(pose.position.x + length, pose.position.y + length), pose.heading)
                .build();
    }
}
