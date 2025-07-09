package org.firstinspires.ftc.teamcode.teleop.presets.pointToPoint;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.Utils;

public class Movement {

    public static Action right(double cm, MecanumDrive drive, Pose2d pose) {
        double dx = Utils.cmToInches(cm);
        return drive.actionBuilder(pose)
                .turnTo(Math.toRadians(90))
                .lineToX(pose.position.x + dx)
                .build();
    }

    public static Action down(double cm, MecanumDrive drive, Pose2d pose) {
        double dy = Utils.cmToInches(cm);
        return drive.actionBuilder(pose)
                .turnTo(Math.toRadians(180))
                .lineToY(pose.position.y - dy)
                .build();
    }

    public static Action left(double cm, MecanumDrive drive, Pose2d pose) {
        double dx = Utils.cmToInches(cm);
        return drive.actionBuilder(pose)
                .turnTo(Math.toRadians(270))
                .lineToX(pose.position.x - dx)
                .build();
    }

    public static Action up(double cm, MecanumDrive drive, Pose2d pose) {
        double dy = Utils.cmToInches(cm);
        return drive.actionBuilder(pose)
                .turnTo(Math.toRadians(0))
                .lineToY(pose.position.y + dy)
                .build();
    }

    public static Action rightDown(double cm, MecanumDrive drive, Pose2d pose) {
        double delta = Utils.cmToInches(cm);
        return drive.actionBuilder(pose)
                .turnTo(Math.toRadians(135))
                .splineTo(new Vector2d(pose.position.x + delta, pose.position.y - delta), Math.toRadians(135))
                .build();
    }

    public static Action leftDown(double cm, MecanumDrive drive, Pose2d pose) {
        double delta = Utils.cmToInches(cm);
        return drive.actionBuilder(pose)
                .turnTo(Math.toRadians(225))
                .splineTo(new Vector2d(pose.position.x - delta, pose.position.y - delta), Math.toRadians(225))
                .build();
    }

    public static Action leftUp(double cm, MecanumDrive drive, Pose2d pose) {
        double delta = Utils.cmToInches(cm);
        return drive.actionBuilder(pose)
                .turnTo(Math.toRadians(315))
                .splineTo(new Vector2d(pose.position.x - delta, pose.position.y + delta), Math.toRadians(315))
                .build();
    }

    public static Action rightUp(double cm, MecanumDrive drive, Pose2d pose) {
        double delta = Utils.cmToInches(cm);
        return drive.actionBuilder(pose)
                .turnTo(Math.toRadians(45))
                .splineTo(new Vector2d(pose.position.x + delta, pose.position.y + delta), Math.toRadians(45))
                .build();
    }
}
