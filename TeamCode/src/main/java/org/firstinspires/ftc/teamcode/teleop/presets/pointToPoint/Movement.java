package org.firstinspires.ftc.teamcode.teleop.presets.pointToPoint;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.Utils;

public class Movement {

    private static final double FIXED_HEADING = 0; // radiani (0 = fata in directia X)

    public static Action right(double cm, MecanumDrive drive, Pose2d pose) {
        double dx = Utils.cmToInches(cm);
        return drive.actionBuilder(pose)
                .lineToXConstantHeading(pose.position.x + dx)
                .build();
    }

    public static Action down(double cm, MecanumDrive drive, Pose2d pose) {
        double dy = Utils.cmToInches(cm);
        return drive.actionBuilder(pose)
                .lineToYConstantHeading(pose.position.y - dy)
                .build();
    }

    public static Action left(double cm, MecanumDrive drive, Pose2d pose) {
        double dx = Utils.cmToInches(cm);
        return drive.actionBuilder(pose)
                .lineToXConstantHeading(pose.position.x - dx)
                .build();
    }

    public static Action up(double cm, MecanumDrive drive, Pose2d pose) {
        double dy = Utils.cmToInches(cm);
        return drive.actionBuilder(pose)
                .lineToYConstantHeading(pose.position.y + dy)
                .build();
    }

    public static Action rightDown(double cm, MecanumDrive drive, Pose2d pose) {
        double delta = Utils.cmToInches(cm);
        return drive.actionBuilder(pose)
                .splineToConstantHeading(
                        new Vector2d(pose.position.x + delta, pose.position.y - delta),
                        FIXED_HEADING
                )
                .build();
    }

    public static Action leftDown(double cm, MecanumDrive drive, Pose2d pose) {
        double delta = Utils.cmToInches(cm);
        return drive.actionBuilder(pose)
                .splineToConstantHeading(
                        new Vector2d(pose.position.x - delta, pose.position.y - delta),
                        FIXED_HEADING
                )
                .build();
    }

    public static Action leftUp(double cm, MecanumDrive drive, Pose2d pose) {
        double delta = Utils.cmToInches(cm);
        return drive.actionBuilder(pose)
                .splineToConstantHeading(
                        new Vector2d(pose.position.x - delta, pose.position.y + delta),
                        FIXED_HEADING
                )
                .build();
    }

    public static Action rightUp(double cm, MecanumDrive drive, Pose2d pose) {
        double delta = Utils.cmToInches(cm);
        return drive.actionBuilder(pose)
                .splineToConstantHeading(
                        new Vector2d(pose.position.x + delta, pose.position.y + delta),
                        FIXED_HEADING
                )
                .build();
    }
}
