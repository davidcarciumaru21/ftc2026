package org.firstinspires.ftc.teamcode.presets;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.teamcode.trajectories.MecanumDrive;

public class Movement {

    private static final double FIXED_HEADING = 0; // radiani (0 = fata in directia X)


    public static Action right(double cm, MecanumDrive drive, Pose2d pose) {
        return drive.actionBuilder(pose)
                .strafeTo(new Vector2d(pose.position.x, pose.position.y + cm))
                .build();
    }

    public static Action down(double cm, MecanumDrive drive, Pose2d pose) {
        return drive.actionBuilder(pose)
                .lineToXConstantHeading(pose.position.x - cm)
                .build();
    }

    public static Action left(double cm, MecanumDrive drive, Pose2d pose) {
        return drive.actionBuilder(pose)
                .strafeTo(new Vector2d(pose.position.x, pose.position.y - cm))
                .build();
    }

    public static Action up(double cm, MecanumDrive drive, Pose2d pose) {
        return drive.actionBuilder(pose)
                .lineToXConstantHeading(pose.position.x + cm)
                .build();
    }

    public static Action rightDown(double cm, MecanumDrive drive, Pose2d pose) {
        return drive.actionBuilder(pose)
                .splineToConstantHeading(
                        new Vector2d(pose.position.x - cm, pose.position.y + cm),
                        FIXED_HEADING
                )
                .build();
    }

    public static Action leftDown(double cm, MecanumDrive drive, Pose2d pose) {
        return drive.actionBuilder(pose)
                .splineToConstantHeading(
                        new Vector2d(pose.position.x - cm, pose.position.y - cm),
                        FIXED_HEADING
                )
                .build();
    }

    public static Action leftUp(double cm, MecanumDrive drive, Pose2d pose) {
        return drive.actionBuilder(pose)
                .splineToConstantHeading(
                        new Vector2d(pose.position.x + cm, pose.position.y - cm),
                        FIXED_HEADING
                )
                .build();
    }

    public static Action rightUp(double cm, MecanumDrive drive, Pose2d pose) {
        return drive.actionBuilder(pose)
                .splineToConstantHeading(
                        new Vector2d(pose.position.x + cm, pose.position.y + cm),
                        FIXED_HEADING
                )
                .build();
    }
}
