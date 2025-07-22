package org.firstinspires.ftc.teamcode.presets;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.teamcode.roadRunner.drives.MecanumDrive;

public class MovementPointToPoint {

    // Fixed heading (in radians) used for spline paths that should maintain a straight orientation
    private static final double FIXED_HEADING = 0; // 0 radians = facing forward along the X-axis

    /**
     * Move right (strafe) by a specified distance in inches.
     */
    public static Action right(double inches, MecanumDrive drive, Pose2d pose) {
        return drive.actionBuilder(pose)
                .strafeTo(new Vector2d(pose.position.x, pose.position.y + inches))
                .build();
    }

    /**
     * Move backward (down along the X-axis) by a specified distance in inches.
     */
    public static Action down(double inches, MecanumDrive drive, Pose2d pose) {
        return drive.actionBuilder(pose)
                .lineToXConstantHeading(pose.position.x - inches)
                .build();
    }

    /**
     * Move left (strafe) by a specified distance in inches.
     */
    public static Action left(double inches, MecanumDrive drive, Pose2d pose) {
        return drive.actionBuilder(pose)
                .strafeTo(new Vector2d(pose.position.x, pose.position.y - inches))
                .build();
    }

    /**
     * Move forward (up along the X-axis) by a specified distance in inches.
     */
    public static Action up(double inches, MecanumDrive drive, Pose2d pose) {
        return drive.actionBuilder(pose)
                .lineToXConstantHeading(pose.position.x + inches)
                .build();
    }

    /**
     * Move diagonally down-right (X decreases, Y increases) by a specified distance in inches.
     */
    public static Action rightDown(double inches, MecanumDrive drive, Pose2d pose) {
        return drive.actionBuilder(pose)
                .splineToConstantHeading(
                        new Vector2d(pose.position.x - inches, pose.position.y + inches),
                        FIXED_HEADING
                )
                .build();
    }

    /**
     * Move diagonally down-left (X decreases, Y decreases) by a specified distance in inches.
     */
    public static Action leftDown(double inches, MecanumDrive drive, Pose2d pose) {
        return drive.actionBuilder(pose)
                .splineToConstantHeading(
                        new Vector2d(pose.position.x - inches, pose.position.y - inches),
                        FIXED_HEADING
                )
                .build();
    }

    /**
     * Move diagonally up-left (X increases, Y decreases) by a specified distance in inches.
     */
    public static Action leftUp(double inches, MecanumDrive drive, Pose2d pose) {
        return drive.actionBuilder(pose)
                .splineToConstantHeading(
                        new Vector2d(pose.position.x + inches, pose.position.y - inches),
                        FIXED_HEADING
                )
                .build();
    }

    /**
     * Move diagonally up-right (X increases, Y increases) by a specified distance in inches.
     */
    public static Action rightUp(double inches, MecanumDrive drive, Pose2d pose) {
        return drive.actionBuilder(pose)
                .splineToConstantHeading(
                        new Vector2d(pose.position.x + inches, pose.position.y + inches),
                        FIXED_HEADING
                )
                .build();
    }
}