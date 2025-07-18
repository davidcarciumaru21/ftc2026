package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.teamcode.trajectories.roadRunnerConfigurations.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utils.MeasurementUnit;

public class Movement {

    /**
     * Move right (positive Y direction) by a specified distance in centimeters.
     */
    public static Action right(double y, MecanumDrive drive, Pose2d pose) {
        double yI = MeasurementUnit.cmToInches(y);
        return drive.actionBuilder(pose)
                .strafeTo(new Vector2d(pose.position.x, pose.position.y + yI))
                .build();
    }

    /**
     * Move backward (negative X direction) by a specified distance in centimeters.
     */
    public static Action down(double x, MecanumDrive drive, Pose2d pose) {
        double xI = MeasurementUnit.cmToInches(x);
        return drive.actionBuilder(pose)
                .lineToXConstantHeading(pose.position.x - xI)
                .build();
    }

    /**
     * Move left (negative Y direction) by a specified distance in centimeters.
     */
    public static Action left(double y, MecanumDrive drive, Pose2d pose) {
        double yI = MeasurementUnit.cmToInches(y);
        return drive.actionBuilder(pose)
                .strafeTo(new Vector2d(pose.position.x, pose.position.y - yI))
                .build();
    }

    /**
     * Move forward (positive X direction) by a specified distance in centimeters.
     */
    public static Action up(double x, MecanumDrive drive, Pose2d pose) {
        double xI = MeasurementUnit.cmToInches(x);
        return drive.actionBuilder(pose)
                .lineToXConstantHeading(pose.position.x + xI)
                .build();
    }

    /**
     * Strafe in any dirrection
     */

    public static Action strafe(double x, double y, MecanumDrive drive, Pose2d pose) {
        double xI = MeasurementUnit.cmToInches(x);
        double yI = MeasurementUnit.cmToInches(y);
        return  drive.actionBuilder(pose)
                .strafeTo(new Vector2d(pose.position.x + xI, pose.position.y + yI))
                .build();
    }


    public static Action spline(double x, double y, MecanumDrive drive, Pose2d pose, double heading){
        double xI = MeasurementUnit.cmToInches(x);
        double yI = MeasurementUnit.cmToInches(y);
        return drive.actionBuilder(pose)
                .splineToLinearHeading(new Pose2d(pose.position.x + xI, pose.position.y + yI, Math.toRadians(heading)), 0)
                .build();
    }

    /**
     * Turns the robot to an absolute angle in degrees.
     */
    public static Action turnTo(double degrees, MecanumDrive drive, Pose2d pose) {
        return drive.actionBuilder(pose)
                .turnTo(Math.toRadians(degrees))
                .build();
    }

    /**
     * Turns the robot by a relative angle in degrees.
     */
    public static Action turn(double degrees, MecanumDrive drive, Pose2d pose) {
        return drive.actionBuilder(pose)
                .turn(Math.toRadians(degrees))
                .build();
    }
}