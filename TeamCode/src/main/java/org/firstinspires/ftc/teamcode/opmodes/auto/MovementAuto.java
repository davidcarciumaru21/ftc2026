package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.teamcode.roadRunner.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunner.localizer.ThreeDeadWheelLocalizer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Utils.MeasurementUnits;

public class MovementAuto {
    private final HardwareMap hardwareMap;

    public MovementAuto(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }

    /**
     * Move straight in any dirrection
     */
    public Action straight(double x, ThreeDeadWheelLocalizer localizer) {
        localizer.update();
        Pose2d pose = localizer.getPose();
        MecanumDrive.PARAMS.maxWheelVel = 50;
        MecanumDrive.PARAMS.maxProfileAccel = 50;
        double xI = MeasurementUnits.cmToInches(x);
        MecanumDrive drive = new MecanumDrive(this.hardwareMap, pose);
        return drive.actionBuilder(pose)
                .lineToXConstantHeading(pose.position.x + xI)
                .build();
    }

    public Action straight(double x, ThreeDeadWheelLocalizer localizer, double vel, double accel) {
        double xI = MeasurementUnits.cmToInches(x);
        MecanumDrive.PARAMS.maxWheelVel = vel;
        MecanumDrive.PARAMS.maxProfileAccel = accel;
        localizer.update();
        Pose2d pose = localizer.getPose();
        MecanumDrive drive = new MecanumDrive(this.hardwareMap, pose);
        return drive.actionBuilder(pose)
                .lineToXConstantHeading(pose.position.x + xI)
                .build();
    }

    /**
     * Strafe in any dirrection
     */
    public Action strafe(double x, double y, ThreeDeadWheelLocalizer localizer) {
        localizer.update();
        Pose2d pose = localizer.getPose();
        double xI = MeasurementUnits.cmToInches(x);
        double yI = MeasurementUnits.cmToInches(y);
        MecanumDrive drive = new MecanumDrive(this.hardwareMap, pose);
        return  drive.actionBuilder(pose)
                .strafeTo(new Vector2d(pose.position.x + xI, pose.position.y + yI))
                .build();
    }

    /**
     * Spline to linear heading
     */
    public Action spline(double x, double y, ThreeDeadWheelLocalizer localizer, double heading){
        localizer.update();
        Pose2d pose = localizer.getPose();
        double xI = MeasurementUnits.cmToInches(x);
        double yI = MeasurementUnits.cmToInches(y);
        MecanumDrive drive = new MecanumDrive(this.hardwareMap, pose);
        return drive.actionBuilder(pose)
                .splineToLinearHeading(new Pose2d(pose.position.x + xI, pose.position.y + yI, Math.toRadians(heading)), 0)
                .build();
    }

    /**
     * Turns the robot to an absolute angle in degrees.
     */

    public Action turnTo(double degrees, ThreeDeadWheelLocalizer localizer, double vel, double accel) {
        MecanumDrive.PARAMS.maxAngVel = Math.PI * (vel / 100);
        MecanumDrive.PARAMS.maxAngAccel = Math.PI * (accel / 100);
        localizer.update();
        Pose2d pose = localizer.getPose();
        MecanumDrive drive = new MecanumDrive(this.hardwareMap, pose);
        return drive.actionBuilder(pose)
                .turnTo(Math.toRadians(degrees))
                .build();
    }

    public Action turnTo(double degrees, ThreeDeadWheelLocalizer localizer) {
        MecanumDrive.PARAMS.maxAngVel = Math.PI;
        MecanumDrive.PARAMS.maxAngAccel = Math.PI;
        localizer.update();
        Pose2d pose = localizer.getPose();
        MecanumDrive drive = new MecanumDrive(this.hardwareMap, pose);
        return drive.actionBuilder(pose)
                .turnTo(Math.toRadians(degrees))
                .build();
    }

    /**
     * Turns the robot by a relative angle in degrees.
     */

    public Action turn(double degrees, ThreeDeadWheelLocalizer localizer, double vel, double accel) {
        MecanumDrive.PARAMS.maxAngVel = Math.PI * (vel / 100);
        MecanumDrive.PARAMS.maxAngAccel = Math.PI * (accel / 100);
        localizer.update();
        Pose2d pose = localizer.getPose();
        MecanumDrive drive = new MecanumDrive(this.hardwareMap, pose);
        return drive.actionBuilder(pose)
                .turn(Math.toRadians(degrees))
                .build();
    }

    public Action turn(double degrees, ThreeDeadWheelLocalizer localizer) {
        MecanumDrive.PARAMS.maxAngVel = Math.PI;
        MecanumDrive.PARAMS.maxAngAccel = Math.PI;
        localizer.update();
        Pose2d pose = localizer.getPose();
        MecanumDrive drive = new MecanumDrive(this.hardwareMap, pose);
        return drive.actionBuilder(pose)
                .turn(Math.toRadians(degrees))
                .build();
    }
}