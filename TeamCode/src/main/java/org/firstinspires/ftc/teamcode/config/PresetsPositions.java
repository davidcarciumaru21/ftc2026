package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.teamcode.Utils.MeasurementUnits;

@Config
public class PresetsPositions {
    public static Pose2d blueBasketPosition = new Pose2d(
            MeasurementUnits.cmToInches(-70),
            MeasurementUnits.cmToInches(30),
            Math.toRadians(-45)
    );
    public static Pose2d redBasketPosition = new Pose2d(
            MeasurementUnits.cmToInches(-70),
            MeasurementUnits.cmToInches(30),
            Math.toRadians(-45)
    );

    public static Pose2d blueSubmersiblePosition = new Pose2d(0, 0, 0);
    public static Pose2d redSubmersibletPosition = new Pose2d(0, 0, 0);

    public static Pose2d blueBasePosition = new Pose2d(0, 0, 0);
    public static Pose2d redBasePosition = new Pose2d(0, 0, 0);

    public static Pose2d blueBase2Position = new Pose2d(0, 0, 0);
    public static Pose2d redBase2Position = new Pose2d(0, 0, 0);
}
