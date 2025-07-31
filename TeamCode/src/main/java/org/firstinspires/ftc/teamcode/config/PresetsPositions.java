package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.teamcode.Utils.MeasurementUnits;

@Config
public class PresetsPositions {

    //==============================Positions==============================

    public static double blueSubX = -20;
    public static double blueSubY = -120;
    public static double blueSubHeadingDeg = 90;

    public static double forwardBlueSubX = -20;
    public static double forwardBlueSubY = -80;
    public static double forwardBlueSubHeadingDeg = 90;

    public static double redSubX = -20;
    public static double redSubY = -120;
    public static double redSubHeadingDeg = 90;

    public static double forwardRedSubX = -20;
    public static double forwardRedSubY = -80;
    public static double forwardRedSubHeadingDeg = 90;

    public static double blueBaseX = -140;
    public static double blueBaseY = -120;
    public static double blueBaseHeadingDeg = 0;

    public static double redBaseX = -140;
    public static double redBaseY = 120;
    public static double redBaseHeadingDeg = 0;

    public static double blueScoreSubX = -100;
    public static double blueScoreSubY = -20;
    public static double blueScoreSubHeadingDeg = 0;

    public static double redScoreSubX = -100;
    public static double redScoreSubY = 0;
    public static double redScoreSubHeadingDeg = 0;

    //=============================Pose getters============================

    public static Pose2d getBlueSubmersiblePose() {
        return new Pose2d(
                MeasurementUnits.cmToInches(blueSubX),
                MeasurementUnits.cmToInches(blueSubY),
                Math.toRadians(blueSubHeadingDeg)
        );
    }

    public static Pose2d getForwardBlueSubmersiblePose() {
        return new Pose2d(
                MeasurementUnits.cmToInches(forwardBlueSubX),
                MeasurementUnits.cmToInches(forwardBlueSubY),
                Math.toRadians(forwardBlueSubHeadingDeg)
        );
    }

    public static Pose2d getRedSubmersiblePose() {
        return new Pose2d(
                MeasurementUnits.cmToInches(redSubX),
                MeasurementUnits.cmToInches(redSubY),
                Math.toRadians(redSubHeadingDeg)
        );
    }

    public static Pose2d getForwardRedSubmersiblePose() {
        return new Pose2d(
                MeasurementUnits.cmToInches(forwardRedSubX),
                MeasurementUnits.cmToInches(forwardRedSubY),
                Math.toRadians(forwardRedSubHeadingDeg)
        );
    }

    public static Pose2d getBlueBasePose() {
        return new Pose2d(
                MeasurementUnits.cmToInches(blueBaseX),
                MeasurementUnits.cmToInches(blueBaseY),
                Math.toRadians(blueBaseHeadingDeg)
        );
    }

    public static Pose2d getRedBasePose() {
        return new Pose2d(
                MeasurementUnits.cmToInches(redBaseX),
                MeasurementUnits.cmToInches(redBaseY),
                Math.toRadians(redBaseHeadingDeg)
        );
    }

    public static Pose2d getBlueScoringSubmersiblePose() {
        return new Pose2d(
                MeasurementUnits.cmToInches(blueScoreSubX),
                MeasurementUnits.cmToInches(blueScoreSubY),
                Math.toRadians(blueScoreSubHeadingDeg)
        );
    }

    public static Pose2d getRedScoringSubmersiblePose() {
        return new Pose2d(
                MeasurementUnits.cmToInches(redScoreSubX),
                MeasurementUnits.cmToInches(redScoreSubY),
                Math.toRadians(redScoreSubHeadingDeg)
        );
    }
}
