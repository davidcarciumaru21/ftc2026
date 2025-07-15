package org.firstinspires.ftc.teamcode.systems.arm;

import com.acmerobotics.dashboard.config.Config;

@Config
/* loaded from: classes7.dex */
public class Positions {
    public static double basketX = -11.91d;
    public static double basketY = 88.67d;
    public static boolean basketElbowUp = true;
    public static double submersibil1X = 87.97d;
    public static double submersibil1Y = -20.18d;
    public static boolean submersibil1ElbowUp = true;
    public static double submersibil2X = 85.31d;
    public static double submersibil2Y = -36.63d;
    public static boolean submersibil2ElbowUp = true;
    public static double specimenUpX = 27.8d;
    public static double specimenUpY = 63.0d;
    public static boolean specimenUpElbowUp = true;
    public static double specimenDownX = 27.85d;
    public static double specimenDownY = 55.62d;
    public static boolean specimenDownElbowUp = true;
    public static double perimeterX = 43.0d;
    public static double perimeterY = 26.0d;
    public static boolean perimeterElbowUp = true;
    public static double startX = -1.37d;
    public static double startY = 8.9d;
    public static boolean startElbowUp = true;
    public static double perimeterUPX = 43.0d;
    public static double perimeterUPY = 30.0d;
    public static boolean perimeterUPElbowUp = true;
    public static double basket3X = 79.84d;
    public static double basket3Y = -33.27d;
    public static boolean basket3ElbowUp = false;

    public static ArmPosition getBasket() {
        return new ArmPosition(basketX, basketY, basketElbowUp);
    }

    public static ArmPosition getBasket3() {
        return new ArmPosition(basket3X, basket3Y, basket3ElbowUp);
    }

    public static ArmPosition start() {
        return new ArmPosition(startX, startY, startElbowUp);
    }

    public static ArmPosition getSubmersibil1() {
        return new ArmPosition(submersibil1X, submersibil1Y, submersibil1ElbowUp);
    }

    public static ArmPosition getSubmersibil2() {
        return new ArmPosition(submersibil2X, submersibil2Y, submersibil2ElbowUp);
    }

    public static ArmPosition getSpecimenUp() {
        return new ArmPosition(specimenUpX, specimenUpY, specimenUpElbowUp);
    }

    public static ArmPosition getSpecimenDown() {
        return new ArmPosition(specimenDownX, specimenDownY, specimenDownElbowUp);
    }

    public static ArmPosition getPerimeter() {
        return new ArmPosition(perimeterX, perimeterY, perimeterElbowUp);
    }

    public static ArmPosition getPerimeterUP() {
        return new ArmPosition(perimeterUPX, perimeterUPY, perimeterUPElbowUp);
    }

    public static class ArmPosition {
        public boolean elbowUp;
        public double x;
        public double y;

        public ArmPosition(double x, double y, boolean elbowUp) {
            this.x = x;
            this.y = y;
            this.elbowUp = elbowUp;
        }
    }
}