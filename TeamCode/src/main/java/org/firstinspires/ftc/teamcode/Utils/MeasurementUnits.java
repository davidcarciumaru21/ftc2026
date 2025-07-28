package org.firstinspires.ftc.teamcode.Utils;

public class MeasurementUnits {
    /**
     * Converts centimeters to inches.
     *
     * @param cm Value in centimeters.
     * @return Equivalent value in inches.
     */
    public static double cmToInches(double cm) {
        return cm / 2.54;
    }

    /**
     * Converts inches to centimeters.
     *
     * @param cm Value in inches (parameter name should ideally be inches, but kept consistent).
     * @return Equivalent value in centimeters.
     */
    public static double inchesToCm(double cm) {
        return cm * 2.54;
    }
}
