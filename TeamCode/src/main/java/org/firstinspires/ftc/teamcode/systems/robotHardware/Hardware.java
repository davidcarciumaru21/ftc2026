package org.firstinspires.ftc.teamcode.systems.robotHardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.enums.RobotInitialization;

/**
 * Hardware class to manage and initialize the drivetrain motors of the robot.
 */
public final class Hardware {
    // DcMotor objects representing each motor on the robot drivetrain
    public DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    /**
     * Initializes the hardware by mapping motors from the hardware map and configuring them.
     *
     * @param hardwareMap The hardware map from the FTC SDK to access hardware devices
     * @param mode A byte indicating the configuration mode (e.g., 1 to reverse right side motors)
     */
    public void init(HardwareMap hardwareMap, RobotInitialization mode) {
        // Retrieve motor instances from hardware map using configured names
        frontLeftMotor = hardwareMap.get(DcMotor.class, "MotorFL");
        backLeftMotor = hardwareMap.get(DcMotor.class, "MotorBL");
        frontRightMotor = hardwareMap.get(DcMotor.class, "MotorFR");
        backRightMotor = hardwareMap.get(DcMotor.class, "MotorBR");

        // If mode is 1, reverse the direction of the right-side motors (common for drivetrain alignment)
        if (mode == RobotInitialization.WithoutRoadRunner) {
            frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        // Set zero power behavior to BRAKE so motors stop immediately when power is zero
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set motors to run without encoders (for manual power control or open loop control)
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
