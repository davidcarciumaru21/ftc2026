package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "MotorCalibration", group = "Calibration")
public class MotorCalibration extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void runOpMode() {

        // Map hardware
        frontLeft = hardwareMap.get(DcMotor.class, "1");
        frontRight = hardwareMap.get(DcMotor.class, "2");
        backLeft = hardwareMap.get(DcMotor.class, "3");
        backRight = hardwareMap.get(DcMotor.class, "4");



        // Reset encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set to run using encoder (not to position)
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Ready. Press Play.");
        telemetry.update();

        waitForStart();

        // Set power to 1.0
        frontLeft.setPower(1.0);
        frontRight.setPower(1.0);
        backLeft.setPower(1.0);
        backRight.setPower(1.0);

        // Run for 10 seconds
        sleep(10_000);

        // Stop motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // Show encoder values
        telemetry.addData("Front Left Ticks", frontLeft.getCurrentPosition());
        telemetry.addData("Front Right Ticks", frontRight.getCurrentPosition());
        telemetry.addData("Back Left Ticks", backLeft.getCurrentPosition());
        telemetry.addData("Back Right Ticks", backRight.getCurrentPosition());
        telemetry.update();

        // Keep showing data
        while (opModeIsActive()) {
            idle();
        }
    }
}
