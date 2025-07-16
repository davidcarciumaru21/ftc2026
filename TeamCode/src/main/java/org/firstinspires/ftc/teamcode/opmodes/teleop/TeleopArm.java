package org.firstinspires.ftc.teamcode.opmodes.teleop;

// Imports for configuration, sensors, drive systems, and control
import static org.firstinspires.ftc.teamcode.trajectories.roadRunnerConfigurations.MecanumDrive.PARAMS;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Utils.MeasurementUnit;
import org.firstinspires.ftc.teamcode.Utils.Telemetry;
import org.firstinspires.ftc.teamcode.presets.Node;
import org.firstinspires.ftc.teamcode.presets.Table;
import org.firstinspires.ftc.teamcode.presets.Robot;
import org.firstinspires.ftc.teamcode.presets.PointToPoint;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.systems.robotHardware.Hardware;
import org.firstinspires.ftc.teamcode.trajectories.roadRunnerConfigurations.MecanumDrive;
import org.firstinspires.ftc.teamcode.trajectories.roadRunnerConfigurations.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.systems.arm.JacobianArm;
import java.util.List;
import org.firstinspires.ftc.teamcode.systems.arm.Positions;

/**
 * TeleOp mode for controlling the robot arm using predefined positions via gamepad input.
 */
@TeleOp(name = "DriveBase-TeleOpPointToPoint", group = "Dev-Teleops")
public class TeleopArm extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the JacobianArm system using the hardware map
        JacobianArm arm = new JacobianArm(hardwareMap);

        // Get reference to CRServo used for intake mechanism
        CRServo intake = hardwareMap.get(CRServo.class, "intake");

        // Wait for the driver to press PLAY
        waitForStart();

        // Load predefined arm positions from Positions class
        Positions.ArmPosition basketPos = Positions.getBasket();
        Positions.ArmPosition perimeterPos = Positions.getPerimeter();
        Positions.ArmPosition specimenUpPos = Positions.getSpecimenUp();
        Positions.ArmPosition specimenDownPos = Positions.getSpecimenDown();
        Positions.ArmPosition submersibil1 = Positions.getSubmersibil1();
        Positions.ArmPosition submersibil2 = Positions.getSubmersibil2();
        Positions.ArmPosition basket3 = Positions.getBasket3();
        Positions.ArmPosition perimeterUp = Positions.getPerimeterUP();

        // Main loop runs while OpMode is active
        while (opModeIsActive()) {

            // GAMEPAD 1: Control arm positions with D-Pad
            if (gamepad1.dpad_up) {
                arm.ArmGoto(basketPos.x, basketPos.y, basketPos.elbowUp);  // Go to basket position
            } else if (gamepad1.dpad_down) {
                arm.ArmGoto(submersibil1.x, submersibil1.y, submersibil1.elbowUp); // Submersible 1
            } else if (gamepad1.dpad_right) {
                arm.ArmGoto(submersibil2.x, submersibil2.y, submersibil2.elbowUp); // Submersible 2
            } else if (gamepad1.dpad_left) {
                arm.ArmGoto(basket3.x, basket3.y, basket3.elbowUp); // Alternate basket position
            }

            // GAMEPAD 2: Secondary operator arm controls
            if (gamepad2.dpad_down) {
                arm.ArmGoto(perimeterPos.x, perimeterPos.y, perimeterPos.elbowUp); // Perimeter low
            } else if (gamepad2.dpad_right) {
                arm.ArmGoto(specimenUpPos.x, specimenUpPos.y, specimenUpPos.elbowUp); // Specimen up
            } else if (gamepad2.dpad_left) {
                arm.ArmGoto(specimenDownPos.x, specimenDownPos.y, specimenDownPos.elbowUp); // Specimen down
            } else if (gamepad2.dpad_up) {
                arm.ArmGoto(perimeterUp.x, perimeterUp.y, perimeterUp.elbowUp); // Perimeter high
            }

            // GAMEPAD 1 bumpers: Intake control
            if (gamepad1.left_bumper) {
                intake.setPower(1.0); // Intake in
            } else if (gamepad1.right_bumper) {
                intake.setPower(-1.0); // Intake out
            } else {
                intake.setPower(0.0); // Stop intake
            }
        }
    }
}