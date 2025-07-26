package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.systems.arm.JacobianArm;
import org.firstinspires.ftc.teamcode.systems.arm.Positions;

@TeleOp(name = "TeleopArm", group = "Dev-Teleops")
@Disabled
public class ArmTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // TYPE: INIT - Initialize arm and intake
        JacobianArm arm = new JacobianArm(hardwareMap);
        CRServo intake = hardwareMap.get(CRServo.class, "intake");

        // TYPE: INIT - Load predefined arm positions
        Positions.ArmPosition basketPos = Positions.getBasket();
        Positions.ArmPosition perimeterPos = Positions.getPerimeter();
        Positions.ArmPosition specimenUpPos = Positions.getSpecimenUp();
        Positions.ArmPosition specimenDownPos = Positions.getSpecimenDown();
        Positions.ArmPosition submersibil1 = Positions.getSubmersibil1();
        Positions.ArmPosition submersibil2 = Positions.getSubmersibil2();
        Positions.ArmPosition basket3 = Positions.getBasket3();
        Positions.ArmPosition perimeterUp = Positions.getPerimeterUP();

        waitForStart();

        // TYPE: CONTROL - Runtime loop
        while(opModeIsActive()) {
            // TYPE: CONTROL - Gamepad1 D-Pad for preset arm positions
            if (gamepad1.dpad_up) {
                arm.ArmGoto(basketPos.x, basketPos.y, basketPos.elbowUp);
            } else if (gamepad1.dpad_down) {
                arm.ArmGoto(submersibil1.x, submersibil1.y, submersibil1.elbowUp);
            } else if (gamepad1.dpad_right) {
                arm.ArmGoto(submersibil2.x, submersibil2.y, submersibil2.elbowUp);
            } else if (gamepad1.dpad_left) {
                arm.ArmGoto(basket3.x, basket3.y, basket3.elbowUp);
            }

            // TYPE: CONTROL - Gamepad2 D-Pad for alternate positions
            if (gamepad2.dpad_down) {
                arm.ArmGoto(perimeterPos.x, perimeterPos.y, perimeterPos.elbowUp);
            } else if (gamepad2.dpad_right) {
                arm.ArmGoto(specimenUpPos.x, specimenUpPos.y, specimenUpPos.elbowUp);
            } else if (gamepad2.dpad_left) {
                arm.ArmGoto(specimenDownPos.x, specimenDownPos.y, specimenDownPos.elbowUp);
            } else if (gamepad2.dpad_up) {
                arm.ArmGoto(perimeterUp.x, perimeterUp.y, perimeterUp.elbowUp);
            }

            // TYPE: CONTROL - Intake control using Gamepad1 bumpers
            if (gamepad1.left_bumper) {
                intake.setPower(1.0);
            } else if (gamepad1.right_bumper) {
                intake.setPower(-1.0);
            } else {
                intake.setPower(0.0);
            }
        }
    }
}