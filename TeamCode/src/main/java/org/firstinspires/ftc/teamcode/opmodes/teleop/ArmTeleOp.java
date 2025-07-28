package org.firstinspires.ftc.teamcode.opmodes.teleop;

//==============================Robot Core=============================
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

//=============================Robot Systems===========================
import org.firstinspires.ftc.teamcode.Utils.TelemetryMethods;
import org.firstinspires.ftc.teamcode.config.ColorConfig;
import org.firstinspires.ftc.teamcode.systems.arm.JacobianArm;
import org.firstinspires.ftc.teamcode.systems.arm.Positions;
import org.firstinspires.ftc.teamcode.systems.colorSensor.SampleDetection;

//=================================Utils===============================
import org.firstinspires.ftc.teamcode.Utils.Gamepads;

@TeleOp(name = "TeleopArm", group = "Dev-Teleops")
public class ArmTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //================Arm and intake initialization=================
        JacobianArm arm = new JacobianArm(hardwareMap);

        double servoPower;
        CRServo intake = hardwareMap.get(CRServo.class, "intake");

        SampleDetection sampleDetector = new SampleDetection(hardwareMap);

        // Wait for the start button
        waitForStart();

        //========================Arm positions=========================
        Positions.ArmPosition basketPos = Positions.getBasket();
        Positions.ArmPosition perimeterPos = Positions.getPerimeter();
        Positions.ArmPosition specimenUpPos = Positions.getSpecimenUp();
        Positions.ArmPosition specimenDownPos = Positions.getSpecimenDown();
        Positions.ArmPosition submersibil1 = Positions.getSubmersibil1();
        Positions.ArmPosition submersibil2 = Positions.getSubmersibil2();
        Positions.ArmPosition basket3 = Positions.getBasket3();
        Positions.ArmPosition perimeterUp = Positions.getPerimeterUP();

        // Main control loop
        while (opModeIsActive()) {
            //=============================================================
            //========================ARM MOVEMENT=========================
            //=============================================================

            //=====================Arm positions gamepad1==================

            if (gamepad1.dpad_up) {
                arm.ArmGoto(basketPos.x, basketPos.y, basketPos.elbowUp);
            } else if (gamepad1.dpad_down) {
                arm.ArmGoto(submersibil1.x, submersibil1.y, submersibil1.elbowUp);
            } else if (gamepad1.dpad_right) {
                arm.ArmGoto(submersibil2.x, submersibil2.y, submersibil2.elbowUp);
            } else if (gamepad1.dpad_left) {
                arm.ArmGoto(basket3.x, basket3.y, basket3.elbowUp);
            }

            //=====================Arm positions gamepad2==================
            if (gamepad2.dpad_down) {
                arm.ArmGoto(perimeterPos.x, perimeterPos.y, perimeterPos.elbowUp);
            } else if (gamepad2.dpad_right) {
                arm.ArmGoto(specimenUpPos.x, specimenUpPos.y, specimenUpPos.elbowUp);
            } else if (gamepad2.dpad_left) {
                arm.ArmGoto(specimenDownPos.x, specimenDownPos.y, specimenDownPos.elbowUp);
            } else if (gamepad2.dpad_up) {
                arm.ArmGoto(perimeterUp.x, perimeterUp.y, perimeterUp.elbowUp);
            }

            //=============================================================
            //========================INTAKE ACTIONS=======================
            //=============================================================

            //========================Servo control========================

            servoPower = 0.0;

            if (gamepad1.left_bumper) {
                servoPower = -1.0;
            } else if (gamepad1.right_bumper) {
                servoPower = 1.0;
            }
            if (gamepad2.left_bumper) {
                servoPower = -1.0;
            } else if (gamepad2.right_bumper) {
                servoPower = 1.0;
            }

            intake.setPower(servoPower);

            //=============================================================
            //==========================TELEMETRY==========================
            //=============================================================

            TelemetryMethods.displayAlliance(telemetry, ColorConfig.alliance);
            TelemetryMethods.displaySampleValidation(telemetry, sampleDetector.checkColor());
            TelemetryMethods.displayCodeVersion(telemetry, "7.29.25.2.33");
            telemetry.addLine("-----------------------------");
            telemetry.update();
        }
    }
}