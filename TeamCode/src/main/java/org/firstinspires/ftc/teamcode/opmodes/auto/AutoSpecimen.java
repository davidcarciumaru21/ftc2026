package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.enums.RobotInitialization;
import org.firstinspires.ftc.teamcode.utils.MeasurementUnit;
import org.firstinspires.ftc.teamcode.systems.arm.ArmAction;
import org.firstinspires.ftc.teamcode.systems.arm.JacobianArm;
import org.firstinspires.ftc.teamcode.systems.arm.Positions;
import org.firstinspires.ftc.teamcode.systems.servo.ServoAction;
import org.firstinspires.ftc.teamcode.roadRunner.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunner.localizer.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.systems.robotHardware.Hardware;

import com.qualcomm.robotcore.hardware.TouchSensor;

@Autonomous(group = "Auto", name = "Auto Specimen")
public class AutoSpecimen extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // --- Initial robot pose (0 cm, 0 cm, 0° heading) ---
        double startX = MeasurementUnit.cmToInches(0);
        double startY = MeasurementUnit.cmToInches(0);
        double startHeading = Math.toRadians(0);
        Pose2d startPose = new Pose2d(startX, startY, startHeading);
        Hardware robot = new Hardware();
        // --- Initialize Road Runner drive and localization ---
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        ThreeDeadWheelLocalizer driveLocalizer = new ThreeDeadWheelLocalizer(
                hardwareMap,
                MecanumDrive.PARAMS.inPerTick,
                startPose
        );
        driveLocalizer.update();
        Pose2d currentPose = driveLocalizer.getPose();
        MovementAuto Movement = new MovementAuto(hardwareMap);

        // --- Initialize arm and servo systems ---
        ServoAction servo = new ServoAction(hardwareMap);
        JacobianArm arm = new JacobianArm(hardwareMap);
        ArmAction armAction = new ArmAction(arm);

        TouchSensor TSL, TSR;
        TSL = hardwareMap.get(TouchSensor.class, "TSL");
        TSR = hardwareMap.get(TouchSensor.class, "TSR");

        waitForStart();

        robot.init(hardwareMap, RobotInitialization.WithRoadRunner);

        Positions.ArmPosition perimeterUpPos = Positions.getPerimeterUP();
        Positions.ArmPosition specimenDownPos = Positions.getSpecimenDownAuto();
        Positions.ArmPosition startPos = Positions.start();
        Positions.ArmPosition specimenUpPos = Positions.getSpecimenUp();
        Positions.ArmPosition perimeterPos = Positions.getPerimeter();
        Positions.ArmPosition submersibil2Pos = Positions.getSubmersibil2();

        Action start = armAction.ArmGoto(startPos.x, startPos.y, startPos.elbowUp);
        Action perimeter = armAction.ArmGoto(perimeterPos.x, perimeterPos.y, perimeterPos.elbowUp);
        Action perimeterUp = armAction.ArmGoto(perimeterUpPos.x, perimeterUpPos.y, perimeterUpPos.elbowUp);
        Action submersibil2 = armAction.ArmGoto(submersibil2Pos.x, submersibil2Pos.y, submersibil2Pos.elbowUp);
        Action specimenDown = armAction.ArmGoto(specimenDownPos.x, specimenDownPos.y, specimenDownPos.elbowUp);
        Action specimenUp = armAction.ArmGoto(specimenUpPos.x, specimenUpPos.y, specimenUpPos.elbowUp);

        Action servoOut = servo.setPower(1.0);  // Activate servo to eject game element
        Action servoStop = servo.setPower(0.0); // Stop the servo after ejection
        Action servoIn = servo.setPower(-1.0);

        arm.resetArm();

        // Primul specimen(deja pe robot)
        while(opModeIsActive()) {
            try {
                Action stafeToSubmersible1 = Movement.strafe(74, 35, driveLocalizer);

                Actions.runBlocking(new ParallelAction(
                        stafeToSubmersible1,
                        specimenUp
                ));


                while (!TSL.isPressed() && !TSR.isPressed()) {
                    robot.backLeftMotor.setPower(0.5);
                    robot.frontLeftMotor.setPower(0.5);
                    robot.backRightMotor.setPower(0.5);
                    robot.frontRightMotor.setPower(0.5);
                }


                Actions.runBlocking(specimenDown);

                sleep(500);

                Action back = Movement.straight(-50, driveLocalizer);
                Actions.runBlocking(back);

                Action resetheading = Movement.turnTo(0, driveLocalizer);

                // Al doilea sample
                Action strafeToSample1 = Movement.strafe(0, -85, driveLocalizer);
                Actions.runBlocking(new SequentialAction(
                        resetheading,
                        strafeToSample1
                ));

                resetheading = Movement.turnTo(0, driveLocalizer);
                Action straightToSample = Movement.straight(100, driveLocalizer);
                Actions.runBlocking(new SequentialAction(
                        resetheading,
                        straightToSample
                ));

                Action rightToSample = Movement.strafe(0, -25, driveLocalizer);
                Actions.runBlocking(rightToSample);

                resetheading = Movement.turnTo(0, driveLocalizer);
                Action downToBase = Movement.straight(-110, driveLocalizer);
                Actions.runBlocking(new SequentialAction(
                        resetheading,
                        downToBase
                ));

                Action exitBase = Movement.straight(20, driveLocalizer, 50, 50);
                Actions.runBlocking(exitBase);
                Action reverseToSpecimen = Movement.turnTo(180, driveLocalizer);
                Actions.runBlocking(new ParallelAction(
                        reverseToSpecimen,
                        perimeter
                ));
                driveLocalizer.update();
                currentPose = driveLocalizer.getPose();

                double headingError = 180 - Math.toDegrees(currentPose.heading.toDouble());
                if (headingError > 0 && headingError < 10) {
                    Action turnAction = Movement.turn(headingError + 2, driveLocalizer, 30, 30);
                    Actions.runBlocking(turnAction);
                }


                while (!TSL.isPressed() && !TSR.isPressed()) {
                    robot.backLeftMotor.setPower(0.5);
                    robot.frontLeftMotor.setPower(0.5);
                    robot.backRightMotor.setPower(0.5);
                    robot.frontRightMotor.setPower(0.5);
                }

                sleep(500);

                Actions.runBlocking(perimeterUp);

                Action backOut = Movement.straight(50, driveLocalizer);
                Actions.runBlocking(backOut);

                Action splineToSubmersible = Movement.spline(0.0, 125.0, driveLocalizer, 0);
                Actions.runBlocking(new ParallelAction(
                        splineToSubmersible,
                        specimenUp
                ));

                sleep(500);

                while (!TSL.isPressed() && !TSR.isPressed()) {
                    robot.backLeftMotor.setPower(0.4);
                    robot.frontLeftMotor.setPower(0.4);
                    robot.backRightMotor.setPower(0.4);
                    robot.frontRightMotor.setPower(0.4);
                }

                Actions.runBlocking(specimenDown);

                Action backToBase = Movement.strafe(-50, -140, driveLocalizer);
                Actions.runBlocking(new SequentialAction(
                        backToBase,
                        start
                ));
                requestOpModeStop();
            }
            catch(Exception e){
                terminateOpModeNow();
            }
        }
    }
}