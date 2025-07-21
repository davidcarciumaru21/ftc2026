package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.text.method.Touch;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Utils.MeasurementUnit;
import org.firstinspires.ftc.teamcode.systems.arm.ArmAction;
import org.firstinspires.ftc.teamcode.systems.arm.JacobianArm;
import org.firstinspires.ftc.teamcode.systems.arm.Positions;
import org.firstinspires.ftc.teamcode.systems.servo.ServoAction;
import org.firstinspires.ftc.teamcode.trajectories.roadRunnerConfigurations.MecanumDrive;
import org.firstinspires.ftc.teamcode.trajectories.roadRunnerConfigurations.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.systems.robotHardware.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Autonomous(group = "Test", name = "Auto Specimen")
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

        robot.init(hardwareMap, (byte) 2);

        Positions.ArmPosition perimeterUpPos = Positions.getPerimeterUP();
        Positions.ArmPosition specimenDownPos = Positions.getSpecimenDown();
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


        // Primul specimen(deja pe robot)

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

        Action straightToSample = Movement.straight(100, driveLocalizer);
        Actions.runBlocking(straightToSample);

        Action rightToSample = Movement.strafe(0, -20, driveLocalizer);
        Actions.runBlocking(rightToSample);

        Action downToBase = Movement.straight(-110, driveLocalizer);
        Actions.runBlocking(downToBase);

        // Al treilea sample

        Action straightToSample2 = Movement.straight(110, driveLocalizer);
        Actions.runBlocking(straightToSample2);

        Action rightToSample2 = Movement.strafe(0, -20, driveLocalizer);
        Actions.runBlocking(rightToSample2);

        Action downToBase2 = Movement.straight(-110, driveLocalizer);
        Actions.runBlocking(downToBase2);

        //Al patrulea sample

        Action straightToSample3 = Movement.straight(110, driveLocalizer);
        Actions.runBlocking(straightToSample3);

        Action rightToSample3 = Movement.strafe(0, -20, driveLocalizer);
        Actions.runBlocking(rightToSample3);

        Action downToBase3 = Movement.straight(-110, driveLocalizer);
        Actions.runBlocking(downToBase3);

        Action forwardOutOfBase = Movement.straight(20, driveLocalizer);
        Actions.runBlocking(new ParallelAction(
                forwardOutOfBase,
                start
        ));

    }
}