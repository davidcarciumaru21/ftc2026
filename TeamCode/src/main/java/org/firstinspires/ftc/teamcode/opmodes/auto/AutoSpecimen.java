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

        Action stafeToSubmersible1 = Movement.strafe(74, 45, driveLocalizer);

        Actions.runBlocking(new ParallelAction(
                stafeToSubmersible1,
                specimenUp
        ));


        robot.backLeftMotor.setPower(1);
        robot.frontLeftMotor.setPower(1);
        robot.backRightMotor.setPower(1);
        robot.frontRightMotor.setPower(1);

        Action back = Movement.straight(-30, driveLocalizer);
        Actions.runBlocking(new SequentialAction(
                specimenDown,
                back
        ));

        robot.backLeftMotor.setPower(0);
        robot.frontLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        robot.frontRightMotor.setPower(0);

        // Al doilea specimen

        Action turnToPick = Movement.turnTo(-81, driveLocalizer);
        Actions.runBlocking(turnToPick);


        Action forwardToPick = Movement.straight(30, driveLocalizer);
        Actions.runBlocking(new SequentialAction(
                submersibil2,
                new ParallelAction(
                        servoIn,
                        forwardToPick
                )));
    }
}