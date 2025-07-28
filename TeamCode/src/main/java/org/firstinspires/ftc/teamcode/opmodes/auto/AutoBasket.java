package org.firstinspires.ftc.teamcode.opmodes.auto;

// FTC and Road Runner imports
import android.graphics.Color;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;

// Hardware/system imports
import org.firstinspires.ftc.teamcode.config.enums.RobotInitialization;
import org.firstinspires.ftc.teamcode.systems.robotHardware.Hardware;
import org.firstinspires.ftc.teamcode.utils.MeasurementUnit;
import org.firstinspires.ftc.teamcode.roadRunner.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunner.localizer.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.systems.arm.ArmAction;
import org.firstinspires.ftc.teamcode.systems.arm.JacobianArm;
import org.firstinspires.ftc.teamcode.systems.arm.Positions;
import org.firstinspires.ftc.teamcode.systems.servo.ServoAction;
import org.firstinspires.ftc.teamcode.systems.colorSensor.SampleDetection;
import org.firstinspires.ftc.teamcode.systems.colorSensor.ColorAction;

@Autonomous(name = "Auto Basket", group = "Auto")
public class AutoBasket extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // --- Initial robot pose (0 cm, 0 cm, 0° heading) ---
        double startX = MeasurementUnit.cmToInches(0);
        double startY = MeasurementUnit.cmToInches(0);
        double startHeading = Math.toRadians(0);
        Pose2d startPose = new Pose2d(startX, startY, startHeading);

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
        SampleDetection SampleD = new SampleDetection(hardwareMap);
        ColorAction color = new ColorAction(SampleD, servo);
        arm.resetArm();

        TouchSensor TSL, TSR;
        TSL = hardwareMap.get(TouchSensor.class, "TSL");
        TSR = hardwareMap.get(TouchSensor.class, "TSR");
        Hardware robot = new Hardware();
        robot.init(hardwareMap, RobotInitialization.WithRoadRunner);

        // --- Wait for the start of the match ---
        waitForStart();


        // --- Predefined arm positions ---
        Positions.ArmPosition basketPos = Positions.getBasket(); // Position for scoring
        Positions.ArmPosition startPos = Positions.start();      // Initial/resting position
        Positions.ArmPosition submersibil2Pos = Positions.getSubmersibil2();
        Positions.ArmPosition perimeterUpPos = Positions.getPerimeterUP();
        // --- Movement and mechanism actions ---

        Action basket = armAction.ArmGoto(basketPos.x, basketPos.y, basketPos.elbowUp); // Move arm to scoring position
        Action start = armAction.ArmGoto(startPos.x, startPos.y, startPos.elbowUp);     // Move arm back to start
        Action submersibil2 = armAction.ArmGoto(submersibil2Pos.x, submersibil2Pos.y, submersibil2Pos.elbowUp);
        Action perimeterUp = armAction.ArmGoto(perimeterUpPos.x, perimeterUpPos.y, perimeterUpPos.elbowUp);

        Action servoOut = servo.setPower(1.0);  // Activate servo to take game element
        Action servoStop = servo.setPower(0.0); // Stop the servo after ejection
        Action servoIn = servo.setPower(-1.0); // Activate servo to eject game element

        Action servoUntilDropped = color.goUntilSampleDropped(5);

        // Punem cel de-al zerolea cub
        while(opModeIsActive()) {
            try {
                // --- Drive to target zone using a spline path ---
                Action splineToDrop = Movement.spline(30, 50, driveLocalizer, -50); // Spline path to target area
                Actions.runBlocking(new ParallelAction(
                        splineToDrop,
                        basket
                ));


                Action back = Movement.straight(-10, driveLocalizer);
                Actions.runBlocking(new SequentialAction(
                        back,
                        servoOut
                ));

                sleep(1000);

                Actions.runBlocking(servoStop);

                double time1 = getRuntime();

                // First sample

                Action backToPositon = Movement.straight(6, driveLocalizer);
                Actions.runBlocking(new ParallelAction(
                        backToPositon,
                        submersibil2
                ));


                Action turnPick = Movement.turnTo(4, driveLocalizer, 50, 50);
                Actions.runBlocking(new ParallelAction(
                        turnPick,
                        servoIn
                ));
                robot.backLeftMotor.setPower(0.2);
                robot.frontLeftMotor.setPower(0.2);
                robot.backRightMotor.setPower(0.2);
                robot.frontRightMotor.setPower(0.2);

                sleep(200);



                Action turnBack = Movement.turnTo(-50, driveLocalizer);
                Actions.runBlocking(new ParallelAction(
                        turnBack,
                        servoStop
                ));


                Actions.runBlocking(basket);

                Action backdrop = Movement.straight(-8, driveLocalizer);
                Actions.runBlocking(new SequentialAction(
                        backdrop,
                        servoOut
                ));



                sleep(500);

                Actions.runBlocking(servoStop);

                double time2 = getRuntime();

                // Al doilea sample

                Action forwardPick = Movement.straight(3.5, driveLocalizer);

                Actions.runBlocking(new ParallelAction(
                        forwardPick,
                        submersibil2
                ));

                Action turnPick2 = Movement.turnTo(24, driveLocalizer, 75, 75);
                Actions.runBlocking(new ParallelAction(
                        turnPick2,
                        servoIn
                ));

                robot.backLeftMotor.setPower(0.2);
                robot.frontLeftMotor.setPower(0.2);
                robot.backRightMotor.setPower(0.2);
                robot.frontRightMotor.setPower(0.2);

                double time3 = getRuntime();

                Actions.runBlocking(servoStop);

                Action backToDrop = Movement.turnTo(-45, driveLocalizer);
                Actions.runBlocking(new ParallelAction(
                        backToDrop,
                        basket
                ));

                Action backToDrop2 = Movement.straight(-5, driveLocalizer);
                Actions.runBlocking(new SequentialAction(
                        backToDrop2,
                        servoOut
                ));

                sleep(500);

                Actions.runBlocking(new SequentialAction(
                        servoStop,
                        submersibil2
                ));

                // Al treilea sample

                Action forwardPick2 = Movement.straight(6, driveLocalizer);
                Actions.runBlocking(forwardPick2);

                Action turnPick3 = Movement.turnTo(40, driveLocalizer, 75, 75);
                Actions.runBlocking(new ParallelAction(
                        turnPick3,
                        servoIn
                ));

                Action turnBack2 = Movement.turnTo(-60, driveLocalizer);
                Actions.runBlocking(new ParallelAction(
                        servoStop,
                        turnBack2
                ));

                Actions.runBlocking(basket);

                Action backDrop2 = Movement.straight(-12, driveLocalizer);
                Actions.runBlocking(new SequentialAction(
                        backDrop2,
                        servoOut
                ));

                sleep(500);

                Action turnToSubmersible = Movement.turnTo(-90, driveLocalizer);
                Actions.runBlocking(new ParallelAction(
                        servoStop,
                        turnToSubmersible
                ));



                Action strafeToSubmersible = Movement.strafe(110, -80, driveLocalizer);

                Actions.runBlocking(new ParallelAction(
                        strafeToSubmersible,
                        perimeterUp
                ));

                while (!TSL.isPressed() && !TSR.isPressed()) {
                    robot.backLeftMotor.setPower(0.5);
                    robot.frontLeftMotor.setPower(0.5);
                    robot.backRightMotor.setPower(0.5);
                    robot.frontRightMotor.setPower(0.5);
                }

                double timeNew = getRuntime();



                requestOpModeStop();
            }
            catch(Exception e){
                terminateOpModeNow();
            }
        }
    }
}