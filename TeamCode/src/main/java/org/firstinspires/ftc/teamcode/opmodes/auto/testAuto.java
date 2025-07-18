package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.trajectories.roadRunnerConfigurations.MecanumDrive.PARAMS;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Utils.MeasurementUnit;
import org.firstinspires.ftc.teamcode.presets.Robot;
import org.firstinspires.ftc.teamcode.trajectories.roadRunnerConfigurations.MecanumDrive;
import org.firstinspires.ftc.teamcode.systems.arm.JacobianArm;
import org.firstinspires.ftc.teamcode.systems.arm.Positions;
import org.firstinspires.ftc.teamcode.systems.arm.ArmAction;

import org.firstinspires.ftc.teamcode.opmodes.auto.Movement;
import org.firstinspires.ftc.teamcode.trajectories.roadRunnerConfigurations.ThreeDeadWheelLocalizer;

@Autonomous(name = "Auto Test", group = "Test")
public class testAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        double startX = MeasurementUnit.cmToInches(0), startY = MeasurementUnit.cmToInches(0);
        double startHeading = Math.toRadians(0);  // Facing forward
        Pose2d startPose = new Pose2d(startX, startY, startHeading);

        // Set up Road Runner drive and localization/
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        ThreeDeadWheelLocalizer driveLocalizer = new ThreeDeadWheelLocalizer(hardwareMap, PARAMS.inPerTick, startPose);
        JacobianArm arm = new JacobianArm(hardwareMap);
        ArmAction armaction = new ArmAction(arm);
        Positions.ArmPosition basketPos = Positions.getBasket();
        Positions.ArmPosition start = Positions.start();
        CRServo intake = hardwareMap.get(CRServo.class, "intake");

        waitForStart();

        driveLocalizer.update();
        Pose2d currentPose = driveLocalizer.getPose();
        Action action1 = Movement.spline(35, 50, drive, currentPose, -50);
        Actions.runBlocking(action1);

        driveLocalizer.update();
        currentPose = driveLocalizer.getPose();

        Action action2 = Movement.down(14, drive, currentPose);
        Action bratUP = armaction.ArmGoto(basketPos.x, basketPos.y, basketPos.elbowUp);

        Actions.runBlocking(new ParallelAction(action2, bratUP));

        intake.setPower(1.0);
        sleep(1000); // wait for 1 second
        intake.setPower(0.0);

        driveLocalizer.update();
        currentPose = driveLocalizer.getPose();

        Action action3 = Movement.up(14, drive, currentPose);
        Action startPos = armaction.ArmGoto(start.x, start.y, start.elbowUp);

        Actions.runBlocking(new ParallelAction(action3, startPos));



        while (opModeIsActive()){
            driveLocalizer.update();
            currentPose = driveLocalizer.getPose();
            telemetry.addData("x", MeasurementUnit.inchesToCm(currentPose.position.x));
            telemetry.addData("y", MeasurementUnit.inchesToCm(currentPose.position.y));
            telemetry.addData("heading", Math.toDegrees(currentPose.heading.toDouble()));
            telemetry.update();
        }
    }
}