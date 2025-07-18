package org.firstinspires.ftc.teamcode.opmodes.auto;

// FTC and Road Runner imports
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// Hardware/system imports
import org.firstinspires.ftc.teamcode.Utils.MeasurementUnit;
import org.firstinspires.ftc.teamcode.presets.Robot;
import org.firstinspires.ftc.teamcode.trajectories.roadRunnerConfigurations.MecanumDrive;
import org.firstinspires.ftc.teamcode.trajectories.roadRunnerConfigurations.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.systems.arm.ArmAction;
import org.firstinspires.ftc.teamcode.systems.arm.JacobianArm;
import org.firstinspires.ftc.teamcode.systems.arm.Positions;
import org.firstinspires.ftc.teamcode.systems.servo.ServoAction;

@Autonomous(name = "Auto Test", group = "Test")
public class testAuto extends LinearOpMode {

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

        // --- Initialize arm and servo systems ---
        ServoAction servo = new ServoAction(hardwareMap);
        JacobianArm arm = new JacobianArm(hardwareMap);
        ArmAction armAction = new ArmAction(arm);

        // --- Predefined arm positions ---
        Positions.ArmPosition basketPos = Positions.getBasket(); // Position for scoring
        Positions.ArmPosition startPos = Positions.start();      // Initial/resting position
        Positions.ArmPosition submersibil2Pos = Positions.getSubmersibil2();
        // --- Movement and mechanism actions ---

        Action basket = armAction.ArmGoto(basketPos.x, basketPos.y, basketPos.elbowUp); // Move arm to scoring position
        Action start = armAction.ArmGoto(startPos.x, startPos.y, startPos.elbowUp);     // Move arm back to start
        Action submersibil2 = armAction.ArmGoto(submersibil2Pos.x, submersibil2Pos.y, submersibil2Pos.elbowUp);

        Action servoOut = servo.setPower(1.0);  // Activate servo to eject game element
        Action servoStop = servo.setPower(0.0); // Stop the servo after ejection
        Action servoIn = servo.setPower(-1.0);

        // --- Wait for the start of the match ---
        waitForStart();

        // --- Drive to target zone using a spline path ---
        Action splineToDrop = Movement.spline(25, 65, drive, startPose, -50); // Spline path to target area
        Actions.runBlocking(new ParallelAction(
                splineToDrop,
                basket
        ));

        // --- Run the intake/outtake servo to release the object ---
        Actions.runBlocking(servoOut);
        sleep(1000); // Allow 1 second for ejection

        // --- Lower the arm and stop the servo ---
        Actions.runBlocking(new ParallelAction(
                submersibil2,
                servoStop
        ));

        // --- Update pose estimation again ---
        driveLocalizer.update();
        currentPose = driveLocalizer.getPose();

        Action turnPick = Movement.turnTo(-15, drive, currentPose);
        Actions.runBlocking(new ParallelAction(
                turnPick,
                servoIn
        ));
    }
}
