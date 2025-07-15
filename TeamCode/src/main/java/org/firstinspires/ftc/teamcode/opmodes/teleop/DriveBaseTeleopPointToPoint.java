package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.trajectories.MecanumDrive.PARAMS;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utils.Utils;
import org.firstinspires.ftc.teamcode.presets.Node;
import org.firstinspires.ftc.teamcode.presets.Table;
import org.firstinspires.ftc.teamcode.presets.Robot;
import org.firstinspires.ftc.teamcode.presets.PointToPoint;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.robotHardware.Hardware;
import org.firstinspires.ftc.teamcode.trajectories.MecanumDrive;
import org.firstinspires.ftc.teamcode.trajectories.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.systems.arm.JacobianArm;
import java.util.List;
import org.firstinspires.ftc.teamcode.systems.arm.Positions;
@TeleOp(name = "DriveBase-TeleOpPointToPoint", group = "Dev-Teleops")
public class DriveBaseTeleopPointToPoint extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        JacobianArm arm = new JacobianArm(hardwareMap);
        CRServo intake = hardwareMap.get(CRServo.class, "intake");
        Table tableObj = new Table(100, 100, 1);
        Robot robotPoinToPoint = new Robot((int)Utils.cmToInches(35.0), (int)Utils.cmToInches(43.0));
        double startX = robotPoinToPoint.height / 2, startY = robotPoinToPoint.width / 2;
        double startHeading = Math.toDegrees(0);
        Pose2d startPose = new Pose2d(startX, startY, startHeading);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        ThreeDeadWheelLocalizer driveLocalizer = new ThreeDeadWheelLocalizer(hardwareMap, PARAMS.inPerTick, startPose);
        boolean currentShareStateGamepad1;
        boolean lastShareGamepad1 = false;
        boolean currentShareStateGamepad2;
        boolean lastShareGamepad2 = false;
        byte driveModeGamepad1 = 1, driveModeGamepad2 = 1;
        tableObj.drawBorder(robotPoinToPoint);
        tableObj.markUnableToReachNodes(robotPoinToPoint);

        boolean lastBState = false;
        boolean currentBState;
        boolean lastCircle = false;

        // Create Hardware instance
        final Hardware robotHardware = new Hardware();

        // Initialize hardware map
        robotHardware.init(hardwareMap, 2);

        // Coefficients for joystick inputs
        double coefX = 1.1;
        double coefY = 1.0;
        double coefRx = 1.0;
        int xNode = 0, yNode = 0;
        Node currentNode = null;
        boolean buzy = false;

        Node endNode = tableObj.table[33][47];

        List<Node> path;

        double x, y, rx;
        double denominator = 0;
        double frontLeftPower, backLeftPower, frontRightPower, backRightPower;
        double coefXGamepad1 = 1.0;
        double coefYGamepad1 = 1.0;
        double coefRxGamepad1 = 1.0;

        double coefXGamepad2 = 1.0;
        double coefYGamepad2 = 1.0;
        double coefRxGamepad2 = 1.0;


        double botHeading, rotX, rotY;

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN));
        imu.initialize(parameters);
        waitForStart();
        Positions.ArmPosition basketPos = Positions.getBasket();
        Positions.ArmPosition perimeterPos = Positions.getPerimeter();
        Positions.ArmPosition specimenUpPos = Positions.getSpecimenUp();
        Positions.ArmPosition specimenDownPos = Positions.getSpecimenDown();
        Positions.ArmPosition submersibil1 = Positions.getSubmersibil1();
        Positions.ArmPosition submersibil2 = Positions.getSubmersibil2();
        Positions.ArmPosition basket3 = Positions.getBasket3();
        Positions.ArmPosition perimeterUp = Positions.getPerimeterUP();

        while(opModeIsActive()) {
            driveLocalizer.update();
            Pose2d currentPose = driveLocalizer.getPose();
            try {
                yNode = 2 * (int) startX - (int) Math.round(currentPose.position.x);
                xNode = (int) Math.round(currentPose.position.y);
                currentNode = tableObj.table[yNode][xNode];
            } catch(Exception e){
                telemetry.addData("Error: ", "Out-of-bounds");
                telemetry.update();
            }


            currentBState = gamepad1.circle;

            if (currentBState && !lastBState) {
                buzy = true;
                try {
                    currentNode.isStart = true;
                    endNode.isEnd = true;
                    tableObj.resetNodeStates();
                    tableObj.drawBorder(robotPoinToPoint);
                    tableObj.markUnableToReachNodes(robotPoinToPoint);
                    tableObj.generateRobot(robotPoinToPoint, new int[] {yNode, xNode});

                    path = PointToPoint.aStar(tableObj, currentNode, endNode, robotPoinToPoint);
                    tableObj.applyPathToTable(path);
                    List<String> moves = PointToPoint.pathToCommands(path, tableObj);
                    List<String> compressedMoves = PointToPoint.compressPath(moves);
                    PointToPoint.executePath(compressedMoves, drive, currentPose);

                } catch(Exception e) {
                    telemetry.addLine("Unable to get to the point");
                    telemetry.update();
                }
                buzy = false;
            }
            // GAMEPAD 1 controls
            if (gamepad1.dpad_up) {
                arm.ArmGoto(basketPos.x, basketPos.y, basketPos.elbowUp);
            } else if (gamepad1.dpad_down) {
                arm.ArmGoto(submersibil1.x, submersibil1.y, submersibil1.elbowUp);
            } else if (gamepad1.dpad_right) {
                arm.ArmGoto(submersibil2.x, submersibil2.y, submersibil2.elbowUp);
            } else if (gamepad1.dpad_left) {
                arm.ArmGoto(basket3.x, basket3.y, basket3.elbowUp);
            }

            // GAMEPAD 2 controls
            if (gamepad2.dpad_down) {
                arm.ArmGoto(perimeterPos.x, perimeterPos.y, perimeterPos.elbowUp);
            } else if (gamepad2.dpad_right) {
                arm.ArmGoto(specimenUpPos.x, specimenUpPos.y, specimenUpPos.elbowUp);
            } else if (gamepad2.dpad_left) {
                arm.ArmGoto(specimenDownPos.x, specimenDownPos.y, specimenDownPos.elbowUp);
            } else if (gamepad2.dpad_up) {
                arm.ArmGoto(perimeterUp.x, perimeterUp.y, perimeterUp.elbowUp);
            }
            if (gamepad1.left_bumper) {
                intake.setPower(1.0);
            } else if (gamepad1.right_bumper) {
                intake.setPower(-1.0);
            } else {
                intake.setPower(0.0);
            }
            if(buzy == false) {
                // Read joystick inputs and apply coefficients
                x = gamepad1.left_stick_x * coefX + gamepad2.left_stick_x * coefX;
                y = -gamepad1.left_stick_y * coefY + -gamepad2.left_stick_y * coefY;
                rx = gamepad1.right_stick_x * coefRx + gamepad2.right_stick_x * coefRx;


                // Normalize denominator to keep power in [-1,1]
                denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

                // Calculate motor powers
                frontLeftPower = (y + x + rx) / denominator;
                backLeftPower = (y - x + rx) / denominator;
                frontRightPower = (y - x - rx) / denominator;
                backRightPower = (y + x - rx) / denominator;

                // Set motor powers
                robotHardware.frontLeftMotor.setPower(frontLeftPower);
                robotHardware.backLeftMotor.setPower(backLeftPower);
                robotHardware.frontRightMotor.setPower(frontRightPower);
                robotHardware.backRightMotor.setPower(backRightPower);
            }
            if(gamepad1.right_trigger > 0.2 || gamepad2.right_trigger > 0.2){
                coefX = 0.5;
                coefY = 0.5;
                coefRx = 0.5;
            }
            else if(gamepad1.left_trigger > 0.2 || gamepad2.left_trigger > 0.2){
                coefX = 0.25;
                coefY = 0.25;
                coefRx = 0.25;
            }
            else{
                coefX = 1.0;
                coefY = 1.0;
                coefRx = 1.0;
            }


            telemetry.addData("x", currentPose.position.x);
            telemetry.addData("y", currentPose.position.y);
            telemetry.addData("NodeX", xNode);
            telemetry.addData("Nodey", yNode);
            telemetry.update();
            lastBState = currentBState;
        }
    }
}