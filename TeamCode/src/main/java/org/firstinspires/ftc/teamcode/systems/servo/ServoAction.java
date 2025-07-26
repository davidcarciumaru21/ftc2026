package org.firstinspires.ftc.teamcode.systems.servo;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ServoAction {

    private final HardwareMap hardwareMap;
    CRServo intake;

    public ServoAction(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        intake = hardwareMap.get(CRServo.class, "intake");
    }
    public Action setPower(double power) {
        return new Action() {

            @Override
            public boolean run(TelemetryPacket packet) {
                intake.setPower(-power);

                // Stop immediately after setting power once
                return false;
            }
        };
    }
}
