package org.firstinspires.ftc.teamcode.systems.arm;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public class ArmAction {
    private final JacobianArm arm;

    public ArmAction(JacobianArm arm) {
        this.arm = arm;
    }

    public Action ArmGoto(double x, double y, boolean elbowUp) {
        return new Action() {
            boolean valid = true;

            @Override
            public boolean run(TelemetryPacket packet) {
                arm.ArmGoto(x, y, elbowUp);

                if (arm.isMoving()) {
                    return true;
                }
                else{
                    return false;
                }
            }
        };
    }
}
