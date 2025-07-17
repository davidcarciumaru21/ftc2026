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
            boolean initialized = false;
            boolean valid = true;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (!initialized) {
                    valid = arm.ArmGoto(x, y, elbowUp);
                    initialized = true;
                }

                if (!valid) {
                    return true; // invalid target, abort immediately
                }

                return !arm.isMoving(); // finish action when arm is done moving
            }
        };
    }
}
