package org.firstinspires.ftc.teamcode.systems.colorSensor;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.systems.servo.ServoAction;

public class ColorAction {

    private final SampleDetection color;
    private final ServoAction servo;

    public ColorAction(SampleDetection color, ServoAction servo) {
        this.color = color;
        this.servo = servo;
    }

    public Action goUntilSampleDropped(double minDistance) {
        return new Action() {

            private long startTime = -1;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (startTime == -1) {
                    // Record start time in milliseconds
                    startTime = System.currentTimeMillis();
                }

                boolean detected = color.checkColorAndDistance(minDistance);
                long elapsed = System.currentTimeMillis() - startTime;

                if (detected || elapsed >= 1000) {
                    servo.setPower(0.0); // Stop servo
                    return false; // Action is done
                } else {
                    servo.setPower(-1.0); // Keep spinning
                    return true; // Action still running
                }
            }
        };
    }
}
