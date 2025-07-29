package org.firstinspires.ftc.teamcode.Utils;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public class WaitAction implements Action {
    private final long durationMillis;
    private long startTime = -1;

    public WaitAction(double durationSeconds) {
        this.durationMillis = (long)(durationSeconds * 1000);
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        if (startTime < 0) {
            startTime = System.currentTimeMillis();
        }

        // Check if the wait duration has elapsed
        return System.currentTimeMillis() - startTime < durationMillis;
    }
}
