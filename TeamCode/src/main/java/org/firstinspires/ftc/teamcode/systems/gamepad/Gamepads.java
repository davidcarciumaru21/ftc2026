package org.firstinspires.ftc.teamcode.systems.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Gamepads {

    public static void rightBumperRumble(Gamepad gamepad) {
        Gamepad.RumbleEffect effect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 100)
                .build();
        gamepad.runRumbleEffect(effect);
    }

    public static void leftBumperRumble(Gamepad gamepad) {
        Gamepad.RumbleEffect effect = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 0.0, 100)
                .build();
        gamepad.runRumbleEffect(effect);
    }
}