package org.firstinspires.ftc.teamcode.presets;

/**
 * Represents a Robot with width and height dimensions.
 * These dimensions can be used for path planning, collision detection,
 * or other calculations involving the robot's size on the field.
 */
public class Robot {
    // Width of the robot (e.g., in grid units or centimeters)
    public int width;

    // Height of the robot (e.g., in grid units or centimeters)
    public int height;

    /**
     * Constructor to create a Robot instance with specified width and height.
     *
     * @param width The width dimension of the robot.
     * @param height The height dimension of the robot.
     */
    public Robot(int width, int height) {
        this.width = width;
        this.height = height;
    }
}
