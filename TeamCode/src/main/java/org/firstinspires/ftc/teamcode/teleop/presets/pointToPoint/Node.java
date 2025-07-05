package org.firstinspires.ftc.teamcode.teleop.presets.pointToPoint;

public class Node {

    public int x, y;
    public int[] coordonates;
    public double g = Double.POSITIVE_INFINITY;
    public double h = 0;
    public double f = Double.POSITIVE_INFINITY;
    public Node parent = null;

    public boolean isObstacle = false;
    public boolean isEnd = false;
    public boolean isStart = false;
    public boolean isPath = false;
    public boolean isUnableToReach = false;
    public boolean hasRobot = false;

    public Node(int x, int y){
        this.x = x;
        this.y = y;

        this.coordonates = new int[] {this.x, this.y};
    }

    @Override
    public String toString() {
        if (hasRobot) return "\u001B[35mR\u001B[0m";        // Purple (Magenta)
        else if (isObstacle) return "\u001B[34mO\u001B[0m"; // Blue
        else if (isUnableToReach) return "\u001B[33mU\u001B[0m"; // Yellow
        else if (isStart) return "\u001B[32mS\u001B[0m";    // Green
        else if (isEnd) return "\u001B[32mE\u001B[0m";      // Green (same as start)
        else if (isPath) return "\u001B[31mP\u001B[0m";     // Red
        else return "\u001B[37m0\u001B[0m";                 // White (default)
    }
}