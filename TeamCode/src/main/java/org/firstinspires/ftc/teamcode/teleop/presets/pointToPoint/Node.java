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
}