package org.firstinspires.ftc.teamcode.presets;

/**
 * Represents a node (cell) in a grid-based pathfinding map.
 * Used for A* algorithm or similar grid navigation strategies.
 */
public class Node {

    // Grid coordinates of the node
    public int x, y;

    // Cached coordinates as an array for quick access
    public int[] coordonates;

    // Cost from start node to this node
    public double g = Double.POSITIVE_INFINITY;

    // Heuristic estimate of cost from this node to end node
    public double h = 0;

    // Total estimated cost (f = g + h)
    public double f = Double.POSITIVE_INFINITY;

    // Reference to the parent node (used to reconstruct path)
    public Node parent = null;

    // Flags for node state
    public boolean isObstacle = false;        // True if node is not traversable
    public boolean isEnd = false;             // True if this is the target node
    public boolean isStart = false;           // True if this is the starting node
    public boolean isPath = false;            // True if node is part of the final path
    public boolean isUnableToReach = false;   // True if node is marked as unreachable
    public boolean hasRobot = false;          // True if robot is currently on this node

    /**
     * Constructor to create a node at (x, y) position.
     * @param x Horizontal coordinate in grid
     * @param y Vertical coordinate in grid
     */
    public Node(int x, int y) {
        this.x = x;
        this.y = y;
        this.coordonates = new int[] {this.x, this.y};
    }

    /**
     * Returns a string representing the node's state.
     * Uses ANSI color codes to visually distinguish node types in console output.
     */
    @Override
    public String toString() {
        if (hasRobot) return "\u001B[35mR\u001B[0m";        // Magenta: Robot position
        else if (isObstacle) return "\u001B[34mO\u001B[0m"; // Blue: Obstacle
        else if (isUnableToReach) return "\u001B[33mU\u001B[0m"; // Yellow: Unreachable
        else if (isStart) return "\u001B[32mS\u001B[0m";    // Green: Start
        else if (isEnd) return "\u001B[32mE\u001B[0m";      // Green: End
        else if (isPath) return "\u001B[31mP\u001B[0m";     // Red: Path
        else return "\u001B[37m0\u001B[0m";                 // White: Default (empty)
    }
}
