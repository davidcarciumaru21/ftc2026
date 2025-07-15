package org.firstinspires.ftc.teamcode.presets;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * Represents a grid/table of nodes for pathfinding and robot navigation.
 * Each cell in the grid is represented by a Node.
 */
public class Table {
    // Number of rows in the grid
    public int rows;

    // Number of columns in the grid
    public int columns;

    // 2D array representing the grid of Nodes
    public Node[][] table;

    // Unit distance/movement cost, e.g., length of a single cell or step
    public int mu;

    // Random generator for selecting random valid nodes
    private Random random = new Random();

    /**
     * Constructor to initialize the grid with given rows, columns and movement unit.
     * Generates the grid and initializes each cell as a Node.
     *
     * @param rows Number of rows in the table
     * @param columns Number of columns in the table
     * @param mu Movement unit or cost associated with each grid cell
     */
    public Table(int rows, int columns, int mu) {
        this.rows = rows;
        this.columns = columns;
        this.mu = mu;
        this.table = new Node[rows][columns];
        this.generateTable();
    }

    /**
     * Initializes each cell in the grid as a new Node object with its (x,y) coordinates.
     */
    private void generateTable() {
        for (int y = 0; y < this.rows; y++) {
            for (int x = 0; x < this.columns; x++) {
                table[y][x] = new Node(x, y);
            }
        }
    }

    /**
     * Prints the grid to the console row by row.
     * Uses the Node's toString method for representation.
     */
    public void printMatrix() {
        for (int y = 0; y < this.rows; y++){
            StringBuilder rowStr = new StringBuilder();
            for (int x = 0; x < this.columns; x++){
                rowStr.append(this.table[y][x].toString());
            }
            System.out.println(rowStr);
        }
    }

    /**
     * Helper method to check if given coordinates (y, x) are within the grid bounds.
     *
     * @param y Row index
     * @param x Column index
     * @return true if coordinates are valid inside the grid, false otherwise
     */
    private boolean isInBounds(int y, int x) {
        return y >= 0 && y < rows && x >= 0 && x < columns;
    }

    /**
     * Marks the nodes along a given path as part of the path.
     * Skips the start and end nodes.
     *
     * @param path List of nodes representing a path
     */
    public void applyPathToTable(List<Node> path){
        for (int i = 1; i < path.size() - 1; i++){
            path.get(i).isPath = true;
        }
    }

    /**
     * Marks nodes that the robot cannot reach due to obstacles nearby.
     * Uses the robot's dimensions to "inflate" obstacles, marking surrounding nodes as unreachable.
     *
     * @param robot The robot whose size is used to determine unreachable nodes
     */
    public void markUnableToReachNodes(Robot robot) {
        int half_width = (int) Math.ceil(robot.width / 2.0);
        int half_height = (int) Math.ceil(robot.height / 2.0);

        for (int y = 0; y < this.rows; y++){
            for (int x = 0; x < this.columns; x++){
                if (this.table[y][x].isObstacle)  {
                    // Mark surrounding nodes as unable to reach based on robot size
                    for (int dy = -half_height; dy <= half_height; dy++){
                        for (int dx = -half_width; dx <= half_width; dx++){
                            int newY = y + dy;
                            int newX = x + dx;
                            if (this.isInBounds(newY, newX)){
                                Node neighbor = this.table[newY][newX];
                                if (!neighbor.isObstacle) {
                                    neighbor.isUnableToReach = true;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    /**
     * Returns a random valid node from the grid that is neither an obstacle nor unreachable.
     * Throws an exception if no such nodes exist.
     *
     * @return A valid Node object
     */
    public Node returnValidNode() {
        List<Node> validNodes = new ArrayList<Node>();
        for (int y = 0; y < rows; y++) {
            for (int x = 0; x < columns; x++) {
                Node node = this.table[y][x];
                if (!node.isObstacle && !node.isUnableToReach) {
                    validNodes.add(node);
                }
            }
        }

        if (validNodes.isEmpty()) {
            throw new RuntimeException("No valid nodes available on the table.");
        }

        return validNodes.get(random.nextInt(validNodes.size()));
    }

    /**
     * Marks the border nodes of the table as unreachable by the robot.
     * The border width depends on the robot's size.
     *
     * @param robot The robot used to calculate border thickness
     */
    public void drawBorder(Robot robot) {
        int half_width = (int) Math.ceil(robot.width / 2.0);
        int half_height = (int) Math.ceil(robot.height / 2.0);

        // Mark left and right borders as unreachable
        for (int y = 0; y < rows; y++) {
            for (int i = 0; i < half_width; i++) {
                this.table[y][i].isUnableToReach = true;
                this.table[y][columns - i - 1].isUnableToReach = true;
            }
        }

        // Mark top and bottom borders as unreachable
        for (int y = 0; y < half_height; y++) {
            for (int x = 0; x < columns; x++) {
                table[y][x].isUnableToReach = true;
                table[rows - y - 1][x].isUnableToReach = true;
            }
        }
    }

    /**
     * Clears the robot presence flag from all nodes in the grid.
     */
    public void clearRobot() {
        for (int y = 0; y < this.rows; y++) {
            for (int x = 0; x < this.columns; x++) {
                if (this.table[y][x].hasRobot) {
                    this.table[y][x].hasRobot = false;
                }
            }
        }
    }

    /**
     * Marks the grid nodes occupied by the robot based on its size and center coordinates.
     * Sets hasRobot to true for those nodes.
     *
     * @param robot The robot whose size defines the area to mark
     * @param coordonates The center coordinates (y, x) where the robot is placed
     */
    public void generateRobot(Robot robot, int[] coordonates) {
        int centerY = coordonates[0];
        int centerX = coordonates[1];
        int halfWidth = (int) Math.ceil(robot.width / 2.0);
        int halfHeight = (int) Math.ceil(robot.height / 2.0);
        for (int dy = -halfHeight; dy <= halfHeight; dy++) {
            for (int dx = -halfWidth; dx <= halfWidth; dx++) {
                int newY = centerY + dy;
                int newX = centerX + dx;
                if (this.isInBounds(newY, newX)) {
                    table[newY][newX].hasRobot = true;
                }
            }
        }
    }

    /**
     * Resets various states of all nodes in the grid to their default values.
     * Used to clear previous pathfinding information before a new search.
     */
    public void resetNodeStates() {
        for (int row = 0; row < rows; row++) {
            for (int col = 0; col < columns; col++) {
                table[row][col].isStart = false;
                table[row][col].isEnd = false;
                table[row][col].isPath = false;
                table[row][col].hasRobot = false;
                table[row][col].g = Double.POSITIVE_INFINITY;
                table[row][col].h = 0;
                table[row][col].f = Double.POSITIVE_INFINITY;
                table[row][col].parent = null;
            }
        }
    }
}
