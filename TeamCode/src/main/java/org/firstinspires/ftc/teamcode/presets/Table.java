package org.firstinspires.ftc.teamcode.presets;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class Table {
    public int rows;
    public int columns;
    public Node[][] table;
    public int mu;
    private Random random = new Random();

    public Table(int rows, int columns, int mu) {
        this.rows = rows;
        this.columns = columns;
        this.mu = mu;
        this.table = new Node[rows][columns];
        this.generateTable();
    }

    private void generateTable() {
        for (int y = 0; y < this.rows; y++) {
            for (int x = 0; x < this.columns; x++) {
                table[y][x] = new Node(x, y);
            }
        }
    }

    public void printMatrix() {
        for (int y = 0; y < this.rows; y++){
            StringBuilder rowStr = new StringBuilder();
            for (int x = 0; x < this.columns; x++){
                rowStr.append(this.table[y][x].toString());
            }
            System.out.println(rowStr);
        }
    }

    private boolean isInBounds(int y, int x) {
        return y >= 0 && y < rows && x >= 0 && x < columns;
    }

    public void applyPathToTable(List<Node> path){
        for (int i = 1; i < path.size() - 1; i++){
            path.get(i).isPath = true;
        }
    }

    public void markUnableToReachNodes(Robot robot) {
        int half_width = (int) Math.ceil(robot.width / 2.0);
        int half_height = (int) Math.ceil(robot.height / 2.0);

        for (int y = 0; y < this.rows; y++){
            for (int x = 0; x < this.columns; x++){
                if (this.table[y][x].isObstacle)  {
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

    public void drawBorder(Robot robot) {
        int half_width = (int) Math.ceil(robot.width / 2.0);
        int half_height = (int) Math.ceil(robot.height / 2.0);

        for (int y = 0; y < rows; y++) {
            for (int i = 0; i < half_width; i++) {
                this.table[y][i].isUnableToReach = true;
                this.table[y][columns - i - 1].isUnableToReach = true;
            }
        }

        for (int y = 0; y < half_height; y++) {
            for (int x = 0; x < columns; x++) {
                table[y][x].isUnableToReach = true;
                table[rows - y - 1][x].isUnableToReach = true;
            }
        }
    }

    public void clearRobot() {
        for (int y = 0; y < this.rows; y++) {
            for (int x = 0; x < this.columns; x++) {
                if (this.table[y][x].hasRobot) {
                    this.table[y][x].hasRobot = false;
                }
            }
        }
    }

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
