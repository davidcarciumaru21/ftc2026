package org.firstinspires.ftc.teamcode.presets;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.firstinspires.ftc.teamcode.roadRunner.drives.MecanumDrive;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;

public class PointToPoint {

    // 8 possible movement directions represented as (dx, dy)
    // These correspond to 8-neighbor connectivity (including diagonals)
    private static final int[][] DIRECTIONS = {
            {1, 0},   // right
            {1, 1},   // right-down
            {0, 1},   // down
            {-1, 1},  // left-down
            {-1, 0},  // left
            {-1, -1}, // left-up
            {0, -1},  // up
            {1, -1}   // right-up
    };

    // Corresponding movement commands for each direction above
    // Each string ends with '(' because distance is appended later, e.g., "right(3.0)"
    private static final String[] MOVEMENTS = {
            "right(", "right-down(", "down(", "left-down(",
            "left(", "left-up(", "up(", "right-up("
    };


    /**
     * Calculates Euclidean distance between two points represented as int arrays [x, y].
     * Used for actual cost calculation between nodes in A*.
     */
    public static double euclidianLenght(int[] a_coordonates, int[] b_coordonates){
        return Math.sqrt(Math.pow(a_coordonates[0] - b_coordonates[0], 2)
                + Math.pow(a_coordonates[1] - b_coordonates[1], 2));
    }

    /**
     * Calculates Manhattan distance between two points [x, y].
     * Used as heuristic estimate in A* search.
     */
    public static int manhattanLenght(int[] a_coordonates, int[] b_coordonates) {
        return Math.abs(a_coordonates[0] - b_coordonates[0]) +
                Math.abs(a_coordonates[1] - b_coordonates[1]);
    }

    /**
     * Finds all valid neighboring nodes around the given node in the grid/table.
     * Checks 8 directions and excludes obstacles or unreachable nodes.
     *
     * @param node The current node to find neighbors for.
     * @param table The grid/table containing nodes.
     * @return List of accessible neighbor nodes.
     */
    public static List<Node> getNeighbors(Node node, Table table) {
        List<Node> neighbors = new ArrayList<>();
        for (int[] dir : DIRECTIONS) {
            int newX = node.x + dir[0];
            int newY = node.y + dir[1];

            // Ensure neighbor coordinates are within table bounds
            if (newX >= 0 && newX < table.columns && newY >= 0 && newY < table.rows) {
                Node neighbor = table.table[newY][newX];

                // Exclude neighbors that are obstacles or marked as unreachable
                if (!neighbor.isObstacle && !neighbor.isUnableToReach) {
                    neighbors.add(neighbor);
                }
            }
        }
        return neighbors;
    }

    /**
     * Reconstructs the path from end node back to start by following parent links.
     * The path is reversed to go from start to end.
     *
     * @param endNode The final node in the path.
     * @return List of nodes representing the full path from start to end.
     */
    public static List<Node> reconstructPath(Node endNode) {
        List<Node> path = new ArrayList<>();
        Node current = endNode;
        while (current != null) {
            path.add(current);
            current = current.parent;  // Follow the parent link back to the start
        }
        Collections.reverse(path);
        return path;
    }


    /**
     * A* pathfinding algorithm implementation.
     * Finds shortest path from startNode to endNode on the given table.
     * Uses Manhattan distance as heuristic and Euclidean distance as cost.
     *
     * @param table The grid/table containing all nodes.
     * @param startNode The starting point node.
     * @param endNode The destination node.
     * @param robot The robot instance (unused in current code but could be for extensions).
     * @return List of nodes representing the path, or null if no path found.
     */
    public static List<Node> aStar(Table table, Node startNode, Node endNode, Robot robot) {
        // Initialize start node costs
        startNode.g = 0;
        startNode.h = manhattanLenght(startNode.coordonates, endNode.coordonates);
        startNode.f = startNode.g + startNode.h;

        List<Node> openSet = new ArrayList<>();  // Nodes to be evaluated
        Set<Node> closedSet = new HashSet<>();   // Nodes already evaluated
        openSet.add(startNode);

        // Main loop runs while there are nodes to process
        while (!openSet.isEmpty()) {
            Node currentNode = null;
            double lowestF = Double.POSITIVE_INFINITY;

            // Find node in openSet with lowest f cost (f = g + h)
            for (Node node : openSet) {
                if (node.f < lowestF) {
                    lowestF = node.f;
                    currentNode = node;
                }
            }

            // If no node found (should not happen), exit
            if (currentNode == null) break;

            // If goal reached, reconstruct and return the path
            if (currentNode == endNode) {
                return reconstructPath(currentNode);
            }

            openSet.remove(currentNode);
            closedSet.add(currentNode);

            // Explore neighbors of current node
            for (Node neighbor : getNeighbors(currentNode, table)) {
                // Skip if neighbor already evaluated
                if (closedSet.contains(neighbor)) continue;

                // Tentative g score is current's g plus distance to neighbor
                double tentativeG = currentNode.g + euclidianLenght(currentNode.coordonates, neighbor.coordonates);

                // If new path to neighbor is better than previous known path
                if (tentativeG < neighbor.g) {
                    neighbor.parent = currentNode;
                    neighbor.g = tentativeG;
                    neighbor.h = manhattanLenght(neighbor.coordonates, endNode.coordonates);
                    neighbor.f = neighbor.g + neighbor.h;

                    // Add neighbor to open set if not already present
                    if (!openSet.contains(neighbor)) {
                        openSet.add(neighbor);
                    }
                }
            }
        }
        // No path found
        return null;
    }

    /**
     * Converts a list of nodes representing a path into movement commands.
     * Each step is converted into a directional movement command with distance.
     * Uses table.mu as the base movement unit.
     *
     * @param path The list of nodes representing the path.
     * @param table The table/grid with movement scaling information.
     * @return List of movement commands as strings, e.g., "right(1.0)"
     */
    public static List<String> pathToCommands(List<Node> path, Table table) {
        List<String> movementList = new ArrayList<>();

        // Iterate through each pair of consecutive nodes in the path
        for (int i = 0; i < path.size() - 1; i++) {
            Node current = path.get(i);
            Node nextNode = path.get(i + 1);

            int dx = nextNode.x - current.x; // horizontal step difference
            int dy = nextNode.y - current.y; // vertical step difference

            boolean foundDirection = false;

            // Match dx, dy with one of the predefined directions
            for (int j = 0; j < DIRECTIONS.length; j++) {
                if (DIRECTIONS[j][0] == dx && DIRECTIONS[j][1] == dy) {
                    double distance;

                    // Diagonal moves have distance = base * sqrt(2) (approx 1.4142)
                    if (j % 2 == 1) {
                        distance = table.mu * 1.4142;
                    } else {
                        // Straight moves have distance = base movement unit
                        distance = table.mu;
                    }

                    // Add the movement command string with direction and distance
                    movementList.add(MOVEMENTS[j] + distance + ")");
                    foundDirection = true;
                    break;
                }
            }

            // If no matching direction found, add "unknown" to signal an issue
            if (!foundDirection) {
                movementList.add("unknown");
            }
        }

        return movementList;
    }

    /**
     * Compresses consecutive identical movement commands into a single command
     * by summing their distances, reducing the number of commands to execute.
     *
     * @param movementList The list of movement commands to compress.
     * @return A compressed list with combined movement distances.
     */
    public static List<String> compressPath(List<String> movementList) {
        List<String> compressedPath = new ArrayList<>();
        int i = 0;

        // Iterate through movementList and combine consecutive identical commands
        while (i < movementList.size()) {
            String currentMove = movementList.get(i);
            int pIdx = currentMove.indexOf('(');
            String func = currentMove.substring(0, pIdx); // e.g., "right"
            double total = Double.parseDouble(currentMove.substring(pIdx + 1, currentMove.length() - 1));
            i++;

            // Continue combining while next moves have the same function name
            while (i < movementList.size()) {
                String nextMove = movementList.get(i);
                int nextPIdx = nextMove.indexOf('(');
                String nextFunc = nextMove.substring(0, nextPIdx);
                double nextVal = Double.parseDouble(nextMove.substring(nextPIdx + 1, nextMove.length() - 1));

                if (nextFunc.equals(func)) {
                    total += nextVal;  // sum distances for consecutive moves
                    i++;
                } else {
                    break; // different command, stop combining
                }
            }

            // Add the compressed move with total distance back to the list
            compressedPath.add(func + "(" + total + ")");
        }
        return compressedPath;
    }

    /**
     * Executes a sequence of compressed movement commands on the robot's mecanum drive.
     * For each command, it calls the appropriate Movement function and runs the action.
     * Updates the current robot pose after each movement.
     *
     * @param compressedPath List of movement commands to execute.
     * @param drive The mecanum drive object controlling the robot.
     * @param startPose The starting pose of the robot.
     */
    public static void executePath(List<String> compressedPath, MecanumDrive drive, Pose2d startPose) {
        Pose2d currentPose = startPose;

        // Process each command string
        for (String move : compressedPath) {
            int pIdx = move.indexOf('(');

            // Basic validation of command format: must contain '(' and end with ')'
            if (pIdx == -1 || !move.endsWith(")")) {
                System.out.println("Invalid move format: " + move);
                continue;
            }

            String funcName = move.substring(0, pIdx).trim();
            String argsStr = move.substring(pIdx + 1, move.length() - 1).trim();

            double length;

            // Parse the distance argument for the movement command
            try {
                length = Double.parseDouble(argsStr);
            } catch (NumberFormatException e) {
                System.out.println("Invalid length argument in move: " + move);
                continue;
            }

            Action action = null;

            // Map movement function name to the corresponding Movement method
            switch (funcName) {
                case "right":
                    action = MovementPointToPoint.right(length, drive, currentPose);
                    break;
                case "right-down":
                    action = MovementPointToPoint.rightDown(length, drive, currentPose);
                    break;
                case "down":
                    action = MovementPointToPoint.down(length, drive, currentPose);
                    break;
                case "left-down":
                    action = MovementPointToPoint.leftDown(length, drive, currentPose);
                    break;
                case "left":
                    action = MovementPointToPoint.left(length, drive, currentPose);
                    break;
                case "left-up":
                    action = MovementPointToPoint.leftUp(length, drive, currentPose);
                    break;
                case "up":
                    action = MovementPointToPoint.up(length, drive, currentPose);
                    break;
                case "right-up":
                    action = MovementPointToPoint.rightUp(length, drive, currentPose);
                    break;
                default:
                    System.out.println("Unknown movement function: " + funcName);
                    continue;
            }

            // If valid action created, execute it and update current pose
            if (action != null) {
                Actions.runBlocking(action);
                currentPose = drive.localizer.getPose();
                System.out.println("Executed: " + funcName + "(" + length + ")");
            }
        }
    }
}
