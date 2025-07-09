package org.firstinspires.ftc.teamcode.teleop.presets.pointToPoint;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;

public class PointToPoint {

    private static final int[][] DIRECTIONS = {
            {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}
    };

    private static final String[] MOVEMENTS = {
            "right(", "right-down(", "down(", "left-down(",
            "left(", "left-up(", "up(", "right-up("
    };

    public static double euclidianLenght(int[] a_coordonates, int[] b_coordonates){
        return Math.sqrt(Math.pow(a_coordonates[0] - b_coordonates[0], 2)
                + Math.pow(a_coordonates[1] - b_coordonates[1], 2));
    }

    public static int manhattanLenght(int[] a_coordonates, int[] b_coordonates) {
        return Math.abs(a_coordonates[0] - b_coordonates[0]) +
                Math.abs(a_coordonates[1] - b_coordonates[1]);
    }

    public static List<Node> getNeighbors(Node node, Table table) {
        List<Node> neighbors = new ArrayList<>();
        for (int[] dir : DIRECTIONS) {
            int newX = node.x + dir[0];
            int newY = node.y + dir[1];
            if (newX >= 0 && newX < table.columns && newY >= 0 && newY < table.rows) {
                Node neighbor = table.table[newY][newX];
                if (!neighbor.isObstacle && !neighbor.isUnableToReach) {
                    neighbors.add(neighbor);
                }
            }
        }
        return neighbors;
    }

    public static List<Node> reconstructPath(Node endNode) {
        List<Node> path = new ArrayList<>();
        Node current = endNode;
        while (current != null) {
            path.add(current);
            current = current.parent;
        }
        Collections.reverse(path);
        return path;
    }


    public static List<Node> aStar(Table table, Node startNode, Node endNode, Robot robot) {
        startNode.g = 0;
        startNode.h = manhattanLenght(startNode.coordonates, endNode.coordonates);
        startNode.f = startNode.g + startNode.h;

        List<Node> openSet = new ArrayList<>();
        Set<Node> closedSet = new HashSet<>();
        openSet.add(startNode);

        while (!openSet.isEmpty()) {
            Node currentNode = null;
            double lowestF = Double.POSITIVE_INFINITY;

            for (Node node : openSet) {
                if (node.f < lowestF) {
                    lowestF = node.f;
                    currentNode = node;
                }
            }

            if (currentNode == null) break;

            if (currentNode == endNode) {
                return reconstructPath(currentNode);
            }

            openSet.remove(currentNode);
            closedSet.add(currentNode);

            for (Node neighbor : getNeighbors(currentNode, table)) {
                if (closedSet.contains(neighbor)) continue;

                double tentativeG = currentNode.g + euclidianLenght(currentNode.coordonates, neighbor.coordonates);

                if (tentativeG < neighbor.g) {
                    neighbor.parent = currentNode;
                    neighbor.g = tentativeG;
                    neighbor.h = manhattanLenght(neighbor.coordonates, endNode.coordonates);
                    neighbor.f = neighbor.g + neighbor.h;

                    if (!openSet.contains(neighbor)) {
                        openSet.add(neighbor);
                    }
                }
            }
        }
        return null;
    }

    public static List<String> pathToCommands(List<Node> path, Table table) {
        List<String> movementList = new ArrayList<>();

        for (int i = 0; i < path.size() - 1; i++) {
            Node current = path.get(i);
            Node nextNode = path.get(i + 1);

            int dx = nextNode.x - current.x;
            int dy = nextNode.y - current.y;

            boolean foundDirection = false;

            for (int j = 0; j < DIRECTIONS.length; j++) {
                if (DIRECTIONS[j][0] == dx && DIRECTIONS[j][1] == dy) {
                    double distance;
                    if (j % 2 == 1) {
                        distance = table.mu * Math.sqrt(2);
                    } else {
                        distance = table.mu;
                    }
                    movementList.add(MOVEMENTS[j] + distance + ")");
                    foundDirection = true;
                    break;
                }
            }

            if (!foundDirection) {
                movementList.add("unknown");
            }
        }

        return movementList;
    }

    public static List<String> compressPath(List<String> movementList) {
        List<String> compressedPath = new ArrayList<>();
        int i = 0;
        while (i < movementList.size()) {
            String currentMove = movementList.get(i);
            int pIdx = currentMove.indexOf('(');
            String func = currentMove.substring(0, pIdx);
            double total = Double.parseDouble(currentMove.substring(pIdx + 1, currentMove.length() - 1));
            i++;

            while (i < movementList.size()) {
                String nextMove = movementList.get(i);
                int nextPIdx = nextMove.indexOf('(');
                String nextFunc = nextMove.substring(0, nextPIdx);
                double nextVal = Double.parseDouble(nextMove.substring(nextPIdx + 1, nextMove.length() - 1));

                if (nextFunc.equals(func)) {
                    total += nextVal;
                    i++;
                } else {
                    break;
                }
            }

            compressedPath.add(func + "(" + total + ")");
        }
        return compressedPath;
    }

    public static void executePath(List<String> compressedPath, MecanumDrive drive, Pose2d startPose) {
        Pose2d currentPose = startPose;

        for (String move : compressedPath) {
            int pIdx = move.indexOf('(');
            if (pIdx == -1 || !move.endsWith(")")) {
                System.out.println("Invalid move format: " + move);
                continue;
            }
            String funcName = move.substring(0, pIdx).trim();
            String argsStr = move.substring(pIdx + 1, move.length() - 1).trim();

            double length;
            try {
                length = Double.parseDouble(argsStr);
            } catch (NumberFormatException e) {
                System.out.println("Invalid length argument in move: " + move);
                continue;
            }

            Action action = null;

            switch (funcName) {
                case "right":
                    action = Movement.right(length, drive, currentPose);
                    break;
                case "right-down":
                    action = Movement.rightDown(length, drive, currentPose);
                    break;
                case "down":
                    action = Movement.down(length, drive, currentPose);
                    break;
                case "left-down":
                    action = Movement.leftDown(length, drive, currentPose);
                    break;
                case "left":
                    action = Movement.left(length, drive, currentPose);
                    break;
                case "left-up":
                    action = Movement.leftUp(length, drive, currentPose);
                    break;
                case "up":
                    action = Movement.up(length, drive, currentPose);
                    break;
                case "right-up":
                    action = Movement.rightUp(length, drive, currentPose);
                    break;
                default:
                    System.out.println("Unknown movement function: " + funcName);
                    continue;
            }

            if (action != null) {
                Actions.runBlocking(action);
                currentPose = drive.localizer.getPose();
                System.out.println("Executed: " + funcName + "(" + length + ")");
            }
        }
    }
}