package org.firstinspires.ftc.teamcode.teleop.presets.pointToPoint;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Consumer;

public class PointToPoint {

    private static final int[][] DIRECTIONS = {
            {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}
    };

    private static final String[] MOVEMENTS = {
            "right(", "right-down(", "down(", "left-down(",
            "left(", "left-up(", "up(", "right-up("
    };

    public static double lenght(int[] a_coordonates, int[] b_coordonates){
        return Math.sqrt(Math.pow(a_coordonates[0] - b_coordonates[0], 2)
                + Math.pow(a_coordonates[1] - b_coordonates[1], 2));
    }

    public static int manhattan_lenght(int[] a_coordonates, int[] b_coordonates) {
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
        startNode.h = manhattan_lenght(startNode.coordonates, endNode.coordonates);
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

                double tentativeG = currentNode.g + lenght(currentNode.coordonates, neighbor.coordonates);

                if (tentativeG < neighbor.g) {
                    neighbor.parent = currentNode;
                    neighbor.g = tentativeG;
                    neighbor.h = manhattan_lenght(neighbor.coordonates, endNode.coordonates);
                    neighbor.f = neighbor.g + neighbor.h;

                    if (!openSet.contains(neighbor)) {
                        openSet.add(neighbor);
                    }
                }
            }
        }
        return null;
    }

    public static List<String> path_to_commands(List<Node> path, Table table) {
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

    public static List<String> compress_path(List<String> movementList) {
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

    public static void execute_path(List<String> compressedPath) {
        Map<String, Consumer<Double>> movementFunctions = new HashMap<>();
        movementFunctions.put("right", Movement::right);
        movementFunctions.put("right-down", Movement::rightDown);
        movementFunctions.put("down", Movement::down);
        movementFunctions.put("left-down", Movement::leftDown);
        movementFunctions.put("left", Movement::left);
        movementFunctions.put("left-up", Movement::leftUp);
        movementFunctions.put("up", Movement::up);
        movementFunctions.put("right-up", Movement::rightUp);

        for (String move : compressedPath) {
            int pIdx = move.indexOf('(');
            String funcName = move.substring(0, pIdx);
            double argument = Double.parseDouble(move.substring(pIdx + 1, move.length() - 1));

            Consumer<Double> func = movementFunctions.get(funcName);
            if (func != null) {
                func.accept(argument);
            } else {
                System.out.println("Unknown movement function: " + funcName);
            }
        }
    }

    public static void animate_path(Robot robot, List<Node> path, Table table) throws InterruptedException {
        table.mark_unable_to_reach_nodes(robot);
        table.apply_path_to_table(path);

        for (Node node : path) {
            try {
                new ProcessBuilder("cmd", "/c", "cls").inheritIO().start().waitFor();
            } catch (Exception e) {
                System.out.println("Could not clear console: " + e.getMessage());
            }
            table.generate_robot(robot, new int[]{node.y, node.x});
            table.print_matrix();
            Thread.sleep(50);
            table.clear_robot();
        }
    }
}