
import java.io.*;
import java.util.*;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
public class oving7 {

    private static int amountOfInterestPoints = 0;

    private static int amountOfNodes = 0;

    private static int amountOfEdges = 0;

    public static void main(String[] args) {

        try {

            int count = 0;

            List<Node> nodeList = readNodeFile("noder.txt");

            System.out.println("Node list created succesfully.");

            WeightedGraphList weightGraphList = new WeightedGraphList(nodeList);

            readEdgeFile("kanter.txt", weightGraphList);

            System.out.println("Edges added to graph succesfully.");

            List<Integer> interestPointNodeNrs = Arrays.asList(5542364, 3863916, 7945639, 878026);

            readLocationFile("interessepunkt.txt", nodeList);

//System.out.println("Locations added to nodes succesfully.");

            WeightedGraphList reversedGraph = reverseGraph(weightGraphList);

            System.out.println("Graph reversed succesfully.");

// Test distance table creation with interest points

//System.out.println("Starting distance table creation");

            int[][] distanceTableFromLandmark = createDistanceTableFrom(weightGraphList, interestPointNodeNrs);

//System.out.println("Distance table from created succesfully.");

            int[][] distanceTableToLandmark = createDistanceTableFrom(reversedGraph, interestPointNodeNrs);

//System.out.println("Distance table to created succesfully.");
 
/*for (int i = 0; i < amountOfNodes; i++) {
 
System.out.println(" " + i + " " + distanceTableToLandmark[i][0] + " " + distanceTableToLandmark[i][1] + " " + 
distanceTableToLandmark[i][2]);
 
if (distanceTableFromLandmark[i][0] != 0) {
 
count++;
 
}
 
}*/

//System.out.println(count);

//int[][] distanceTableFromLandmark = loadDistanceTableBinary("testNewTo.bin");

//int[][] distanceTableToLandmark = loadDistanceTableBinary("testNewFrom.bin");

//System.out.println("Saving distance tables to binary files");

// saveDistanceTableBinary(distanceTableFromLandmark, "distanceTableFrom.bin");

//System.out.println("Distance table from saved succesfully.");

// saveDistanceTableBinary(distanceTableToLandmark, "distanceTableTo.bin");

//System.out.println("Distance table to saved succesfully.");

// Load distance tables from binary files

            Node startNode = weightGraphList.getNode(2486870);

            Node goalNode = weightGraphList.getNode(5394165);

            resetDistanceInWeight(weightGraphList);

            double distanceToGoalDijkstras = dijkstrasPathFinder(weightGraphList, startNode, goalNode);

            System.out.println("Dijkstras completed succesfully.");

            resetDistanceInWeight(weightGraphList);

            double distanceToGoal = aStar(weightGraphList, startNode, goalNode, distanceTableFromLandmark, distanceTableToLandmark,
                    interestPointNodeNrs.size());

            System.out.println("A star completed");

            Node levangerNode = weightGraphList.getNode(790843);

//System.out.println("\n Closest petrolstation nodes to Levanger: ");

//List<Node> closestNodes = findClosestTypeNode(weightGraphList, levangerNode , 8);

//printPath(closestNodes);

        } catch (FileNotFoundException e) {

            e.printStackTrace();

        } catch (IOException e) {

            throw new RuntimeException(e);

        }

    }

// -----------------------------//

    //
    File reading
//


// -----------------------------//

    /**

     * Reads a file containing nodes and their coordinates

     *

     * @param filename The name of the file to read

     * @return A list of nodes

     * @throws FileNotFoundException If the file is not found

     */

    public static List<Node> readNodeFile(String filename) throws FileNotFoundException {

        List<Node> nodeList = new ArrayList<>();

        try (BufferedReader bufferedReader = new BufferedReader(new FileReader(filename))) {

            String line;

            boolean firstLine = true;

            while ((line = bufferedReader.readLine()) != null) {

// ignore first line in the file

                if (firstLine) {

                    String[] firstLineList = line.trim().split("\\s+");

                    amountOfNodes = Integer.parseInt(firstLineList[0]);

                    firstLine = false;

//System.out.println("First line skipped");
                    continue;

                }

// Split the line into an array of strings

                String[] textLine = line.trim().split("\\s+");

// System.out.println("Line before split: " + line);

                int node = Integer.parseInt(textLine[0]);

                float breddegrad = Float.parseFloat(textLine[1]);

                float lengdegrad = Float.parseFloat(textLine[2]);

// Add the node to the priority queue, java's priority queue is a min heap but doesnt allow for updating the priority of an 
                element

                Node newNode = new Node(node, breddegrad, lengdegrad);

                nodeList.add(newNode);

// System.out.println("Node: " + node + " Breddegrad: " + breddegrad + " Lengdegrad: " + lengdegrad);

            }

        } catch (IOException e) {

            e.printStackTrace();

        }

        return nodeList;

    }

    /**

     * Reads a file containing edges between nodes

     *

     * @param filename
    The name of the file to read

     * @param weightGraphList A graph to add the edges to, must be initialized

     * @throws FileNotFoundException If the file is not found

     */

    public static void readEdgeFile(String filename, WeightedGraphList weightGraphList) throws FileNotFoundException {

        try (BufferedReader bufferedReader = new BufferedReader(new FileReader(filename))) {

            String line;

            boolean firstLine = true;

            while ((line = bufferedReader.readLine()) != null) {

                if (firstLine) {

                    String[] firstLineList = line.trim().split("\\s+");

                    amountOfEdges = Integer.parseInt(firstLineList[0]);

                    firstLine = false;

// System.out.println("First line ignored");
                    continue;

                }

// Split the line into an array of strings

//System.out.println("Line before split: " + line);

// Split the line into an array of strings, \\s+ is a regex that matches one or more whitespace characters (makes sure to 
                ignore multiple spaces)

                String[] textLine = line.trim().split("\\s+");

//System.out.println("Node from: " + textLine[0] + " Node to: " + textLine[1] + " Drive time: " + textLine[2] + " Length: " + 
                textLine[3] + " Speed limit: " + textLine[4]);

                int fromNode = Integer.parseInt(textLine[0]);

                int toNode = Integer.parseInt(textLine[1]);

                int driveTime = Integer.parseInt(textLine[2]);

                int length = Integer.parseInt(textLine[3]);

                int speedLimit = Integer.parseInt(textLine[4]);

// Add the edge to the graph

                weightGraphList.addEdge(fromNode, toNode, driveTime, length, speedLimit);

            }

        } catch (IOException e) {

            e.printStackTrace();

        }

    }

    /**


     * Reads a file containing locations and their names

     *

     * @param filename The name of the file to read

     * @throws FileNotFoundException If the file is not found

     */

    public static void readLocationFile(String filename, List<Node> nodeList) throws FileNotFoundException {

        try (BufferedReader bufferedReader = new BufferedReader(new FileReader(filename))) {

            String line;

            boolean firstLine = true;

            String[] parts = new String[3]; // Midlertidig buffer for splittede verdier

            int count = 0;

            while ((line = bufferedReader.readLine()) != null) {

// Håndter første linje

                if (firstLine) {

                    hsplit(line, parts, 1); // Splitt kun første verdi

                    amountOfInterestPoints = Integer.parseInt(parts[0]);

                    firstLine = false;

                    continue;

                }

// Bruk tilpasset split-metode for å dele opp linjen i tre deler

                String[] res = line.split("\\s+");

                int nodeNr = Integer.parseInt(res[0]);

                int placeType = Integer.parseInt(res[1]);

// join the rest of the parts to get the name

                StringBuilder name = new StringBuilder();

                for (int i = 2; i < res.length; i++) {

                    name.append(res[i]);

                    if (i < res.length - 1) {

                        name.append(" ");

                    }

                }

// remove the quotes from the name

                name = new StringBuilder(name.toString().replaceAll("\"", ""));

// Oppdater noden i nodeList med de nye dataene

                Node node = nodeList.get(nodeNr);

                node.placeType = placeType;

                node.name = name.toString();
// 
                for (int i = 0; i < nodeList.size(); i++) {
// 
                    if (nodeList.get(i).nodeNr == nodeNr) {
// 
                        nodeList.get(i).placeType = placeType;
// 
                        nodeList.get(i).name = name.toString();
// 
                    }
// 
                }

                count++;
 
/*if (count%1000 == 0){
 
System.out.println("Count: "+ count + " Node: " + nodeNr + " Name: " + name + " Place type: " + placeType);
 
}*/

            }

        } catch (IOException e) {

            e.printStackTrace();

        }

    }

    static void hsplit(String linje, String[] felt, int antall) {

        int j = 0;

        int lengde = linje.length();

        for (int i = 0; i < antall; i++) {

// Hopp over innledende blanke tegn

            while (j < lengde && linje.charAt(j) <= ' ') {

                j++;

            }

// Hvis vi har nådd slutten av linjen, avslutt

            if (j >= lengde) {

                felt[i] = null;

                return;

            }

// Finn starten av ordet

            int ordstart = j;

// Finn slutten av ordet (hopp over ikke-blanke tegn)

            while (j < lengde && linje.charAt(j) > ' ') {

                j++;

            }

// Lagre ordet i felt-arrayet

            felt[i] = linje.substring(ordstart, j);

        }


    }

// -----------------------------//

// 
    Graph
//

// -----------------------------//

// Naboliste

    /**

     * A weighted graph represented as a linked list of neighbors

     */

    private static class WeightedGraphList {

        private List<Node> nodeList;

        private List<List<Edge>> weightedGraphList;

        public WeightedGraphList(List<Node> nodeList) {

            this.nodeList = nodeList;

            weightedGraphList = new ArrayList<>();

// Create an empty neighbor list for each node

            for (int i = 0; i < nodeList.size(); i++) {

                weightedGraphList.add(new ArrayList<>());

            }

        }

        public List<Node> getNodeList() {

            return nodeList;

        }

        public Node getNode(int node) {

            return nodeList.get(node);

        }

        public List<Edge> getEdges(int node) {

            return weightedGraphList.get(node);

        }

// Adds an edge between node fromNode and toNode with the weight

        public void addEdge(int fromNode, int toNode, int driveTime, int length, int speedLimit) {

            weightedGraphList.get(fromNode).add(new Edge(toNode, driveTime, length, speedLimit));

        }

    }

    public static WeightedGraphList reverseGraph(WeightedGraphList originalGraph) {

// Get the list of nodes in the original graph

        List<Node> nodeList = originalGraph.getNodeList();

// Create a new graph with the same nodes

        WeightedGraphList reversedGraph = new WeightedGraphList(nodeList);

// Loop through all the nodes in the original graph

        for (int fromNode = 0; fromNode < nodeList.size(); fromNode++) {

            List<Edge> edges = originalGraph.getEdges(fromNode);

            for (Edge edge : edges) {

// reverse the edge

                reversedGraph.addEdge(edge.to, fromNode, edge.driveTime, edge.length, edge.speedLimit);

            }

        }

        return reversedGraph;

    }

//public static WeightedGraphList reversedWeightedGraphList(){

//}

// -----------------------------//

    //
    Helper methods
//

// -----------------------------//

    /**

     * Returns a list of the node numbers of the interest points

     *

     * @param weightedGraphList The graph to get the interest points from

     * @return A list of the node numbers of the interest points

     */

    private static List<Integer> interesPointNodeNrs(WeightedGraphList weightedGraphList) {

        List<Integer> interestPointNodeNrs = new ArrayList<>();

        for (Node node : weightedGraphList.getNodeList()) {

            if (node.placeType != 0) {

                interestPointNodeNrs.add(node.nodeNr);

            }

        }

        return interestPointNodeNrs;

    }


    private static void resetDistanceInWeight(WeightedGraphList weightedGraphList) {

// Set all node distances to max before finding path

        for (Node node : weightedGraphList.getNodeList()) {

            node.distance = Integer.MAX_VALUE;

            node.aStarDistance = Integer.MAX_VALUE;

            node.huristicDistance = 0;

        }

    }

//-----------------------------//

// 
    Node
//

//-----------------------------//

    /**

     * A node in the graph, with the values nodeNr, breddegrad, lengdegrad and distance.

     * Distance is used for Dijkstra's algorithm and is set to Integer.MAX_VALUE by default

     */

    public static class Node {

        private final int nodeNr;

        private final float breddegrad;

        private final float lengdegrad;

        private int distance = Integer.MAX_VALUE;

        private double aStarDistance = Integer.MAX_VALUE;

        private double huristicDistance = Integer.MAX_VALUE;

        private int placeType;

        private String name;

        /**

         * Creates a new node

         *

         * @param node
        The node number

         * @param breddegrad The latitude of the node

         * @param lengdegrad The longitude of the node

         */

        public Node(int node, float breddegrad, float lengdegrad) {

            this.nodeNr = node;

            this.breddegrad = breddegrad;

            this.lengdegrad = lengdegrad;

            this.name = null;

            this.placeType = 0;

        }

    }

/**

 * A weighted edge in the graph, with the values driveTime, length and speedLimit

 */

// -----------------------------//

// 
    Edge
//

// -----------------------------//

    public static class Edge {

        private final int to;

        private final int driveTime;

        private final int length;

        private final int speedLimit;

        /**

         * Creates a new edge, in practice it is a carroad

         *

         * @param to
        The node the edge goes to

         * @param driveTime The time it takes to drive the edge, (hundredths of a second)

         * @param length
        The length of the edge (meters)

         * @param speedLimit The speed limit of the edge (km/h)

         */

        public Edge(int to, int driveTime, int length, int speedLimit) {

            this.to = to;

            this.driveTime = driveTime;

            this.length = length;

            this.speedLimit = speedLimit;

        }

    }

    private int getTo(Edge edge) {

        return edge.to;

    }

    private int getDriveTime(Edge edge) {

        return edge.driveTime;

    }

    private int getLength(Edge edge) {

        return edge.length;

    }

    private int getSpeedLimit(Edge edge) {

        return edge.speedLimit;


    }

// -----------------------------//

// 
    Dijkstra
//

// -----------------------------//

    public static void dijkstra(WeightedGraphList weightedGraphList, Node startNode) {

// Create a priority queue, that compares the distance of the nodes

        PriorityQueue<Node> priorityQueue = new PriorityQueue<>(Comparator.comparingInt(o -> o.distance));

// Set the distance of the start node to 0

        startNode.distance = 0;

// Add the start node to the priority queue

        priorityQueue.add(startNode);

        while (!priorityQueue.isEmpty()) {

// Get the node with the smallest distance

            Node currentNode = priorityQueue.poll();

// Get the neighbors of the current node

            List<Edge> neighbors = weightedGraphList.getEdges(currentNode.nodeNr);

// Loop through the neighbors

            for (Edge neighbor : neighbors) {

// Get the node number of the neighbor

                int neighborNodeNr = neighbor.to;

// Get the distance to the neighbor

                int distanceToNeighbor = currentNode.distance + neighbor.driveTime;

// If the distance to the neighbor is less than the current distance

                if (distanceToNeighbor < weightedGraphList.getNodeList().get(neighborNodeNr).distance) {

// Set the distance to the neighbor

                    weightedGraphList.getNodeList().get(neighborNodeNr).distance = distanceToNeighbor;

// Add the neighbor to the priority queue

                    priorityQueue.add(weightedGraphList.getNodeList().get(neighborNodeNr));

                }

            }

        }

    }

    public static double dijkstrasPathFinder(WeightedGraphList weightedGraphList, Node startNode, Node goal) {

// Create a priority queue, that compares the distance of the nodes

        PriorityQueue<Node> priorityQueue = new PriorityQueue<>(Comparator.comparingInt(o -> o.distance));

// Set the distance of the start node to 0

        startNode.distance = 0;

// Add the start node to the priority queue

        priorityQueue.add(startNode);

        Map<Node, Node> cameFrom = new HashMap<>();

        int count = 0;

        long startTime = System.currentTimeMillis();

        while (!priorityQueue.isEmpty()) {

// Get the node with the smallest distance

            Node currentNode = priorityQueue.poll();

            count++;

            if (currentNode.equals(goal)) {

                long finishTime = System.currentTimeMillis();

                long timeElapsed = finishTime - startTime;

                System.out.println("Dijsktras");

                List<Node> path = reconstructPath(cameFrom, startNode, goal);

// System.out.println("Optimal sti: ");

                System.out.println("Amount of nodes in path: " + path.size());

                System.out.println("Antall noder besøkt: " + count);

                String resultTime = formatTime(goal.distance);

                System.out.println("Tid i bil (tt:mm:ss): " + resultTime);

                System.out.println("Tid brukt på å finne sti: " + timeElapsed + " ms");

// printPath(path);

                return currentNode.distance;

            }

// Get the neighbors of the current node

            List<Edge> neighbors = weightedGraphList.getEdges(currentNode.nodeNr);

// Loop through the neighbors

            for (Edge neighbor : neighbors) {

// Get the node number of the neighbor

                int neighborNodeNr = neighbor.to;

// Get the distance to the neighbor

                int distanceToNeighbor = currentNode.distance + neighbor.driveTime;

// If the distance to the neighbor is less than the current distance

                if (distanceToNeighbor < weightedGraphList.getNodeList().get(neighborNodeNr).distance) {

                    cameFrom.put(weightedGraphList.getNodeList().get(neighborNodeNr), currentNode);

// Set the distance to the neighbor

                    weightedGraphList.getNodeList().get(neighborNodeNr).distance = distanceToNeighbor;

// Add the neighbor to the priority queue

                    priorityQueue.add(weightedGraphList.getNodeList().get(neighborNodeNr));

                }

            }

        }

        System.out.println("No path found");