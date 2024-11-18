import java.io.*;
import java.util.*;
public class oving7 {
    private static int amountOfInterestPoints = 0;
    private static int amountOfNodes = 0;
    private static int amountOfEdges = 0;

    public static void main(String[] args) {
        try {

            List<Node> nodeList = readNodeFile("islandNoder.txt");
            System.out.println("Node list created succesfully.");
            WeightedGraphList weightGraphList = new WeightedGraphList(nodeList);

            readEdgeFile("islandKanter.txt", weightGraphList);
            System.out.println("Edges added to graph succesfully.");

            readLocationFile("isIntPkt.txt", nodeList);
            System.out.println("Locations added to nodes succesfully.");

            WeightedGraphList reversedGraph = reverseGraph(weightGraphList);
            // Test distance table creation with interest points
            List<Integer> interestPointNodeNrs = Arrays.asList(38942, 78888, 109786);

            int[][] distanceTableFromLandmark = createDistanceTableFrom(weightGraphList, interestPointNodeNrs);
            int[][] distanceTableToLandmark = createDistanceTableFrom(reversedGraph, interestPointNodeNrs);



            saveDistanceTableBinary(distanceTableFromLandmark, "distanceTableFrom.bin");
            saveDistanceTableBinary(distanceTableToLandmark, "distanceTableTo.bin");

            // System.out.println("Special node information of node 1: " + nodeList.get(1).name + " " + nodeList.get(1).placeType);

        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }



    // -----------------------------//
    //           File reading       //
    // -----------------------------//
/**
 * Reads a file containing nodes and their coordinates
 * @param filename The name of the file to read
 * @return A list of nodes
 * @throws FileNotFoundException If the file is not found
 */
public static List<Node> readNodeFile(String filename) throws FileNotFoundException {
        List<Node> nodeList = new ArrayList<>();
    try(BufferedReader bufferedReader = new BufferedReader(new FileReader(filename))) {
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
            // Add the node to the priority queue, java's priority queue is a min heap but doesnt allow for updating the priority of an element
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
     * @param filename The name of the file to read
     * @param weightGraphList A graph to add the edges to, must be initialized
     * @throws FileNotFoundException If the file is not found
     */
    public static void readEdgeFile(String filename, WeightedGraphList weightGraphList) throws FileNotFoundException {

        try(BufferedReader bufferedReader = new BufferedReader(new FileReader(filename))) {
            String line;
            boolean firstLine = true;
            while ((line = bufferedReader.readLine()) != null) {
                if (firstLine){
                    String[] firstLineList = line.trim().split("\\s+");
                    amountOfEdges = Integer.parseInt(firstLineList[0]);
                    firstLine = false;
                    // System.out.println("First line ignored");
                    continue;
                }
                // Split the line into an array of strings
                //System.out.println("Line before split: " + line);
                // Split the line into an array of strings, \\s+ is a regex that matches one or more whitespace characters (makes sure to ignore multiple spaces)
                String[] textLine = line.trim().split("\\s+");

                //System.out.println("Node from: " + textLine[0] + " Node to: " + textLine[1] + " Drive time: " + textLine[2] + " Length: " + textLine[3] + " Speed limit: " + textLine[4]);

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
     * @param filename The name of the file to read
     * @throws FileNotFoundException If the file is not found
     */
    public static void readLocationFile(String filename, List<Node> nodeList) throws FileNotFoundException {

        try (BufferedReader bufferedReader = new BufferedReader(new FileReader(filename))) {
            String line;
            boolean firstLine = true;

            while ((line = bufferedReader.readLine()) != null) {
                if (firstLine) {
                    String[] firstLineList = line.trim().split("\\s+");
                    amountOfInterestPoints = Integer.parseInt(firstLineList[0]);
                    firstLine = false;
                    continue;
                }

                // Split the line into an array of strings, \\s+ is a regex that matches one or more whitespace characters (makes sure to ignore multiple spaces)
                String[] parts = line.split("\\s+", 3);
                int nodeNr = Integer.parseInt(parts[0]);
                int placeType = Integer.parseInt(parts[1]);

                String name = parts[2].replace("\"", ""); // Remove quotes from the name

                for (int i = 0; i < nodeList.size(); i++) {
                    if (nodeList.get(i).nodeNr == nodeNr) {
                        nodeList.get(i).name = name;
                        nodeList.get(i).placeType = placeType;
                       // System.out.println("Node: " + nodeNr + " Name: " + name + " Place type: " + placeType);
                    }

                }
            }
        } catch (IOException e) {
            e.printStackTrace();

        }

    }


    // -----------------------------//
    //           Graph              //
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
        public List<Node> getNodeList(){
            return nodeList;
        }
        public List<Edge> getEdges(int node){
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
    //           Helper methods     //
    // -----------------------------//

    /**
     * Returns a list of the node numbers of the interest points
     * @param weightedGraphList The graph to get the interest points from
     * @return A list of the node numbers of the interest points
     */
    private static List<Integer> interesPointNodeNrs(WeightedGraphList weightedGraphList){
        List<Integer> interestPointNodeNrs = new ArrayList<>();
        for (Node node : weightedGraphList.getNodeList()){
            if (node.placeType != 0){
                interestPointNodeNrs.add(node.nodeNr);
            }
        }
        return interestPointNodeNrs;
    }
    private static void resetDistanceInWeight(WeightedGraphList weightedGraphList){
        // Set all node distances to max before finding path
        for (Node node : weightedGraphList.getNodeList()) {
            node.distance = Integer.MAX_VALUE;
        }
    }

    //-----------------------------//
    //           Node              //
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
        private int placeType;
        private String name;

        /**
         * Creates a new node
         * @param node The node number
         * @param breddegrad The latitude of the node
         * @param lengdegrad The longitude of the node
         */

        public Node(int node, float breddegrad, float lengdegrad){
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
    //           Edge              //
    // -----------------------------//
    public static class Edge{
        private final int to;
        private final int driveTime;
        private final int length;
        private final int speedLimit;

        /**
         * Creates a new edge, in practice it is a carroad
         * @param to The node the edge goes to
         * @param driveTime The time it takes to drive the edge, (hundredths of a second)
         * @param length The length of the edge (meters)
         * @param speedLimit The speed limit of the edge (km/h)
         */
        public Edge(int to, int driveTime, int length, int speedLimit){
            this.to = to;
            this.driveTime = driveTime;
            this.length = length;
            this.speedLimit = speedLimit;

        }
    }
    private int getTo(Edge edge){
        return edge.to;
    }
    private int getDriveTime(Edge edge){
        return edge.driveTime;
    }
    private int getLength(Edge edge){
        return edge.length;
    }
    private int getSpeedLimit(Edge edge){
        return edge.speedLimit;
    }

    // -----------------------------//
    //           Dijkstra           //
    // -----------------------------//

    public static void dijkstra(WeightedGraphList weightedGraphList, Node startNode){

        // Create a priority queue, that compares the distance of the nodes
        PriorityQueue<Node> priorityQueue = new PriorityQueue<>(Comparator.comparingInt(o -> o.distance));
        // Set the distance of the start node to 0
        startNode.distance = 0;
        // Add the start node to the priority queue
        priorityQueue.add(startNode);

        while (!priorityQueue.isEmpty()){
            // Get the node with the smallest distance
            Node currentNode = priorityQueue.poll();
            // Get the neighbors of the current node
            List<Edge> neighbors = weightedGraphList.getEdges(currentNode.nodeNr);
            // Loop through the neighbors
            for (Edge neighbor : neighbors){
                // Get the node number of the neighbor
                int neighborNodeNr = neighbor.to;
                // Get the distance to the neighbor
                int distanceToNeighbor = currentNode.distance + neighbor.driveTime;
                // If the distance to the neighbor is less than the current distance
                if (distanceToNeighbor < weightedGraphList.getNodeList().get(neighborNodeNr).distance){
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

        while (!priorityQueue.isEmpty()) {
            // Get the node with the smallest distance
            Node currentNode = priorityQueue.poll();
            count++;
            if (currentNode.equals(goal)) {
                List<Node> path = reconstructPath(cameFrom, startNode, goal);
                System.out.println("Optimal sti: " + path);
                System.out.println("Anntal noder besøkt: " + count);
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
                    // Set the distance to the neighbor
                    weightedGraphList.getNodeList().get(neighborNodeNr).distance = distanceToNeighbor;
                    // Add the neighbor to the priority queue
                    priorityQueue.add(weightedGraphList.getNodeList().get(neighborNodeNr));
                }
            }
        }
        return 0;
    }
    // -----------------------------//
    //           Distance table     //
    // -----------------------------//
    /**
     * Creates a table with the distances between the interest points, rows are the nodes, columns are the interest points
     * @param weightedGraphList The graph to calculate the distances from
     * @return A table with the distances between the interest points
     */
    public static int[][] createDistanceTableFrom(WeightedGraphList weightedGraphList, List<Integer> interestPointNodeNrs){
        // Create a table with the distances between the interest points, rows are the nodes, columns are the interest points
        int[][] distanceTable = new int[amountOfNodes][interestPointNodeNrs.size()];


        // Loop through the interest points
        for (int i = 0; i < interestPointNodeNrs.size(); i++) {
            Node currentLandmark = weightedGraphList.getNodeList().get(interestPointNodeNrs.get(i));

            for (Node node: weightedGraphList.getNodeList()){
                node.distance = Integer.MAX_VALUE;
            }

            currentLandmark.distance = 0;

            dijkstra(weightedGraphList, currentLandmark);

            for (int j = 0; j < amountOfInterestPoints; j++) {
                distanceTable[j][i] = weightedGraphList.getNodeList().get(j).distance;
            }
            System.out.println("Distance calculated for landmark: "+ currentLandmark.nodeNr+ " " + currentLandmark.name);
        }
        return distanceTable;
    }

    // -----------------------------//
    //      Save distance table     //
    // -----------------------------//
    // Ai har produsert disse, vet ikke om de faktisk fungerer
    public static void saveDistanceTableBinary(int[][] distanceTable, String filename) throws IOException {
        try (DataOutputStream dos = new DataOutputStream(new FileOutputStream(filename))) {
            int rows = distanceTable.length;
            int cols = distanceTable[0].length;

            // Lagre dimensjoner
            dos.writeInt(rows);
            dos.writeInt(cols);

            // Lagre matrisedata
            for (int[] row : distanceTable) {
                for (int value : row) {
                    dos.writeInt(value);
                }
            }
        }

    }
    public static int[][] loadDistanceTableBinary(String filename) throws IOException {
        try (DataInputStream dis = new DataInputStream(new FileInputStream(filename))) {
            // Les dimensjoner
            int rows = dis.readInt();
            int cols = dis.readInt();

            int[][] distanceTable = new int[rows][cols];

            // Les matrisedata
            for (int i = 0; i < rows; i++) {
                for (int j = 0; j < cols; j++) {
                    distanceTable[i][j] = dis.readInt();
                }
            }

            return distanceTable;
        }
    }
    // euclid heuristic, calculates the euclidean distance between two nodes
    public static double heuristicLandmark(Node start, Node goal, int landmark, int[][] distanceFromLandmark, int[][] distanceToLandmark) throws IOException {
        double landmarkToGoal = distanceFromLandmark[goal.nodeNr][landmark];
        double landmarkToStart = distanceFromLandmark[start.nodeNr][landmark];
        double nodeToLandmark = distanceToLandmark[start.nodeNr][landmark];
        double goalToLandmark = distanceToLandmark[goal.nodeNr][landmark];

        return Math.max( Math.max(landmarkToGoal - landmarkToStart, 0) , (nodeToLandmark - goalToLandmark) );
    }
public static void aStarPriority(Node node1, Node node2, int[][] distanceTableFromLandmark, int[][] distanceTableToLandmark, int nrLandmarks) throws IOException {
    // Konverter grader til radianer
    double b1 = Math.toRadians(node1.breddegrad);
    double l1 = Math.toRadians(node1.lengdegrad);
    double b2 = Math.toRadians(node2.breddegrad);
    double l2 = Math.toRadians(node2.lengdegrad);

    // Beregn differanser for breddegrad og lengdegrad
    double deltaB = b2 - b1;
    double deltaL = l2 - l1;

    // Bruk Haversine-formelen
    double a = Math.pow(Math.sin(deltaB / 2), 2) +
            Math.cos(b1) * Math.cos(b2) * Math.pow(Math.sin(deltaL / 2), 2);
    double c = 2 * Math.asin(Math.sqrt(a));

    // Returner avstanden
    double dist1 = 6371.0 * c;

    double bestDist2 = 0;
    for (int i = 0; i < nrLandmarks; i++) {
        double dist2 = heuristicLandmark(node1, node2, i, distanceTableFromLandmark, distanceTableToLandmark);
        if (dist2 < bestDist2) {
            bestDist2 = dist2;
        }
    }

    node1.distance = (int) (dist1 + bestDist2);
}

public static double aStar(WeightedGraphList graph, Node start, Node goal, int[][] distanceTableFromLandmark, int[][] distanceTableToLandmark, int nrLandmarks) throws IOException {
    PriorityQueue<Node> pq = new PriorityQueue<>(Comparator.comparingInt(o -> (int) o.aStarDistance));
    Map<Node, Node> cameFrom = new HashMap<>();
    start.aStarDistance = 0;
    pq.add(start);
    int count = 0;

    while (!pq.isEmpty()) {
        count++;
        Node current = pq.poll();
        if (current == goal) {
            break;
        }
        List<Edge> neighbors = graph.getEdges(current.nodeNr);
        for (Edge edge : neighbors) {
            Node neighbor = graph.getNodeList().get(edge.to);
            double newDist = current.aStarDistance + edge.driveTime;
            if (newDist < neighbor.aStarDistance) {
                neighbor.aStarDistance = newDist;
                cameFrom.put(neighbor, current);
                aStarPriority(neighbor, goal, distanceTableFromLandmark, distanceTableToLandmark, nrLandmarks);
                pq.add(neighbor);
            }
        }
    }


    List<Node> path = reconstructPath(cameFrom, start, goal);
    System.out.println("Optimal sti: " + path);
    System.out.println("Anntal noder besøkt: " + count);
    return goal.aStarDistance;
}

private static List<Node> reconstructPath(Map<Node, Node> cameFrom, Node start, Node goal) {
    List<Node> path = new ArrayList<>();
    Node current = goal;

    while (current != null) {
        path.add(current);
        current = cameFrom.get(current);
    }

    Collections.reverse(path); // Snu listen for å få stien fra start til mål
    return path;
}


}
