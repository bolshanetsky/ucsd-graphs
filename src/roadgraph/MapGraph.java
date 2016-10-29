/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.*;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.function.Consumer;
import java.util.stream.Collectors;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {

	private Set<GeographicPoint> verticies;
    private Set<Edge> edges;
    private Map<GeographicPoint, List<Edge>> adjacencyList;

	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
        verticies = new HashSet<>();
        edges = new HashSet<>();
        adjacencyList = new HashMap<>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		return verticies.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		return verticies;
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		return edges.size();
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		if (verticies.contains(location) || location == null) {
            return false;
        }
        else {
            verticies.add(location);
            return true;
        }
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

        // validate parameters
        if (!verticies.contains(from) || !verticies.contains(to)) {
            throw new IllegalArgumentException("To or From GeographicalPoint are not present in MapGraph");
        }

        if (from == null || to == null || roadName == null || roadType == null) {
            throw new IllegalArgumentException("No null parameters allowed");
        }

        if (length < 0) {
            throw new IllegalArgumentException("Edge length is less than 0");
        }

        // add Edge to set
        Edge newEdge = new Edge(from, to, roadName, roadType, length);
        edges.add(newEdge);

        // add Edge to adjacency list
        if (!adjacencyList.containsKey(from)) {
            List<Edge> newEdgesList = new ArrayList<>();
            newEdgesList.add(newEdge);
            adjacencyList.put(from, newEdgesList);
        }
        else {
            List<Edge> oldEdgesList = adjacencyList.get(from);
            oldEdgesList.add(newEdge);
            adjacencyList.put(from, oldEdgesList);
        }
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}

	/** Find the path from start to goal using breadth first search
	 *
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start,
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
        // initialize objects
        Queue<GeographicPoint> discoveryQueue = new LinkedBlockingQueue<>();
        HashSet<GeographicPoint> visited = new HashSet<>();
        HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap();
        parentMap.put(start, null);


        // initial setup
        discoveryQueue.add(start);
        visited.add(start);

        while (!discoveryQueue.isEmpty()){

            GeographicPoint current = discoveryQueue.poll();

            // Hook for visualization.  See writeup.
            nodeSearched.accept(current);

            if (current.equals(goal)) {
                return generatePath(current, parentMap);
            }

            List<GeographicPoint> neighbors = getNeighbors(current);
            for (GeographicPoint neighbor: neighbors) {
                if (!visited.contains(neighbor)) {
                    discoveryQueue.add(neighbor);
                    visited.add(neighbor);
                    parentMap.put(neighbor, current);
                }
            }
        }

		return null;
	}

    /**
     * Return all neighbors points that have edge with passed point.
     * @param current
     * @return List of neighbor points.
     */
    private List<GeographicPoint> getNeighbors(GeographicPoint current) {
        if (adjacencyList.get(current) == null) {
            return new ArrayList<>();
        } else {
            return adjacencyList.get(current).stream().map(edge -> edge.getToPoint()).collect(Collectors.toList());
        }

    }

    /**
     * Generates path from passed endPoint to the start of search using parents Map
     * @param endPoint
     * @param parentMap
     * @return List of location points from start till end point.
     */
    private List<GeographicPoint> generatePath(GeographicPoint endPoint, HashMap<GeographicPoint, GeographicPoint> parentMap) {
        List<GeographicPoint> path = new ArrayList<>();
        GeographicPoint current = endPoint;

        while (current != null) {
            path.add(current);
            current = parentMap.get(current);
        }

        Collections.reverse(path);

        return path;
    }


    /** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
        List<GeographicPoint> results = firstMap.bfs(new GeographicPoint(4.0, 1.0), new GeographicPoint(1, 1));
        results.forEach(result -> System.out.println(result));
		System.out.println("DONE.");
		
		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		/*
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		*/
		
		
		/* Use this code in Week 3 End of Week Quiz */
		/*MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}
	
}
