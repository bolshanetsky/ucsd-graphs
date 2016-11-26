package roadgraph;

import geography.GeographicPoint;


public class Node implements Comparable<Node> {

    private GeographicPoint location;
    private double distance;
    private double priority;

    public Node(GeographicPoint location) {
        this.location = location;
        this.distance = Double.MAX_VALUE;
        this.priority = Double.MAX_VALUE;
    }

    public GeographicPoint getLocation() {
        return this.location;
    }

    public double getDistance(){
        return this.distance;
    }

    public void setDistance(double distance) {
        this.distance = distance;
    }

    /** ToString to print out a MapNode object
     *  @return the string representation of a MapNode
     */
    @Override
    public String toString()
    {
        return "[NODE at location (" + location + ")";
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        Node node = (Node) o;

        return location.equals(node.location);

    }

    @Override
    public int hashCode() {
        return location.hashCode();
    }

    @Override
    public int compareTo(Node other) {
        if (this.priority == other.priority){
            return 0;
        }

        if (this.priority < other.priority){
            return -1;
        }

        if (this.priority > other.priority){
            return 1;
        }

        assert this.equals(other);

        return 0;
    }

    public double getPriority() {
        return priority;
    }

    public void setPriority(double priority) {
        this.priority = priority;
    }
}
