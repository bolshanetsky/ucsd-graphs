package roadgraph;

import geography.GeographicPoint;


public class Edge {

    private GeographicPoint fromPoint;
    private GeographicPoint toPoint;
    private String roadName;
    private String roadType;
    private double length;


    public Edge (GeographicPoint fromPoint, GeographicPoint toPoint, String roadName,
                 String roadType, double length) {
        this.fromPoint = fromPoint;
        this.toPoint = toPoint;
        this.roadName = roadName;
        this.roadType = roadType;
        this.length = length;
    }

    public GeographicPoint getToPoint() {
        return this.toPoint;
    }

    public GeographicPoint getFromPoint() {
        return this.fromPoint;
    }

    public String getRoadName(){
        return this.roadName;
    }

    public String getRoadType(){
        return this.roadType;
    }

    public double getLength(){
        return this.length;
    }
}
