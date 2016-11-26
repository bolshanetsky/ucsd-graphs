package roadgraph;

import geography.GeographicPoint;


public class Edge {

    private double timeToDriveThrough;

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
        this.timeToDriveThrough = length / getSpeedLimitByRoadType(roadType);
    }


    public static int getSpeedLimitByRoadType(String roadType){
        switch (roadType) {
            case "primary": return 50;
            case "residential": return 30;
            case "motorway": return 90;
            case "tertiary": return 30;
            case "secondary_link": return 40;
            case "motorway_link": return 50;
            default: return 50;
        }
    }

    public double getTimeToDriveThrough() {
        return timeToDriveThrough;
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
