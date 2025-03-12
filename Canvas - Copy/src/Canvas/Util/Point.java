package Canvas.Util;

public interface Point {
    public double getX();
    public double getY();

    public default double distanceTo(Point vector){
        return Math.sqrt(Math.pow(this.getX()-vector.getX(),2)+Math.pow(this.getY()-vector.getY(),2));
    }
}
