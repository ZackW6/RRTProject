package Canvas.Pathing.RRT;

import java.awt.Color;
import java.awt.Graphics2D;

import Canvas.Shapes.Obj;
import Canvas.Shapes.Rectangle;
import Canvas.Shapes.UserButtons.ClickableButton;
import Canvas.Util.DrawingAccessable;
import Canvas.Util.Point;
import Canvas.Util.Vector2D;

public class Obstacle extends Rectangle implements ClickableButton, Point{

    private boolean currentObstacle;
    public Obstacle(double X, double Y, double Width, double Height, boolean currentObstacle) {
        super(X, Y, Width, Height, Color.RED, false);
        this.currentObstacle = currentObstacle;
        if (currentObstacle){
            this.setColor(Color.RED);
        }else{
            this.setColor(Color.BLUE);
        }
    }

    public void actsAsObstacle(boolean currentObstacle){
        this.currentObstacle = currentObstacle;
        if (currentObstacle){
            this.setColor(Color.RED);
        }else{
            this.setColor(Color.BLUE);
        }
    }

    public void setDisplayColor(Color color){
        this.col = color;
    }

    public void setDisplayFill(boolean fill){
        this.fill = fill;
    }

    public boolean didCollide(Vector2D x, Vector2D y){
        if (!currentObstacle){
            return false;
        }
        return doesLineIntersectRectangle(x, y, coords.x, coords.y, coords.x+width, coords.y+height);
    }

    // Calculate the orientation of the triplet (p, q, r)
    private int orientation(Vector2D p, Vector2D q, Vector2D r) {
        double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);

        if (val == 0) return 0; // Collinear
        return (val > 0) ? 1 : 2; // Clockwise or counterclockwise
    }

    // Check if point q lies on line segment pr
    private boolean onSegment(Vector2D p, Vector2D q, Vector2D r) {
        return q.x <= Math.max(p.x, r.x) && q.x >= Math.min(p.x, r.x) &&
               q.y <= Math.max(p.y, r.y) && q.y >= Math.min(p.y, r.y);
    }

    // Check if two line segments p1q1 and p2q2 intersect
    public boolean doIntersect(Vector2D p1, Vector2D q1, Vector2D p2, Vector2D q2) {
        int o1 = orientation(p1, q1, p2);
        int o2 = orientation(p1, q1, q2);
        int o3 = orientation(p2, q2, p1);
        int o4 = orientation(p2, q2, q1);

        // General case
        if (o1 != o2 && o3 != o4) return true;

        // Special cases
        if (o1 == 0 && onSegment(p1, p2, q1)) return true;
        if (o2 == 0 && onSegment(p1, q2, q1)) return true;
        if (o3 == 0 && onSegment(p2, p1, q2)) return true;
        if (o4 == 0 && onSegment(p2, q1, q2)) return true;

        return false;
    }

    // Check if a line segment intersects a rectangle
    public boolean doesLineIntersectRectangle(Vector2D p1, Vector2D p2, double xmin, double ymin, double xmax, double ymax) {
        Vector2D bottomLeft = new Vector2D(xmin, ymin);
        Vector2D bottomRight = new Vector2D(xmax, ymin);
        Vector2D topLeft = new Vector2D(xmin, ymax);
        Vector2D topRight = new Vector2D(xmax, ymax);

        // Check if either point is inside the rectangle
        if (isPointInsideRectangle(p1, xmin, ymin, xmax, ymax) || 
            isPointInsideRectangle(p2, xmin, ymin, xmax, ymax)) {
            return true;
        }

        // Check intersection with each rectangle edge
        if (doIntersect(p1, p2, bottomLeft, bottomRight)) return true;
        if (doIntersect(p1, p2, topLeft, topRight)) return true;
        if (doIntersect(p1, p2, bottomLeft, topLeft)) return true;
        if (doIntersect(p1, p2, bottomRight, topRight)) return true;

        return false;
    }

    // Check if a point is inside a rectangle
    private boolean isPointInsideRectangle(Vector2D p, double xmin, double ymin, double xmax, double ymax) {
        return p.x >= xmin && p.x <= xmax && p.y >= ymin && p.y <= ymax;
    }

    public boolean isPointInsideRectangle(Vector2D p) {
        return p.x >= this.getX() && p.x <= this.getX() + this.getWidth() && p.y >= this.getY() && p.y <= this.getY() + this.getHeight();
    }

    @Override
    public void runOnClick() {
        
    }

    @Override
    public Vector2D getDimensions() {
        return Vector2D.of(this.getWidth(), this.getHeight());
    }

    @Override
    public double getX() {
        return coords.x;
    }

    @Override
    public double getY() {
        return coords.y;
    }

    public boolean isObstacle(){
        return currentObstacle;
    }
}
