package Canvas.Shapes;
import java.awt.*;
import java.util.ArrayList;
import java.util.List;

import Canvas.Util.Vector2D;

public class Polygon extends Polygoni{

    public Polygon(double X,double Y,double[] arrintx,double[] arrinty, Color Color,boolean tf){
        super(X,Y,arrintx,arrinty,Color,tf);
    }

    public Polygon(double X,double Y, List<Vector2D> points, Color Color,boolean tf){
        super(X,Y,points,Color,tf);
    }

    /**
     * Add a vertex to the current polygon
     * @param index where to add the vertex
     * @param x
     * @param y
     */
    public void addVertex(int index, double x, double y){
        points.add(index, Vector2D.of(x, y));
        recheck(points);
    }

    /**
     * change the vertex of any index point
     * @param indexOfPoint
     * @param x
     * @param y
     */
    public void setVertexPos(int indexOfPoint, double x, double y){
        points.set(indexOfPoint, Vector2D.of(x, y));
        recheck(points);
    }
}