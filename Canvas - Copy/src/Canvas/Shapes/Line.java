package Canvas.Shapes;
import java.awt.*;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;

import Canvas.Util.ArrMath;
import Canvas.Util.Vector2D;

public class Line extends Polygoni{

    private Vector2D[] ends = new Vector2D[2];
    private double lineWidth = 0;

    public Line(double X,double Y,double[] arrintx,double[] arrinty, Color Color,double LineWidth){
        super(X,Y,arrintx,arrinty,Color,true);
        this.lineWidth = LineWidth;
        remakeLine(arrintx, arrinty, LineWidth);
        ends[0] = Vector2D.of(arrintx[0],arrinty[0]);
        ends[1] = Vector2D.of(arrintx[1],arrinty[1]);
    }

    public Line(double X,double Y,List<Vector2D> points, Color Color,double LineWidth){
        super(X,Y,points,Color,true);
        this.lineWidth = LineWidth;
        remakeLine(points, LineWidth);
        ends[0] = points.get(0);
        ends[1] = points.get(1);
    }

    private void remakeLine(List<Vector2D> points, double lineWidth) {
        double[] arrintx = new double[2];
        double[] arrinty = new double[2];
        for (int i = 0; i < 2; i++){
            arrintx[i] = points.get(i).x;
            arrinty[i] = points.get(i).y;
        }
        remakeLine(arrintx, arrinty, lineWidth);
    }

    private void remakeLine(double[] arrintx, double[] arrinty, double LineWidth){
        double slope = (arrinty[1]-arrinty[0])/(arrintx[1]-arrintx[0]);

        double perpendicular = -1.0/(slope+0.00000000001);

        double movex = Math.sqrt((Math.pow(LineWidth/2,2))/(1+Math.pow(perpendicular,2)));
        double movey = movex * perpendicular;

        points.clear();
        points.add(Vector2D.of(arrintx[0]+movex,arrinty[0]+movey));
        points.add(Vector2D.of(arrintx[0]-movex,arrinty[0]-movey));
        points.add(Vector2D.of(arrintx[1]-movex,arrinty[1]-movey));
        points.add(Vector2D.of(arrintx[1]+movex,arrinty[1]+movey));
    }
    
    public void setVertexPos(int indexOfPoint, double x, double y){
        ends[indexOfPoint] = Vector2D.of(x, y);
        remakeLine(List.of(ends), this.lineWidth);
    }

    public void setLineWidth(double lineWidth){
        this.lineWidth = lineWidth;
        remakeLine(List.of(ends), this.lineWidth);
    }

    public double getLineWidth(){
        return lineWidth;
    }

    public List<Vector2D> vertices(){
        return List.of(ends);
    }
}