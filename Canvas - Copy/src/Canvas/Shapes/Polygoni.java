package Canvas.Shapes;
import java.awt.*;
import java.awt.Polygon;
import java.awt.image.AbstractMultiResolutionImage;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;

import javax.swing.text.View;

import Canvas.Util.ArrMath;
import Canvas.Util.Vector2D;

public abstract class Polygoni extends Obj{

    protected CopyOnWriteArrayList<Vector2D> points;
    protected Vector2D translationVector;


    public Polygoni(double X,double Y,double[] arrintx,double[] arrinty, Color Color,boolean tf){
        super(X,Y,0,0,Color,tf);

        CopyOnWriteArrayList<Vector2D> points = new CopyOnWriteArrayList<>();
        for (int i = 0; i < arrintx.length; i++){
            points.add(Vector2D.of(arrintx[i],arrinty[i]));
        }
        
        recheck(points);
    }

    public Polygoni(double X, double Y, List<Vector2D> points, Color Color,boolean tf){
        super(X,Y,0,0,Color,tf);

        recheck(new CopyOnWriteArrayList<>(points.toArray(new Vector2D[points.size()])));
    }

    protected void recheck(List<Vector2D> points){
        this.points = (CopyOnWriteArrayList<Vector2D>) points;
        Vector2D dimensions = getDimensions(points);

        this.translationVector = getTranslation(points);
        this.width = dimensions.x;
        this.height = dimensions.y;
    }

    private Vector2D getDimensions(List<Vector2D> points){
        double greatx = points.get(0).x;
        double greaty = points.get(0).y;
        double leastx = points.get(0).x;
        double leasty = points.get(0).y;
        for (int i = 1; i < points.size(); i++){
            if (points.get(i).x < leastx){
                leastx = points.get(i).x;
            }
            if (points.get(i).x > greatx){
                greatx = points.get(i).x;
            }
            if (points.get(i).y < leasty){
                leasty = points.get(i).y;
            }
            if (points.get(i).y > greaty){
                greaty = points.get(i).y;
            }
        }
        double width = greatx - leastx;
        double height = greaty - leasty;
        return Vector2D.of(width, height);
    }

    private Vector2D getTranslation(List<Vector2D> points){
        double greatx = points.get(0).x;
        double greaty = points.get(0).y;
        double leastx = points.get(0).x;
        double leasty = points.get(0).y;
        for (int i = 1; i < points.size(); i++){
            if (points.get(i).x < leastx){
                leastx = points.get(i).x;
            }
            if (points.get(i).x > greatx){
                greatx = points.get(i).x;
            }
            if (points.get(i).y < leasty){
                leasty = points.get(i).y;
            }
            if (points.get(i).y > greaty){
                greaty = points.get(i).y;
            }
        }
        double width = (greatx + leastx)/2;
        double height = (greaty + leasty)/2;
        return Vector2D.of(width, height);
    }

    public Vector2D getTranslationVector(){
        return translationVector;
    }

    public List<Vector2D> getPoints(){
        return points;
    }

    /**
     * The Polygon application of the show method, draws a Polygon based on instance data
     */
    @Override
    public void show(Graphics2D g2dBuffer, double zoomRatio){

        int[] arrintx = new int[points.size()];
        int[] arrinty = new int[points.size()];
        for (int i = 0; i < points.size(); i++){
            arrintx[i] = (int)points.get(i).x;
            arrinty[i] = (int)points.get(i).y;
        }
        int[] z=new int[arrintx.length];
        int[] a=new int[arrinty.length];
        
        for (int y=0;y<arrintx.length;y++){
            z[y]=(int) -translationVector.x;
            a[y]=(int) -translationVector.y;
        }
        int[] b=ArrMath.addArrs(arrintx,z);
        int[] c=ArrMath.addArrs(arrinty,a);

        for (int y=0;y<arrintx.length;y++){
            b[y]=(int)(b[y]*zoomRatio);
            c[y]=(int)(c[y]*zoomRatio);
        }

        if (fill){
            g2dBuffer.fillPolygon(b,c,arrintx.length);
        }else{
            g2dBuffer.drawPolygon(b,c,arrintx.length);
        }
    }
}