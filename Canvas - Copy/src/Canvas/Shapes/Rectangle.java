package Canvas.Shapes;
import java.awt.*;

public class Rectangle extends Polygoni{

    public Rectangle(double X,double Y,double Width,double Height, Color Color,boolean tf){
        super(X,Y,new double []{0,Width,Width,0},new double []{0,0,Height,Height},Color,tf);
    }

    /**
     * Set width of the oval
     * @param newSize
     */
    public void changeWidth(int newSize){
        this.width=newSize;
        resetRect();
    }

    /**
     * Set length of the oval
     * @param newSize
     */
    public void changeHeight(int newSize){
        this.height = newSize;
        resetRect();
    }

    protected void resetRect(){
        points.get(0).x = 0;
        points.get(0).y = 0;
        points.get(1).x = width;
        points.get(1).y = 0;
        points.get(2).x = width;
        points.get(2).y = height;
        points.get(3).x = 0;
        points.get(3).y = height;
    }
}