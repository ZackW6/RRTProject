package Canvas.Shapes;
import java.awt.*;

public class Circle extends Oval{ 

    public Circle(double X,double Y,double Diameter, Color Color,boolean tf){   
        super(X,Y,Diameter,Diameter,Color,tf);
        this.width=Diameter;
    }
    /**
     * Set the size of the circle, reset diameter
     */
    public void setSize(int newSize){
        this.width = newSize;
        this.height = newSize;
    }
}