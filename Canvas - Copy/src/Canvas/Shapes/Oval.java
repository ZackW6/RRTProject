package Canvas.Shapes;
import java.awt.*;

public class Oval extends Obj{

    public Oval(double X,double Y,double Width,double Length, Color Color,boolean tf){   
        super(X,Y,Width,Length,Color,tf);
    }

    /**
     * Set width of the oval
     * @param newSize
     */
    public void setWidth(int newSize){
        this.width=newSize;
    }

    /**
     * Set length of the oval
     * @param newSize
     */
    public void setHeight(int newSize){
        this.height = newSize;
    }

    /**
     * Set width of the oval
     * @param newSize
     */
    public void setWidth(double newSize){
        this.width=newSize;
    }

    /**
     * Set length of the oval
     * @param newSize
     */
    public void setHeight(double newSize){
        this.height = newSize;
    }

    /**
     * The oval application of the show method, draws an oval based on instance data
     */
    @Override
    public void show(Graphics2D g2dBuffer, double zoomRatio){
        if (fill){
            g2dBuffer.fillOval((int)((-width/2)*zoomRatio), (int)((-height/2)*zoomRatio), (int)(width*zoomRatio), (int)(height*zoomRatio));
        }else{
            g2dBuffer.drawOval((int)((-width/2)*zoomRatio), (int)((-height/2)*zoomRatio), (int)(width*zoomRatio), (int)(height*zoomRatio));
        }
    }
}