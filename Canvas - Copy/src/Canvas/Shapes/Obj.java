package Canvas.Shapes;
import java.awt.*;
import java.awt.Rectangle;

import Canvas.Util.DrawingAccessable;
import Canvas.Util.Vector2D;
/**
 * Base class for all shapes of the library
 */
public abstract class Obj implements DrawingAccessable{
    protected Vector2D coords = new Vector2D(0, 0);
    protected double width;
    protected double height;
    protected Color col;
    protected double degree = 0;
    protected boolean fill;
    protected Vector2D addedCoords = new Vector2D(0, 0);
    protected double degree2 = 0;

    /**
     * Base class public for the Obj class, not meant to be implimented
     * @param xcoord
     * @param ycoord
     * @param width
     * @param height
     * @param col
     * @param fill
     */
    protected Obj(double xcoord,double ycoord,double width, double height, Color col,boolean fill){
        this.coords.x=xcoord;
        this.coords.y=ycoord;
        this.col=col;
        this.width=width;
        this.height=height;
        this.fill=fill;
    }

    /**
     * Rotate based on center
     * @param rotateDegree
     */
    public void rotate(double rotateDegree){
        this.degree=rotateDegree;
    }

    /**
     * Add on to current x and y
     * @param moveX
     * @param moveY
     */
    public void move(double moveX,double moveY){
        this.coords.x=moveX+coords.x;
        this.coords.y=moveY+coords.y;
    }

    public double getDegree(){
        return degree;
    }

    public double getRotPointDegree(){
        return degree2;
    }

    /**
     * Change whether or not the shape is filled
     * @param fillOrNo
     */
    public void changeFill(boolean fillOrNo){
        this.fill=fillOrNo;
    }

    /**
     * Rotate the shape around a certain point, will not rotate around self
     * @param x
     * @param y
     * @param deg
     */
    public void rotPoint(double x, double y, double deg){
        this.degree2=deg;
        double radius = Math.sqrt(Math.pow(x-(coords.x+width/2),2)+Math.pow(y-(coords.y+height/2),2));
        double curRad = Math.toDegrees(Math.atan2((coords.y+height/2) - y, (coords.x+width/2) - x));
        addedCoords.x = (int)(x+radius*Math.cos(Math.toRadians(degree2+curRad)))-(int)coords.x-width/2;
        addedCoords.y = (int)(y+radius*Math.sin(Math.toRadians(degree2+curRad)))-(int)coords.y-height/2;
    }

    /**
     * Set the position of the shape
     * @param x
     * @param y
     */
    public void setPosition(double x, double y){
        this.coords.x=x;
        this.coords.y=y;
    }

    /**
     * Set the current color of the shape
     * @param newCol
     */
    public void setColor(Color col){
        this.col=col;
    }

    /**
     * @param g2dBuffer
     */
    public abstract void show(Graphics2D g2dBuffer, double zoomRatio);

    public Vector2D getCoords(){
        return coords;
    }

    public double getWidth(){
        return width;
    }

    public double getHeight(){
        return height;
    }

    @Override
    public Obj getObj(){
        return this;
    }
}
