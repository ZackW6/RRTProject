package Canvas.Pathing.RRT;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import Canvas.Shapes.Circle;
import Canvas.Shapes.Line;
import Canvas.Shapes.Obj;
import Canvas.Util.DrawingAccessable;
import Canvas.Util.Vector2D;

public class Node extends Vector2D implements DrawingAccessable{

    protected double cost = Double.POSITIVE_INFINITY;
    private Node parent = null;

    private boolean isDynamic = false;

    private Circle circle = new Circle(0, 0, 5, Color.BLUE, true);

    public Node(double x, double y, Node parent) {
        super(x, y);
        this.parent = parent;
        circle.setPosition(x-circle.getWidth()/2,y-circle.getHeight()/2);
    }

    public Node(Node copy) {
        super(copy.x, copy.y);
        this.parent = copy.parent;
        circle.setPosition(x-circle.getWidth()/2,y-circle.getHeight()/2);
    }

    public Node(double x, double y){
        super(x, y);
        circle.setPosition(x-circle.getWidth()/2,y-circle.getHeight()/2);
    }

    public Node(Node place, Node parent){
        super(place.x, place.y);
        this.parent = parent;
        circle.setPosition(x-circle.getWidth()/2,y-circle.getHeight()/2);
    }

    public Node(Vector2D vec){
        super(vec.x, vec.y);
        circle.setPosition(x-circle.getWidth()/2,y-circle.getHeight()/2);
    }

    public void setPosition(double x, double y){
        this.x = x;
        this.y = y;
        circle.setPosition(x-circle.getWidth()/2,y-circle.getHeight()/2);
    }

    public Vector2D getVector2D(){
        return Vector2D.of(this.x, this.y);
    }

    public Node getParent(){
        return parent;
    }

    public void setParent(Node parent){
        this.parent = parent;
    }

    public boolean isDescendedOf(Node possibleParent){
        if (this.equals(possibleParent)){
            return true;
        }
        Node temp = new Node(this);
        while (temp != null){
            if (temp.equals(possibleParent)){
                return true;
            }
            temp = temp.getParent();
        }
        return false;
    }

    public void setColor(Color col){
        circle.setColor(col);
    }

    public void setWidth(double width){
        circle.setWidth(width);
        circle.setPosition(x-circle.getWidth()/2,y-circle.getHeight()/2);
    }

    public Circle getCircle(){
        return circle;
    }

    @Override
    public Obj getObj() {
        return getCircle();
    }

    public void setDynamic(boolean dynamic){
        this.isDynamic = dynamic;
    }

    public boolean isDynamic(){
        return isDynamic;
    }

    /**
     * Much more accurate than taking cost directly, needs to be run
     * in certain cases the cost may have been calculated already which is
     * the only reason to use the previous method
     *  */ 
    public double getCost(){
        double cost = 0;
        Node temp = new Node(this);

        while (temp.getParent() != null){
            cost += temp.distanceTo(temp.getParent());
            temp = temp.getParent();
        }
        
        this.cost = cost;
        return cost;
    }
}
