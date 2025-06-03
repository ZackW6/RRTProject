package Canvas.Pathing.RRT;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import Canvas.Shapes.Circle;
import Canvas.Shapes.Line;
import Canvas.Shapes.Obj;
import Canvas.Util.DrawingAccessable;
import Canvas.Util.Vector2D;

public class Node extends Vector2D implements DrawingAccessable{

    private double cost = Double.POSITIVE_INFINITY;
    protected Node parent = null;

    private Circle circle = new Circle(0, 0, 5, Color.BLUE, true);

    public Node(double x, double y, Node parent, boolean child) {
        super(x, y);
        this.parent = parent;

        circle.setPosition(x-circle.getWidth()/2,y-circle.getHeight()/2);
    }

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

    //TODO problem point likely
    public void setParent(Node parent){
        if (parent != null){
            setParent(parent, parent.getCost() + parent.distanceTo(this));
            return;
        }
        this.parent = parent;
        cost = Double.MAX_VALUE;
    }

    public void setParent(Node parent, double cost){
        this.parent = parent;
        this.cost = cost;
    }

    /**
     * whether the node, in any of its lineage, has a relative of this node
     * @param possibleParent
     * @return
     */
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
    /**
     * Much more accurate than taking stored cost, but being run constantly is expensive, so the most recent calculation is stored in "getStoredCost()"
     *  */ 
    public double getCost(){
        double cost = 0;
        Node temp = new Node(this);

        int count = 0;

        while (temp.getParent() != null){
            count++;
            cost += temp.distanceTo(temp.getParent());
            temp = temp.getParent();
            if (count > 1000){
                System.out.println("NOO");
                cost = Double.MAX_VALUE;
                break;
            }
        }
        
        this.cost = cost;
        return cost;
    }

    public double getStoredCost(){
        return cost;
    }
}
