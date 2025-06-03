package Canvas.Pathing.RRT;

import Canvas.Shapes.PolyShape;
import Canvas.Shapes.VisualJ;
import Canvas.Util.Random;
import Canvas.Util.Vector2D;
import java.util.ArrayList;
import java.util.List;

public class RRT extends RRTHelperBase{

    private boolean isFinished = false;

    private double bestCost = Double.POSITIVE_INFINITY;

    PolyShape path = null;

    /**
     * The most simple RRT, builds a path to the goal, but does not improve it
     * @param vis
     * @param field
     */
    public RRT(VisualJ vis, Field field) {
        super(vis, field);
    }

    @Override
    protected void runAction(){
        Node randomPoint = getRandomPoint(getBias());
        Node nearestPoint = getNearestPoint(randomPoint);
        Node newPoint = getNewPoint(randomPoint, nearestPoint);

        if (!collidesObstacle(newPoint) && !isFinished()) {
            
            drawing.add(newPoint.getCircle());
            
            this.nodes.add(newPoint);
            
            if (newPoint.distanceTo(getGoal()) < getMinimumDist() && newPoint.getCost() + newPoint.distanceTo(getGoal()) < bestCost) {
                bestCost = newPoint.getCost() + newPoint.distanceTo(getGoal());
                nodes.add(new Node(getGoal().x, getGoal().y, newPoint));
                // isFinished = true;
                if (path == null){
                    path = getPath(new Node(getGoal().x, getGoal().y, newPoint));
                    drawing.add(path);
                }else{
                    drawing.remove(path);
                    path = getPath(new Node(getGoal().x, getGoal().y, newPoint));
                    drawing.add(path);
                }
            }
        }
        //Do not let the node amount exceed 100000, graphics becomes slower than 20ms at this amout
        prune(100000);
    }

    public boolean isFinished() {
        return isFinished;
    }

    //Move goal, look for parents closeby, but if none delete current displayed path
    @Override
    public  void setGoal(Vector2D goal){
        bestCost = Double.POSITIVE_INFINITY;
        this.goal.setPosition(goal.x, goal.y);
        for (Node node : getNearbyNodes(this.goal,100.0)){
            double potentialCost = node.getCost() + node.distanceTo(goal);
            if (potentialCost < bestCost && !collidesObstacle(node, this.goal)){
                this.goal.setParent(node);
                bestCost = this.goal.getCost();
            }
        }
        if (this.goal.getParent() != null){
            if (path != null){
                drawing.remove(path);
                path = null;
            }
            path = getPath(this.goal);
            drawing.add(path);
        }
    }

    //Reset with new obstacles
    @Override
    public  void setObstacles(List<Obstacle> obstacles) {
        super.setObstacles(obstacles);
        bestCost = Double.POSITIVE_INFINITY;
        goal.setParent(null);
        if (path != null){
            drawing.remove(path);
            path = null;
        }
    }

    //Reset with new start, but look for close nodes of start to keep
    @Override
    public  void setStart(Vector2D start){
        goal.setParent(null);
        ArrayList<Node> goodNodes = new ArrayList<>();
        for (Node node : getNearbyNodes(new Node(start))){
            if (!node.equals(this.start) && !collidesObstacle(node, this.start)){
                node.setParent(this.start);
                goodNodes.add(node);
            }
        }
        nodes.clear(drawing);

        this.start.setPosition(start.x, start.y);

        drawing.add(this.start.getCircle());
        for (Node node : goodNodes){
            drawing.add(node.getObj());
        }
        nodes.add(this.start);
        nodes.addAll(goodNodes);

        if (goal.getParent() == null){
            bestCost = Double.POSITIVE_INFINITY;
            return;
        }
        if (path != null){
            drawing.remove(path);
            path = null;
        }

        path = getPath(this.goal);
        drawing.add(path);
        bestCost = goal.getCost();
    }

    protected double getBestCost(){
        return bestCost;
    }

    @Override
    public void prune(int max) {
        List<Node> list = nodes.toList();
        if (list.size() > max) {    
            // Prune nodes exceeding the limit
            while (list.size() > max) {
                int randInt = Random.randInt(0,list.size()-1);
                drawing.remove(list.get(randInt).getCircle());
                
                nodes.remove(list.get(randInt));
                list.remove(randInt);
            }
        }
    }
}
