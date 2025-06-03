package Canvas.Pathing.RRT;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.function.ToDoubleFunction;

import Canvas.Commands.InstantCommand;
import Canvas.Shapes.Circle;
import Canvas.Shapes.PolyShape;
import Canvas.Shapes.VisualJ;
import Canvas.Util.Profile;
import Canvas.Util.Vector2D;
import Canvas.Util.Maps.KDTree;
import Canvas.Util.Maps.KDTree.KDNode;

public class RRTStar extends RRTHelperBase {

    double bestCost = Double.POSITIVE_INFINITY;

    protected boolean isFinished = false;
    protected VisualJ vis;
    List<PolyShape> paths = new ArrayList<>();

    /**
     * RRT which expands to find the goal, and then continues to improve cost
     * @param vis
     * @param field
     */
    public RRTStar(VisualJ vis, Field field) {
        super(vis, field);
        this.vis = vis;
    }

    InstantCommand reRouteCommand = new InstantCommand(()->{});

    @Override
    public void process(){
        obstacleMaxBoundsCalculation();
        for (int i = 0; i < initActions.length; i++){
            initActions[i].run();
            initActions[i] = ()->{};
        }
        
        runAction();
    }

    private Node lastGoal = goal;

    @Override
    protected void runAction() {

        //This is the synchronous running of a reroute, which can take a significant amount of time, on a thread.
        //This allows the user to keep making changes, which will eventually be upheld when this command finishes.
        if (reRouteCommand.isRunning()){
            return;
        }else if (reRouteThread != null){
            System.out.println("CONT");
            reRouteCommand = new InstantCommand(reRouteThread);
            reRouteCommand.schedule();
            reRouteThread = null;
            return;
        }

        if (goal.getParent() == null || lastGoal == null || lastGoal.x != goal.x || lastGoal.y != goal.y){
            setGoalProcess();
        }

        Node randomPoint = getRandomPoint(getBias());
        Node nearestPoint = getNearestPoint(randomPoint);
        Node newPoint = getNewPoint(randomPoint, nearestPoint);

        if (!isFinished()) {
            newPoint.setParent(nearestPoint);

            List<Node> nearbyNodes = getNearbyNodes(newPoint);

            double currentCost = Double.POSITIVE_INFINITY;

            boolean gotParent = false;
            ArrayList<Node> confirmedNodes = new ArrayList<>();

            for (Node nearbyNode : nearbyNodes) {
                
                double potentialCost = nearbyNode.getCost() + newPoint.distanceTo(nearbyNode);

                if (potentialCost < currentCost && !collidesObstacle(newPoint, nearbyNode)) {
                    currentCost = potentialCost;
                    gotParent = true;
                    confirmedNodes.add(nearbyNode);
                    newPoint.setParent(nearbyNode, potentialCost);
                }
            }

            if (!gotParent){
                return;
            }
            drawing.add(newPoint.getCircle());
            nodes.add(newPoint);
            
            for (Node node : confirmedNodes) {
                if (newPoint.getStoredCost() + newPoint.distanceTo(node) < node.getStoredCost()) {
                    node.setParent(newPoint, newPoint.getStoredCost() + newPoint.distanceTo(node));
                }
            }
            
            Node goalSight = new Node(goal, newPoint);

            if (!collidesObstacle(goalSight)){

                double potentialCost = newPoint.getStoredCost() + newPoint.distanceTo(goal);
                if (potentialCost < bestCost){
                    goal.setParent(newPoint, potentialCost);
                }
            }
        }

        double cost = goal.getCost();

        if (!goal.isDescendedOf(start) || collidesObstacle(goal, goal) || collidesObstacle(start, start)){
            if (paths.size()>0){
                drawing.remove(paths.get(0));
                paths.clear();
            }
            cost = Double.POSITIVE_INFINITY;
            return;
        }

        if (cost < bestCost){
            bestCost = cost;
            if (paths.size()>0){
                drawing.remove(paths.get(0));
                paths.clear();
            }
            paths.add(getPath(this.goal));
            drawing.add(paths.get(0));
        }

        if (paths.size() > 0){
            drawing.moveIndex(paths.get(0),drawing.getArray().size()-3);
        }
        
        drawing.moveIndex(goal.getCircle(),drawing.getArray().size()-1);
        drawing.moveIndex(start.getCircle(),drawing.getArray().size()-2);
        // capNodeCount(1000);
        prune(5000);
    }

    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public  void setGoal(Vector2D goal){
        bestCost = Double.POSITIVE_INFINITY;
        this.goal.setParent(null);
        this.goal.setPosition(goal.x, goal.y);
    }

    public void setGoalProcess(){
        for (Node node : getNearbyNodes(this.goal)){
            double potentialCost = node.getCost() + node.distanceTo(goal);
            if (potentialCost < bestCost && !collidesObstacle(node, this.goal)){
                this.goal.setParent(node);
                bestCost = this.goal.getCost();
            }
        }
        if (this.goal.getParent() != null){
            if (paths.size()>0){
                drawing.remove(paths.get(0));
                paths.clear();
            }
            paths.add(getPath(this.goal));
            drawing.add(paths.get(0));
        }
    }

    @Override
    public  void setObstacles(List<Obstacle> obstacles) {
        super.setObstacles(obstacles);
        bestCost = Double.POSITIVE_INFINITY;
        goal.setParent(null);
        if (paths.size()>0){
            drawing.remove(paths.get(0));
            paths.clear();
        }
    }

    Runnable reRouteThread;

    @Override
    public void setStart(Vector2D start) {
        this.start.getCost();
        this.start.setPosition(start.x, start.y);

        setGoalProcess();
        
        reRouteThread = ()->{
            if (collidesObstacle(this.start, this.start)){
                return;
            }
            ArrayList<Node> children = new ArrayList<>();
            ArrayList<Node> toFix = new ArrayList<>();
            nodes.traverseNodes((node)->{
                node.setParent(null);
                toFix.add(node);
            });

            for (Node nearbyNode : getNearbyNodes(this.start, 25)) {
                if (!nearbyNode.equals(this.start) && !collidesObstacle(this.start, nearbyNode)){
                    children.add(nearbyNode);
                    nearbyNode.setParent(this.start);
                }
            }
            if (children.size() == 0){
                return;
            }

            reRoute(children, toFix, -1);
            bestCost = Double.POSITIVE_INFINITY;
            setGoalProcess();
        };
    }

    @Override
    public void addObstacles(List<Obstacle> obstacles) {
        addObstacle(obstacles.get(0));
        drawing.getArray().addAll(obstacles);
        // TODO Auto-generated method stub
        setStart(start);
    }

    public void addObstacle(Obstacle obstacle){
        obstacles.add(obstacle);
        for (Node node : getNearbyNodes(new Node(obstacle.getCoords()))){
            if (collidesObstacle(node, node)){
                nodes.remove(node);
                drawing.remove(node.getCircle());
            }
        }
    }

    @Override
    public void removeObstacles(List<Obstacle> obstacles) {
        this.obstacles.removeAll(obstacles);
        drawing.getArray().removeAll(obstacles);
        setStart(start);
    }

    public void reRoute(List<Node> children, List<Node> toFix, int lastAmount){
        
        if (children.size() == 0){
            return;
        }
        int newAmount = toFix.size();
        if (lastAmount == newAmount){
            for (Node n : toFix){
                if (n.equals(start) || n.equals(goal)){
                    continue;
                }
                drawing.remove(n.getObj());
                nodes.remove(n);
            }
            return;
        }
        
        HashMap<Node, Integer> newChildren = new HashMap<>();
        for (Node child : children){
            
            if (child.equals(goal)){
                continue;
            }
            double childCost = child.getCost();
            for (Node near : getNearbyNodes(child)){
                if (near.equals(child) || near.equals(goal)){
                    continue;
                }
                
                if (childCost + near.distanceTo(child) < near.getStoredCost() && !collidesObstacle(child, near)){
                    if (near.getParent() == null){
                        newChildren.put(near, 0);
                    }
                    near.setParent(child);
                }else if(near.distanceTo(child) + (near.getStoredCost() == Double.MAX_VALUE ? childCost : Double.MAX_VALUE) < childCost && !collidesObstacle(child, near)){
                    child.setParent(near);
                }else{
                    continue;
                }
                toFix.remove(near);
            }
        }
        
        reRoute(List.copyOf(newChildren.keySet()), toFix, newAmount);
    }

    // @Override
    // public  void setStart(Vector2D start){
    //     // LinkedList<Node> keepNodes = new LinkedList<>(nodes.toList());
    //     // keepNodes.remove(this.start);
    //     goal.setParent(null);
    //     ArrayList<Node> goodNodes = new ArrayList<>();
    //     for (Node node : nodes.toList()){
    //         if (!node.equals(this.start) && !collidesObstacle(node, this.start)){
    //             node.setParent(this.start);
    //             goodNodes.add(node);
    //         }
    //     }
    //     nodes.clear(drawing);

    //     this.start.setPosition(start.x, start.y);

    //     drawing.add(this.start.getCircle());
    //     for (Node node : goodNodes){
    //         drawing.add(node.getObj());
    //     }
    //     nodes.add(this.start);
    //     nodes.addAll(goodNodes);

    //     // nodes.addAll(keepNodes);

        

    //     // Map<Node, List<Node>> nearbyNodesCache = new HashMap<>();

    //     // if (collidesObstacle(this.start, this.start)){
    //     //     return;
    //     // }

    //     // for (Node node : keepNodes){
            
    //     //     if (!node.equals(this.start) && node.getParent().equals(this.start) && collidesObstacle(node)){
    //     //         double currentCost = Double.POSITIVE_INFINITY;

    //     //         List<Node> nearbyNodes = nearbyNodesCache.computeIfAbsent(node, this::getNearbyNodes);

    //     //         for (Node potentialParent : nearbyNodes){
                    
    //     //             double potentialCost = potentialParent.getCost() + potentialParent.distanceTo(node);
    //     //             if (potentialCost > currentCost || potentialParent.isDescendedOf(node)){
    //     //                 continue;
    //     //             }

    //     //             if (!collidesObstacle(node, potentialParent)){
                        
    //     //                 node.setParent(potentialParent);
    //     //                 currentCost = potentialCost;
                        
    //     //             }
                    
    //     //         }
    //     //     }
    //     // }

    //     // for (Node node : getNearbyNodes(this.start,calculateRadius() * 3)){
    //     //     if (!node.equals(this.start) && !collidesObstacle(node, this.start)){
    //     //         node.setParent(this.start);
    //     //     }
    //     // }

    //     if (goal.getParent() == null){
    //         bestCost = Double.POSITIVE_INFINITY;
    //         return;
    //     }
    //     if (paths.size()>0){
    //         drawing.remove(paths.get(0));
    //         paths.clear();
    //     }

    //     paths.add(getPath(this.goal));
    //     drawing.add(paths.get(0));
    //     bestCost = goal.getCost();
    // }

    protected double getBestCost(){
        return bestCost;
    }

    @Override
    public void prune(int max) {
        List<Node> list = nodes.toList();
        list.remove(start);
        list.remove(goal);
        if (list.size() > max) {
            // Sort nodes by total cost (cost to reach the node + estimated cost to goal)
            Collections.shuffle(list);
            // Prune nodes exceeding the limit
            while (list.size() > max) {
                if (goal.getParent() != null && goal.isDescendedOf(list.get(list.size() - 1))){
                    list.remove(list.size() - 1);
                    continue;
                }

                drawing.remove(list.get(list.size() - 1).getCircle());
                
                nodes.remove(list.get(list.size() - 1));
                list.remove(list.size() - 1);
            }
        }
    }
}