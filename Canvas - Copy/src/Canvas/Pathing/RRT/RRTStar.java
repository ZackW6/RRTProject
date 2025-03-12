package Canvas.Pathing.RRT;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.function.ToDoubleFunction;

import Canvas.Shapes.Circle;
import Canvas.Shapes.PolyShape;
import Canvas.Shapes.VisualJ;
import Canvas.Util.KDTree.KDNode;
import Canvas.Util.KDTree;
import Canvas.Util.Profile;
import Canvas.Util.Vector2D;

public class RRTStar extends RRTHelperBase {

    double bestCost = Double.POSITIVE_INFINITY;

    protected boolean isFinished = false;
    protected VisualJ vis;
    List<PolyShape> paths = new ArrayList<>();

    public RRTStar(VisualJ vis, Field field) {
        super(vis, field);
        this.vis = vis;
    }

    @Override
    protected void runAction() {
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
                    newPoint.setParent(nearbyNode);
                    currentCost = potentialCost;
                    gotParent = true;
                    confirmedNodes.add(nearbyNode);
                    newPoint.cost = potentialCost;
                }
            }

            if (!gotParent){
                return;
            }
            drawing.add(newPoint.getCircle());
            nodes.add(newPoint);
            
            for (Node node : confirmedNodes) {
                if (newPoint.cost + newPoint.distanceTo(node) < node.cost) {
                    node.setParent(newPoint);
                }
            }
            
            Node goalSight = new Node(goal, newPoint);

            if (!collidesObstacle(goalSight)){

                double potentialCost = newPoint.cost + newPoint.distanceTo(goal);
                if (potentialCost < bestCost){
                    goal.setParent(newPoint);
                    goal.cost = potentialCost;
                }
            }
        }

        double cost = goal.getCost();

        if (goal.getParent() == null){
            cost = Double.POSITIVE_INFINITY;
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
        prune(10000);
    }

    protected List<Node> organizeByCost(List<Node> list, Node extraNode){
        ArrayList<Node> sorted = new ArrayList<>(list);
        sorted.sort(new Comparator<Node>() {
            @Override
            public int compare(Node item1, Node item2) {
                if (extraNode == null){
                    return Double.compare(item1.getCost(), item2.getCost());
                }
                return Double.compare(item1.getCost() + item1.distanceTo(extraNode), item2.getCost() + item2.distanceTo(extraNode));
            }
        });
        return sorted;
    }

    protected List<Node> organizeByCost(List<Node> list){
        return organizeByCost(list, null);
    }

    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public  void setGoal(Vector2D goal){
        bestCost = Double.POSITIVE_INFINITY;
        this.goal.setPosition(goal.x, goal.y);
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

    @Override
    public  void setStart(Vector2D start){
        // LinkedList<Node> keepNodes = new LinkedList<>(nodes.toList());
        // keepNodes.remove(this.start);
        goal.setParent(null);
        ArrayList<Node> goodNodes = new ArrayList<>();
        for (Node node : nodes.toList()){
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

        // nodes.addAll(keepNodes);

        

        // Map<Node, List<Node>> nearbyNodesCache = new HashMap<>();

        // if (collidesObstacle(this.start, this.start)){
        //     return;
        // }

        // for (Node node : keepNodes){
            
        //     if (!node.equals(this.start) && node.getParent().equals(this.start) && collidesObstacle(node)){
        //         double currentCost = Double.POSITIVE_INFINITY;

        //         List<Node> nearbyNodes = nearbyNodesCache.computeIfAbsent(node, this::getNearbyNodes);

        //         for (Node potentialParent : nearbyNodes){
                    
        //             double potentialCost = potentialParent.getCost() + potentialParent.distanceTo(node);
        //             if (potentialCost > currentCost || potentialParent.isDescendedOf(node)){
        //                 continue;
        //             }

        //             if (!collidesObstacle(node, potentialParent)){
                        
        //                 node.setParent(potentialParent);
        //                 currentCost = potentialCost;
                        
        //             }
                    
        //         }
        //     }
        // }

        // for (Node node : getNearbyNodes(this.start,calculateRadius() * 3)){
        //     if (!node.equals(this.start) && !collidesObstacle(node, this.start)){
        //         node.setParent(this.start);
        //     }
        // }

        if (goal.getParent() == null){
            bestCost = Double.POSITIVE_INFINITY;
            return;
        }
        if (paths.size()>0){
            drawing.remove(paths.get(0));
            paths.clear();
        }

        paths.add(getPath(this.goal));
        drawing.add(paths.get(0));
        bestCost = goal.getCost();
    }

    protected double getBestCost(){
        return bestCost;
    }

    
    /**
     * with no node applicable returns null
     * @param node
     * @param list
     * @return
     */
    protected Node findBestParent(Node node, List<Node> list){
        double currentCost = Double.POSITIVE_INFINITY;
        Node bestParent = null;
        for (Node nearbyNode : list) {
            
            double potentialCost = nearbyNode.getCost() + node.distanceTo(nearbyNode);

            if (potentialCost < currentCost && !collidesObstacle(node, nearbyNode)) {
                bestParent = nearbyNode;
                currentCost = potentialCost;
                node.cost = potentialCost;
            }
        }
        return bestParent;
    }

    @Override
    public void prune(int max) {
        List<Node> list = nodes.toList();
        if (list.size() > max) {
            // Sort nodes by total cost (cost to reach the node + estimated cost to goal)
            list.sort(Comparator.comparingDouble(node -> node.getCost() + node.distanceTo(goal)));
    
            // Prune nodes exceeding the limit
            while (list.size() > max) {
                drawing.remove(list.get(list.size() - 1).getCircle());
                
                nodes.remove(list.get(list.size() - 1));
                list.remove(list.size() - 1);
            }
        }
    }
}