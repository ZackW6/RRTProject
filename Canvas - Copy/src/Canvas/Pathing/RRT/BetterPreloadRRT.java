package Canvas.Pathing.RRT;

import Canvas.Pathing.RRT.RRTHelperBase.Field;
import Canvas.Shapes.PolyShape;
import Canvas.Shapes.VisualJ;
import Canvas.Util.Vector2D;
import Canvas.Util.Maps.KDTree;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

public class BetterPreloadRRT implements RRTBase{

    
    KDTree<Obstacle> obstacles = new KDTree<>();

    ArrayList<Obstacle> staticObstacles = new ArrayList<>();

    HashMap<Obstacle, ArrayList<Node>[]> dynamicObstacles = new HashMap<>();

    ArrayList<Node> killedNodes = new ArrayList<>();
    ArrayList<Node> brokenNodes = new ArrayList<>();

    HashMap<Node, ArrayList<Node>> nodes = new HashMap<>();
    
    PolyShape drawing = new PolyShape(0, 0);
    Field field;
    PolyShape path = null;

    Node start = new Node(850,450);
    Node goal = new Node(1100,650);

    VisualJ vis;

    /**
     * this list of obstacles should only be used for either loaded from a file, or something else, ask zack
     * @param vis
     * @param field
     * @param obstacles
     */
    public BetterPreloadRRT(VisualJ vis, Field field, List<Obstacle> obstacles){
        this.field = field;
        vis.add(drawing);
        this.vis = vis;
        setObstacles(obstacles);
    }

    private void compute() {
        if (!nodes.containsKey(goal) || !nodes.containsKey(start)){
            return;
        }

        for (Node node : nodes.get(start)){
            node.setParent(start);
        }

        for (int i = 0; i < 10; i++){
            nodes.forEach((node, visibleNodes) ->{
                double nodeCost = Double.POSITIVE_INFINITY;
                for (Node visable : visibleNodes){
                    if (!visable.isDescendedOf(start) || visable.equals(start)){
                        continue;
                    }
                    double visCost = visable.getCost();

                    if (node.isDescendedOf(this.start)){
                        nodeCost = node.getCost();
                    }

                    if (visCost + node.distanceTo(visable) > nodeCost){
                        continue;
                    }
                    
                    if (visable.isDescendedOf(node)){
                        continue;
                    }

                    node.setParent(visable);
                }
            });
        }

        if (path != null){
            drawing.remove(path);
        }

        if (!goal.isDescendedOf(start) || collidesObstacle(start,start)){
            return;
        }

        path = getPath(goal);
        drawing.add(path);
    }

    @Override
    public  void setStart(Vector2D start) {
        try {
            Thread.sleep((long).02);
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        this.start.setPosition(start.x, start.y);
        nodes.get(this.start).clear();
        nodes.forEach((node, visibleNodes)->{
            if (!node.equals(this.start) && !collidesObstacle(node, this.start)){
                nodes.get(this.start).add(node);
                if (!visibleNodes.contains(this.start)){
                    visibleNodes.add(this.start);
                }
            }else{
                if (visibleNodes.contains(this.start)){
                    visibleNodes.remove(this.start);
                    node.setParent(null);
                }
            }
        });
        compute();
    }

    @Override
    public  void setGoal(Vector2D goal) {
        this.goal.setPosition(goal.x, goal.y);
        nodes.get(this.goal).clear();
        this.goal.setParent(null);
        nodes.forEach((node, visibleNodes)->{
            if (!node.equals(this.goal) && !collidesObstacle(node, this.goal)){
                if (!visibleNodes.contains(this.goal)){
                    visibleNodes.add(this.goal);
                    nodes.get(this.goal).add(node);
                }else{
                    nodes.get(this.goal).add(node);
                }
            }else{
                if (visibleNodes.contains(this.goal)){
                    visibleNodes.remove(this.goal);
                }
            }
        });
        compute();
    }

    /**
     * in this case it is a complete reset, but it is fast enough that if you are using a grid based obstacle system you should use this
     */
    @Override
    public  void setObstacles(List<Obstacle> obstacles) {
        staticObstacles.clear();
        nodes.clear();
        this.obstacles.clear();
        dynamicObstacles.clear();
        drawing.getArray().clear();
        
        nodes.put(start, new ArrayList<>());

        nodes.put(goal, new ArrayList<>());
        goal.setParent(null);
        
        path = null;
        staticObstacles = new ArrayList<>(simplifyObstacles(obstacles));
        drawing.getArray().addAll(staticObstacles);

        if (staticObstacles.isEmpty()){
            return;
        }

        Obstacle closest = staticObstacles.get(0);
        for (Obstacle obstacle : staticObstacles){
            if (obstacle.distanceTo(Vector2D.of(field.x()/2,field.y()/2)) < closest.distanceTo(Vector2D.of(field.x()/2,field.y()/2))){
                closest = obstacle;
            }
        }

        ArrayList<Obstacle> mixedObstacles = new ArrayList<>(staticObstacles);
        Collections.shuffle(mixedObstacles);

        mixedObstacles.remove(closest);
        this.obstacles.add(closest);
        this.obstacles.addAll(mixedObstacles);

        for (Obstacle obstacle : staticObstacles){
            List<Node> workableNodes = findCornerNodes(obstacle);
            for (Node node : workableNodes){
                nodes.put(node, new ArrayList<>());
            }
        }
        
        nodes.forEach((node, visibleNodes) ->{
            drawing.add(node.getCircle());
            nodes.forEach((visible, dontCare) ->{
                if (!node.equals(visible) && !collidesObstacle(node, visible)){
                    visibleNodes.add(visible);
                }
            });
        });

        compute();
    }



    @Override
    public void process() {
        if (toBeStart.x != start.x || toBeStart.y != start.y){
            setStart(toBeStart);
        }
        if (toBeGoal.x != goal.x || toBeGoal.y != goal.y){
            setGoal(toBeGoal);
        }
        obstacleMaxBoundsCalculation();
    }

    @Override
    public void scheduleObstacles(List<Obstacle> obstacles) {
        setObstacles(obstacles);
    }

    Node toBeStart = new Node(start);
    @Override
    public void scheduleStart(Vector2D start) {
        toBeStart.setPosition(start.x, start.y);
    }

    Node toBeGoal = new Node(goal);
    @Override
    public void scheduleGoal(Vector2D goal) {
        toBeGoal.setPosition(goal.x, goal.y);
    }

    @Override
    public void addObstacles(List<Obstacle> obstacles) {
        for (Obstacle obstacle : obstacles){
            addObstacle(obstacle);
        }
    }

    private void addObstacle(Obstacle obstacle){
        obstacles.add(obstacle);
        ArrayList<Node>[] killedBrokenNodes = new ArrayList[3];
        killedBrokenNodes[0] = new ArrayList<>();
        killedBrokenNodes[1] = new ArrayList<>();
        killedBrokenNodes[2] = new ArrayList<>();
        ArrayList<Node> nodesToKill = new ArrayList<>();
        ArrayList<Node> brokenNodes = killedBrokenNodes[1];
        ArrayList<Node> createdNodes = killedBrokenNodes[2];

        drawing.add(obstacle);
        
        for (Node node : nodes.keySet()){
            ArrayList<Node> toRemove = new ArrayList<>();
            if (obstacle.didCollide(node, node)){
                if (!node.equals(goal) && !node.equals(start)){
                    // nodes.remove(node);
                    drawing.remove(node.getObj());
                    nodesToKill.add(node);
                }
                
                for (Node nodeR : nodes.get(node)){
                    if (!nodes.containsKey(nodeR)){
                        continue;
                    }
                    nodes.get(nodeR).remove(node);
                }
                continue;
            }
            for (Node visible : nodes.get(node)){
                if (nodes.containsKey(visible) && obstacle.didCollide(node, visible)){
                    nodes.get(visible).remove(node);
                    toRemove.add(visible);

                    brokenNodes.add(visible);
                    
                    node.setParent(null);
                    visible.setParent(null);
                }
            }
            nodes.get(node).removeAll(toRemove);
        }

        killedNodes.addAll(nodesToKill);
        for (int i = 0; i < nodesToKill.size(); i++){
            nodes.remove(nodesToKill.get(i));
        }

        dynamicObstacles.put(obstacle, killedBrokenNodes);

        List<Node> workingNodes = findCornerNodesDynamic(obstacle);
        createdNodes.addAll(workingNodes);
        for (Node node : workingNodes){
            nodes.put(node, new ArrayList<>());
            drawing.add(node.getCircle());
            for (Node potentialVis : nodes.keySet()){
                if (!node.equals(potentialVis) && !collidesObstacle(node, potentialVis)){
                    nodes.get(node).add(potentialVis);
                    if (!workingNodes.contains(potentialVis)){
                        nodes.get(potentialVis).add(node);
                    }
                }
            }
        }

        compute();
    }

    public List<Node> findCornerNodesDynamic(Obstacle obstacle){
        Vector2D corner1 = obstacle.getCoords();
        Vector2D corner2 = new Vector2D(corner1.x + obstacle.getWidth(), corner1.y);
        Vector2D corner3 = new Vector2D(corner1.x, corner1.y + obstacle.getHeight());
        Vector2D corner4 = new Vector2D(corner1.x + obstacle.getWidth(), corner1.y + obstacle.getHeight());
        ArrayList<Node> workingNodes = new ArrayList<>();

        Node node1 = new Node(corner1.add(Vector2D.of(-.001,-.001)));
        node1.getCircle().setColor(Color.GREEN);
        if (!collidesObstacle(new Node(corner1.add(Vector2D.of(-.001,-.001))), new Node(corner1.add(Vector2D.of(-.001,-.001))))){
            workingNodes.add(node1);
        }else{
            killedNodes.add(node1);
        }
        Node node2 = new Node(corner2.add(Vector2D.of(.001,-.001)));
        node2.getCircle().setColor(Color.GREEN);
        if (!collidesObstacle(new Node(corner2.add(Vector2D.of(.001,-.001))), new Node(corner2.add(Vector2D.of(.001,-.001))))){
            workingNodes.add(node2);
        }else{
            killedNodes.add(node2);
        }
        Node node3 = new Node(corner3.add(Vector2D.of(-.001,.001)));
        node3.getCircle().setColor(Color.GREEN);
        if (!collidesObstacle(new Node(corner3.add(Vector2D.of(-.001,.001))), new Node(corner3.add(Vector2D.of(-.001,.001))))){
            workingNodes.add(node3);
        }else{
            killedNodes.add(node3);
        }
        Node node4 = new Node(corner4.add(Vector2D.of(.001,.001)));
        node4.getCircle().setColor(Color.GREEN);
        if (!collidesObstacle(new Node(corner4.add(Vector2D.of(.001,.001))), new Node(corner4.add(Vector2D.of(.001,.001))))){
            workingNodes.add(node4);
        }else{
            killedNodes.add(node4);
        }
        return workingNodes;
    }

    public void removeLastDynamic(){
        if (dynamicObstacles.keySet().size() > 0){
            removeObstacle(dynamicObstacles.keySet().toArray(new Obstacle[0])[0]);
        }
    }

    @Override
    public void removeObstacles(List<Obstacle> obstacles) {
        for (Obstacle obstacle : obstacles){
            removeObstacle((Obstacle)obstacle);
        }
    }

    private void removeObstacle(Obstacle obstacle){
        drawing.remove(obstacle);
        obstacles.remove(obstacle);

        //Delete all references to the created nodes by this obstacle
        for (Node node : dynamicObstacles.get(obstacle)[2]){
            drawing.remove(node.getObj());
            nodes.remove(node);
            
            nodes.forEach((key, data)->{
                data.remove(node);
            });
        }
        killedNodes.removeAll(dynamicObstacles.get(obstacle)[2]);
        //Delete the references from other obstacles too
        dynamicObstacles.forEach((obst, deleted)->{
            deleted[1].removeAll(dynamicObstacles.get(obstacle)[2]);
        });

        for (Node node : dynamicObstacles.get(obstacle)[2]){
            nodes.remove(node);
        }

        //Now add back all of the deleted nodes, or try to, with connections
        ArrayList<Node> toRemove = new ArrayList<>();
        for (Node node : killedNodes){
            System.out.println("AHHH");
            if (!collidesObstacle(node, node)){
                ArrayList<Node> visibleNodes = new ArrayList<>();
                nodes.put(node, visibleNodes);
                toRemove.add(node);
                drawing.add(node.getCircle());
                nodes.forEach((visible, dontCare) ->{
                    if (!node.equals(visible) && !collidesObstacle(node, visible)){
                        visibleNodes.add(visible);
                    }
                });
            }
        }
        killedNodes.removeAll(toRemove);

        //Now check back with all the ones that you destroyed the connections of, and if they are still alive, try and reconnect them
        for (Node node : dynamicObstacles.get(obstacle)[1]){
            if (nodes.containsKey(node)){
                ArrayList<Node> visibleNodes = new ArrayList<>();
                nodes.put(node, visibleNodes);
                nodes.forEach((visible, dontCare) ->{
                    if (!node.equals(visible) && !collidesObstacle(node, visible)){
                        visibleNodes.add(visible);
                    }
                });
            }
        }

        dynamicObstacles.remove(obstacle);
        compute();
    }

    /**
     * useless here
     */
    @Override
    public void setMinimumDistance(double dist) {
        
    }

    /**
     * useless here
     */
    @Override
    public void setMaxStepDist(double dist) {
        
    }

    @Override
    public void setDrawingCoords(double x, double y) {
        drawing.setPosition(x,y);
    }

    /**
     * useless here
     */
    @Override
    public void setBias(double bias) {
        
    }

    @Override
    public Field getField() {
        return this.field;
    }

    /**
     * in this case only static obstacles
     */
    @Override
    public List<Obstacle> getObstacles() {
        return List.copyOf(staticObstacles);
    }

    @Override
    public KDTree<Obstacle> getKDTreeObstacles(){
        return obstacles;
    }

    @Override
    public List<Node> getNodes() {
        return List.copyOf(nodes.keySet());
    }

    @Override
    public Node getStart() {
        return start;
    }

    @Override
    public Node getGoal() {
        return goal;
    }

    /**
     * useless in this
     */
    @Override
    public double getMinimumDist() {
        return 0;
    }

    /**
     * useless in this
     */
    @Override
    public double getMaxStepSize() {
        return 0;
    }

    @Override
    public Vector2D getDrawingCoords() {
        return drawing.getCoords();
    }

    /**
     * useless in this
     */
    @Override
    public double getBias() {
        return 0;
    }

    @Override
    public void delete() {
        this.drawing.getArray().clear();
        this.nodes.clear();
        staticObstacles.clear();
        path = null;
        start = new Node(850,450);
        goal = new Node(1100,650);
        vis.remove(drawing);
    }

    /**
     * unused
     */
    @Override
    public void prune(int max) {
        
    }


    double[] maxBounds = new double[]{0,0};

    public double[] obstacleMaxBounds() {
        return maxBounds;
    }

    public void obstacleMaxBoundsCalculation() {
        double[] obstacleDimensions = new double[]{0,0};
        getKDTreeObstacles().traverseNodes(obstacle -> {
            if (obstacle.getWidth() > obstacleDimensions[0]){
                obstacleDimensions[0] = obstacle.getWidth();
            }
            if (obstacle.getHeight() > obstacleDimensions[1]){
                obstacleDimensions[1] = obstacle.getHeight();
            }
        });
        maxBounds = obstacleDimensions;
    }
    
}
