package Canvas.Pathing.RRT;

import java.awt.Color;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import Canvas.Pathing.RRT.RRTHelperBase.Field;
import Canvas.Shapes.Line;
import Canvas.Shapes.PolyShape;
import Canvas.Shapes.VisualJ;
import Canvas.Util.KDTree;
import Canvas.Util.Vector2D;

public class PreloadRRT implements RRTBase{

    KDTree<ModifiedNode> allNodes = new KDTree<ModifiedNode>();

    KDTree<ModifiedObstacle> obstacles = new KDTree<>();
    ArrayList<ModifiedObstacle> wholeObstacles = new ArrayList<>();

    ModifiedNode start = new ModifiedNode(850,450);
    ModifiedNode goal = new ModifiedNode(850,450);

    VisualJ vis;

    Field field;

    double bestCost = Double.POSITIVE_INFINITY;

    protected PolyShape drawing;

    protected PolyShape path = null;

    class ModifiedNode extends Node{

        ArrayList<ModifiedNode> visibleNodes = new ArrayList<>();
        ModifiedObstacle parentObstacle = null;
        public ModifiedNode(double x, double y) {
            super(x, y);
        }

        public ModifiedNode(Node node) {
            super(node.x, node.y);
        }

        public ModifiedNode(Vector2D vec) {
            super(vec.x, vec.y);
        }

        public void addVisibleNode(ModifiedNode node){
            visibleNodes.add(node);
        }

        public void removeVisibleNode(ModifiedNode node){
            visibleNodes.remove(node);
        }

    }

    class ModifiedObstacle extends Obstacle{
        public ArrayList<ModifiedNode> removedNodes = new ArrayList<>();
        public ArrayList<ModifiedNode> childNodes = new ArrayList<>();
        public ModifiedObstacle(double X, double Y, double Width, double Height, boolean currentObstacle) {
            super(X, Y, Width, Height, currentObstacle);
        }
        public ModifiedObstacle(Obstacle obstacle) {
            super(obstacle.getX(), obstacle.getY(), obstacle.getWidth(), obstacle.getHeight(), obstacle.isObstacle());
        }
    }

    public PreloadRRT(VisualJ vis, Field field, List<Obstacle> obstacles) {
        wholeObstacles = new ArrayList<>();
        for (Obstacle obstacle : obstacles) {
            wholeObstacles.add(new ModifiedObstacle(obstacle));
        }

        this.vis = vis;
        
        ArrayList<ModifiedObstacle> workingObstacles = new ArrayList<>();
        for (ModifiedObstacle obstacle : wholeObstacles){
            if (obstacle.isObstacle()){
                workingObstacles.add(obstacle);
            }
        }
        this.field = field;
        this.drawing = new PolyShape(0, 0);
        vis.add(drawing);

        drawing.getArray().addAll(wholeObstacles);
        this.obstacles.addAll(workingObstacles);

        if (!collidesObstacle(start, start)){
            allNodes.add(start);
        }

        if (!collidesObstacle(goal, goal)){
            allNodes.add(goal);
        }

        for (ModifiedObstacle obstacle : workingObstacles){
            List<ModifiedNode> workableNodes = findCornerNodes(obstacle);
            allNodes.addAll(workableNodes);
        }
        
        for (ModifiedNode node : allNodes.toList()){
            drawing.add(node.getCircle());
            for (ModifiedNode potentialVis : allNodes.toList()){
                if (!node.equals(potentialVis) && !collidesObstacle(node, potentialVis)){
                    node.addVisibleNode(potentialVis);
                }
            }
        }

        compute();
    }

    public void process(){
        // if (paths.size()>0){
        //     drawing.remove(paths.get(0));
        //     paths.clear();
        // }

        // paths.add(getPath(goal));
        // drawing.add(paths.get(0));
    }

    public List<ModifiedNode> findCornerNodes(ModifiedObstacle obstacle){
        Vector2D corner1 = obstacle.getCoords();
        Vector2D corner2 = new Vector2D(corner1.x + obstacle.getWidth(), corner1.y);
        Vector2D corner3 = new Vector2D(corner1.x, corner1.y + obstacle.getHeight());
        Vector2D corner4 = new Vector2D(corner1.x + obstacle.getWidth(), corner1.y + obstacle.getHeight());
        ArrayList<ModifiedNode> workingNodes = new ArrayList<>();

        ModifiedNode node1 = new ModifiedNode(corner1.add(Vector2D.of(-.001,-.001)));
        obstacle.childNodes.add(node1);
        node1.parentObstacle = obstacle;
        if (!collidesObstacle(new Node(corner1.add(Vector2D.of(-.001,-.001))), new Node(corner1.add(Vector2D.of(-.001,-.001))))){
            workingNodes.add(node1);
        }
        ModifiedNode node2 = new ModifiedNode(corner2.add(Vector2D.of(.001,-.001)));
            obstacle.childNodes.add(node2);
            node2.parentObstacle = obstacle;
        if (!collidesObstacle(new Node(corner2.add(Vector2D.of(.001,-.001))), new Node(corner2.add(Vector2D.of(.001,-.001))))){
            workingNodes.add(node2);
        }
        ModifiedNode node3 = new ModifiedNode(corner3.add(Vector2D.of(-.001,.001)));
            obstacle.childNodes.add(node3);
            node3.parentObstacle = obstacle;
        if (!collidesObstacle(new Node(corner3.add(Vector2D.of(-.001,.001))), new Node(corner3.add(Vector2D.of(-.001,.001))))){
            workingNodes.add(node3);
        }
        ModifiedNode node4 = new ModifiedNode(corner4.add(Vector2D.of(.001,.001)));
        obstacle.childNodes.add(node4);
        node4.parentObstacle = obstacle;
        if (!collidesObstacle(new Node(corner4.add(Vector2D.of(.001,.001))), new Node(corner4.add(Vector2D.of(.001,.001))))){
            workingNodes.add(node4);
        }
        return workingNodes;

    }

    private void compute(){
        if (!allNodes.toList().contains(goal) || !allNodes.toList().contains(start)){
            return;
        }

        for (ModifiedNode node : start.visibleNodes){
            node.setParent(start);
        }

        for (int i = 0; i < 10; i++){
            for (ModifiedNode node : allNodes.toList()){

                double nodeCost = Double.POSITIVE_INFINITY;
                for (ModifiedNode visable : node.visibleNodes){
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
                
            }
        }
        if (path != null){
            drawing.remove(path);
        }

        if (!goal.isDescendedOf(start) || collidesObstacle(start,start)){
            bestCost = Double.POSITIVE_INFINITY;
            return;
        }

        

        path = getPath(goal);
        drawing.add(path);
        bestCost = goal.getCost();
    }

    public PolyShape getPath(Node nodePathEnd){
        Node activeNode = new Node(nodePathEnd);
        PolyShape poly = new PolyShape(0, 0);
        int i = 0;
        while(activeNode.getParent() != null && i <100){
            poly.add(
                new Line(activeNode.x + Math.min(activeNode.getParent().x-activeNode.x,0), activeNode.y + Math.min(activeNode.getParent().y-activeNode.y,0)
                , List.of(Vector2D.of(0,0)
                ,Vector2D.of(activeNode.getParent().x-activeNode.x,activeNode.getParent().y-activeNode.y))
                , Color.GREEN, 3)
            );
            activeNode = activeNode.getParent();
            i++;
        }
        return poly;
    }

    @Override
    public void scheduleObstacles(List<Obstacle> obstacles) {
        setObstacles(obstacles);
    }

    @Override
    public void scheduleStart(Vector2D start) {
        setStart(start);
    }

    @Override
    public void scheduleGoal(Vector2D goal) {
        setGoal(goal);
    }

    /**
     * in this case it is a complete reset, so only use when changing the map completely
     */
    @Override
    public void setObstacles(List<Obstacle> obstacles) {
        wholeObstacles = new ArrayList<>();
        for (Obstacle obstacle : obstacles){
            wholeObstacles.add(new ModifiedObstacle(obstacle));
        }
        drawing.getArray().clear();
        allNodes.clear();
        this.obstacles.clear();
        ArrayList<ModifiedObstacle> workingObstacles = new ArrayList<>();
        for (ModifiedObstacle obstacle : wholeObstacles){
            if (obstacle.isObstacle()){
                workingObstacles.add(obstacle);
            }
        }
        drawing.getArray().addAll(wholeObstacles);
        this.obstacles.addAll(workingObstacles);

        // if (!collidesObstacle(start, start)){
            allNodes.add(start);
        // }

        // if (!collidesObstacle(goal, goal)){
            allNodes.add(goal);
        // }

        for (ModifiedObstacle obstacle : workingObstacles){
            List<ModifiedNode> workableNodes = findCornerNodes(obstacle);
            allNodes.addAll(workableNodes);
        }

        for (ModifiedNode node : allNodes.toList()){
            drawing.add(node.getCircle());
            for (ModifiedNode potentialVis : allNodes.toList()){
                if (!node.equals(potentialVis) && !collidesObstacle(node, potentialVis)){
                    node.addVisibleNode(potentialVis);
                }
            }
        }

        compute();
    }

    @Override
    public void setStart(Vector2D start) {
        this.start.setPosition(start.x, start.y);
        this.start.visibleNodes.clear();
        for (ModifiedNode node : allNodes.toList()){
            if (!node.equals(this.start) && !collidesObstacle(node, this.start)){
                if (!node.visibleNodes.contains(this.start)){
                    node.visibleNodes.add(this.start);
                    this.start.visibleNodes.add(node);
                }else{
                    this.start.visibleNodes.add(node);
                }
            }else{
                if (node.visibleNodes.contains(this.start)){
                    node.visibleNodes.remove(this.start);
                    node.setParent(null);
                }
            }
        }
        compute();
    }

    @Override
    public void addObstacles(List<Obstacle> obstacles) {
        for (Obstacle obstacle : obstacles){
            addObstacle(obstacle);
        }
    }

    private void addObstacle(Obstacle obstacle){

        ModifiedObstacle newObstacle = new ModifiedObstacle(obstacle);

        for (ModifiedNode node : allNodes.toList()){
            List<ModifiedNode> toRemove = new ArrayList<>();

            if (obstacle.didCollide(node, node)){
                if (!node.equals(goal) && !node.equals(start)){
                    allNodes.remove(node);
                    drawing.remove(node.getObj());
                    newObstacle.removedNodes.add(node);
                }
                
                for (ModifiedNode nodeR : node.visibleNodes){
                    nodeR.removeVisibleNode(node);
                }
                continue;
            }
            for (ModifiedNode visible : node.visibleNodes){
                if (obstacle.didCollide(node, visible)){
                    visible.visibleNodes.remove(node);
                    toRemove.add(visible);
                    node.setParent(null);
                    visible.setParent(null);
                }
            }
            node.visibleNodes.removeAll(toRemove);
        }
        wholeObstacles.add(newObstacle);
        obstacles.add(newObstacle);
        drawing.add(obstacle);

        List<ModifiedNode> workingNodes = findCornerNodes(newObstacle);
        allNodes.addAll(workingNodes);
        for (ModifiedNode node : workingNodes){
            drawing.add(node.getCircle());
            node.getCircle().setColor(Color.GREEN);
            for (ModifiedNode potentialVis : allNodes.toList()){
                if (!node.equals(potentialVis) && !collidesObstacle(node, potentialVis)){
                    node.addVisibleNode(potentialVis);
                    if (!workingNodes.contains(potentialVis)){
                        potentialVis.addVisibleNode(node);
                    }
                }
            }
        }

        compute();
    }

    @Override
    public void removeObstacles(List<Obstacle> obstacles) {
        for (Obstacle obstacle : obstacles){
            removeObstacle((ModifiedObstacle)obstacle);
        }
    }

    private void removeObstacle(ModifiedObstacle obstacle){

        // drawing.remove(obstacle);
        // obstacles.remove(obstacle);
        // wholeObstacles.remove(obstacle);
        // allNodes.removeAll(obstacle.childNodes);
        

        // for (ModifiedNode node : obstacle.childNodes){
        //     drawing.remove(node.getObj());
        //     for (ModifiedNode nodeR : node.visibleNodes){
        //         nodeR.removeVisibleNode(node);
        //     }
        // }

        // ArrayList<ModifiedNode> reAdd = new ArrayList<>();
        // for (ModifiedObstacle obstacle2 : obstacles.findKNearest(obstacle, 10)){
        //     for (ModifiedNode node : obstacle2.childNodes){
        //         if (obstacles.search(node.parentObstacle) != null && !collidesObstacle(node, node)){
        //             reAdd.add(node);
        //             allNodes.add(node);
        //         }
        //     }
            
        // }

        // for (ModifiedNode node : reAdd){
        //     drawing.add(node.getCircle());
        //     node.getCircle().setColor(Color.GREEN);
        //     for (ModifiedNode potentialVis : allNodes.toList()){
        //         if (!node.equals(potentialVis) && !collidesObstacle(node, potentialVis)){
        //             node.addVisibleNode(potentialVis);
        //             if (!reAdd.contains(potentialVis)){
        //                 potentialVis.addVisibleNode(node);
        //             }
        //         }
        //     }
        // }

        // compute();
    }

    @Override
    public void setGoal(Vector2D goal) {
        this.goal.setPosition(goal.x, goal.y);
        this.goal.visibleNodes.clear();
        this.goal.setParent(null);
        for (ModifiedNode node : allNodes.toList()){
            if (!node.equals(this.goal) && !collidesObstacle(node, this.goal)){
                if (!node.visibleNodes.contains(this.goal)){
                    node.visibleNodes.add(this.goal);
                    this.goal.visibleNodes.add(node);
                }else{
                    this.goal.visibleNodes.add(node);
                }
            }else{
                if (node.visibleNodes.contains(this.goal)){
                    node.visibleNodes.remove(this.goal);
                }
            }
        }
        compute();
    }

    /**
     * unused
     */
    @Override
    public void setMinimumDistance(double dist) {
        
    }

    /**
     * unused
     */
    @Override
    public void setMaxStepDist(double dist) {
        
    }

    @Override
    public void setDrawingCoords(double x, double y) {
        drawing.setPosition(x, y);
    }

    /**
     * unused
     */
    @Override
    public void setBias(double bias) {
        
    }

    @Override
    public Field getField() {
        return this.field;
    }

    @Override
    public List<Obstacle> getObstacles() {
        ArrayList<Obstacle> list = new ArrayList<>();
        for (Obstacle obstacle : obstacles.toList()){
            list.add(obstacle);
        }
        return list;
    }

    /**
     * dont use until fixed
     */
    @Override
    public List<Node> getNodes() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getNodes'");
    }

    @Override
    public Vector2D getStart() {
        return this.start;
    }

    @Override
    public Vector2D getGoal() {
        return this.goal;
    }

    /**
     * unused
     */
    @Override
    public double getMinimumDist() {
        return 0;
    }

    /**
     * unused
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
     * unused
     */
    @Override
    public double getBias() {
        return 0;
    }

    @SuppressWarnings("unchecked")
    @Override
    public KDTree<Obstacle> getKDTreeObstacles(){
        KDTree<? extends Obstacle> list = (KDTree<? extends Obstacle>)obstacles;
        return (KDTree<Obstacle>) list;
    }
    
    @Override
    public void delete() {
        this.drawing.getArray().clear();
        allNodes.clear();
        obstacles.clear();
        wholeObstacles.clear();
        bestCost = Double.POSITIVE_INFINITY;
        start = new ModifiedNode(850,450);
        goal = new ModifiedNode(850,450);
        path = null;
        vis.remove(drawing);
    }

    @Override
    public void prune(int max) {
        
    }
}
