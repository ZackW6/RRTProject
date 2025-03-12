package Canvas.Pathing.RRT;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import Canvas.Shapes.Circle;
import Canvas.Shapes.Line;
import Canvas.Shapes.Obj;
import Canvas.Shapes.PolyShape;
import Canvas.Shapes.Rectangle;
import Canvas.Shapes.Square;
import Canvas.Shapes.VisualJ;
import Canvas.Util.KDTree;
import Canvas.Util.Vector2D;

public abstract class RRTHelperBase implements RRTBase{
    protected KDTree<Node> nodes = new KDTree<>();
    // protected ArrayList<Node> nodes = new ArrayList<>();

    protected KDTree<Obstacle> obstacles = new KDTree<Obstacle>();

    protected Node start = new Node(0, 0);

    protected Node goal = new Node(1000,600);

    private Field field = new Field(0, 0, 1000, 600);

    private double minimumDist = 100;

    private double maxStepSize = 10;

    protected PolyShape drawing;

    protected VisualJ vis;

    private double bias = 0;
    
    private Runnable[] initActions = new Runnable[3];
    /**
     * 
     * @param vis screen to act on
     */
    public RRTHelperBase(VisualJ vis, Field field){
        for (int i = 0; i < initActions.length; i++){
            initActions[i] = ()->{};
        }

        this.field = field;
        ArrayList<Obj> temp = new ArrayList<>();
        this.start.getCircle().setColor(Color.ORANGE);
        temp.add(start.getCircle());
        goal.setColor(Color.MAGENTA);
        temp.add(goal.getCircle());
        drawing = new PolyShape(0, 0, temp);
        vis.add(drawing);
        this.vis = vis;
        nodes.add(start);
    }

    @Override
    public void process(){
        for (int i = 0; i < initActions.length; i++){
            initActions[i].run();
            initActions[i] = ()->{};
        }
        
        runAction();
    }

    protected abstract void runAction();

    @Override
    public PolyShape getPath(Node nodePathEnd){
        Node activeNode = new Node(nodePathEnd);
        PolyShape poly = new PolyShape(0, 0);
        int i = 0;
        while(activeNode.getParent() != null && i <1000){
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

    protected Node getRandomPoint(double bias) {
        double chance = Math.random();
        if (chance > bias){
            double x = ((Math.random()*(field.width-field.x))+field.x);//Random.randDouble(0.0, field.width());
            double y = ((Math.random()*(field.height-field.y))+field.y);//Random.randDouble(0.0, field.height());
            return new Node(x, y);
        }

        return new Node(goal.x,goal.y);
    }

    protected Node getNearestPoint(Node newPoint){
        return nodes.findKNearest(newPoint, 1).get(0);
    }

    protected Node getNewPoint( Node random, Node nearest) {
        double theta = Math.atan2(random.y - nearest.y, random.x - nearest.x);
        double newX = (nearest.x + maxStepSize * Math.cos(theta));
        double newY = (nearest.y + maxStepSize * Math.sin(theta));

        return new Node(newX, newY, nearest);
    }

    protected List<Node> getNearbyNodes(Node newPoint) {
        return getNearbyNodes(newPoint, calculateRadius());
    }

    protected List<Node> getNearbyNodes(Node newPoint, double radius) {
        return nodes.findInRange(new Node(newPoint.x - radius, newPoint.y - radius), new Node(newPoint.x + radius, newPoint.y + radius));
    }

    protected List<Node> getNearbyNodes(Node newPoint, List<Node> toLookThrough) {
        List<Node> nearbyNodes = new ArrayList<>();
        double radius = calculateRadius();
        for (Node node : toLookThrough) {
            if (node.distanceTo(newPoint) < radius) {
                nearbyNodes.add(node);
            }
        }
        return nearbyNodes;
    }

    private double calculateRadius() {
        return 100;
    }

    public void scheduleObstacles(List<Obstacle> obstacles){
        initActions[0] = ()->{setObstacles(obstacles);};
    }

    public void scheduleStart(Vector2D start){
        initActions[1] = ()->{setStart(start);};
    }

    public void scheduleGoal(Vector2D goal){
        initActions[2] = ()->{setGoal(goal);};
    }

    /**
     * forces a full reset of all paths, use only for full field changes
     * @param obstacles
     */
    public  void setObstacles(List<Obstacle> obstacles){
        nodes.clear(drawing);

        nodes.add(this.start);
        drawing.add(this.start.getCircle());

        this.obstacles.clear(drawing);
        this.obstacles.addAll(obstacles);
        drawing.getArray().addAll(obstacles);
        
    }

    public  void addObstacles(List<Obstacle> obstacles){

    }

    public  void removeObstacles(List<Obstacle> obstacles){
        
    }

    public  void setStart(Vector2D start){

        nodes.clear(drawing);

        this.start.setPosition(start.x, start.y);

        nodes.add(this.start);
        nodes.add(this.goal);
        this.start.getCircle().setColor(Color.ORANGE);
        drawing.add(this.start.getCircle());
        drawing.add(this.goal.getCircle());
    }

    public  void setGoal(Vector2D goal){
        
        if (drawing.indexOf(this.goal.getCircle())!=-1){
            drawing.remove(this.goal.getCircle());
        }
        this.goal = new Node(goal.x,goal.y);
        this.goal.setColor(Color.MAGENTA);
        drawing.add(this.goal.getCircle());
    }

    public void setMinimumDistance(double dist){
        this.minimumDist = dist;
    }

    public void setMaxStepDist(double dist){
        this.maxStepSize = dist;
    }

    public void setDrawingCoords(double x, double y){
        this.drawing.setPosition(x, y);
    }

    public void setBias(double bias){
        this.bias = bias;
    }

    public Field getField(){
        return field;
    }

    public List<Obstacle> getObstacles(){
        return obstacles.toList();
    }

    public List<Node> getNodes(){
        return List.copyOf(nodes.toList());
    }

    public Vector2D getStart(){
        return start;
    }

    public Vector2D getGoal(){
        return goal;
    }

    public double getMinimumDist(){
        return minimumDist;
    }

    public double getMaxStepSize(){
        return maxStepSize;
    }

    public Vector2D getDrawingCoords(){
        return drawing.getCoords();
    }

    public double getBias(){
        return bias;
    }

    @Override
    public KDTree<Obstacle> getKDTreeObstacles(){
        return obstacles;
    }
    
    public record Field(double x, double y, double width, double height){};

    @Override
    public void delete() {
        drawing.getArray().clear();
        obstacles.clear();
        nodes.clear();
        start = new Node(0, 0);
        goal = new Node(1000,600);
        field = new Field(0, 0, 1000, 600);
        minimumDist = 10;
        maxStepSize = 10;
        bias = 0;
        vis.remove(drawing);
    }
}
