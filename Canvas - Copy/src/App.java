import Canvas.Commands.Commands;
import Canvas.Commands.TimedCommand;
import Canvas.FileUtil.FileParser;
import Canvas.Inputs.KeyInput;
import Canvas.Inputs.MouseInput;
import Canvas.Inputs.MouseInput.MouseInputs;
import Canvas.Inputs.MouseInput.MouseSide;
import Canvas.Pathing.RRT.BetterPreloadRRT;
import Canvas.Pathing.RRT.InformedRRTStar;
import Canvas.Pathing.RRT.MomentumRRT;
import Canvas.Pathing.RRT.Obstacle;
import Canvas.Pathing.RRT.RRT;
import Canvas.Pathing.RRT.RRTBase;
import Canvas.Pathing.RRT.RRTContainer;
import Canvas.Pathing.RRT.RRTHelperBase.Field;
import Canvas.Pathing.RRT.RRTStar;
import Canvas.Shapes.Text;
import Canvas.Shapes.VisualJ;
import Canvas.Util.Profile;
import Canvas.Util.Random;
import Canvas.Util.Vector2D;
import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

class MouseMovingHelper{
    public Vector2D mouseMarkCoords = Vector2D.of(0,0);
    public Vector2D startMarkCoords = Vector2D.of(0,0);
    public Vector2D startFrameMove = Vector2D.of(0,0);
    public boolean hasReleased = true;
}

public class App {
    public static VisualJ vis = null;
    public static MouseInput mouse = null;
    public static KeyInput keyboard = null;

    public static Profile profile = new Profile();
    public static Text graphicsFR = new Text(100, 120, 15, Color.RED, "");
    public static Text physicsFR = new Text(100, 140, 15, Color.RED, "");
    public static Text addingText = new Text(200, 200, 100, Color.RED, "");

    public static RRTContainer rrtContainer = new RRTContainer();

    public static int state = 0;

    
    public static void main(String[] args) {
        
        vis = new VisualJ("Simulation",1700,900,Color.black);

        mouse = new MouseInput(vis);
        keyboard = new KeyInput(vis);
        
        vis.startThread();
        
        FileParser fileParser = new FileParser("Canvas - Copy/src/Canvas/FileUtil/");
        List<Obstacle> obstacles = fileParser.loadSquares(new RRT(vis, new Field(0, 0, 1700, 900)), "navgrid");

        RRTBase rrt = new RRT(vis, new Field(0, 0, 1700, 900));
        rrtContainer.contained = rrt;
        // for (int i = 0; i < 300; i++){
        //     double x = Random.randDouble(0, 1700);
        //     double y = Random.randDouble(0, 900);

        //     double width = Random.randDouble(0, 40);
        //     double height = Random.randDouble(0, 40);
        //     obstacles.add(new Obstacle(x, y, width, height, true));
        // }

        rrtContainer.contained.scheduleObstacles(obstacles);
        
        // rrt.setBias(.3);
        // rrt.setMinimumDistance(200);
        // rrt.setMaxStepDist(10);
        // rrt.scheduleStart(Vector2D.of(850,450));
        // rrt.setDrawingCoords(0,0);
        // rrt.scheduleGoal(Vector2D.of(1000,300));


        TimedCommand timedCommand1 = new TimedCommand(()->{
            try {
                rrtContainer.contained.process();
            } catch (Exception e) {
                
            }
            
        }, 0);
        timedCommand1.schedule();

        keyboard.keyPressed("i").onTrue(Commands.runOnce(()->{
            rrtContainer.contained.scheduleObstacles(fileParser.loadSquares(rrtContainer.contained, "AvoidStage"));
        }));

        keyboard.keyPressed("k").onTrue(Commands.runOnce(()->{
            rrtContainer.contained.scheduleObstacles(fileParser.loadSquares(rrtContainer.contained, "navgrid"));
        }));

        keyboard.keyPressed("l").onTrue(Commands.runOnce(()->{
            rrtContainer.contained.scheduleObstacles(fileParser.loadSquares(rrtContainer.contained, "note3Correct"));
        }));

        keyboard.keyPressed("j").onTrue(Commands.runOnce(()->{
            rrtContainer.contained.scheduleObstacles(fileParser.loadSquares(rrtContainer.contained, "SmallGridSizes"));
        }));

        keyboard.keyPressed("a").onTrue(Commands.runOnce(()->{
            Vector2D point = vis.screenRelativePoint(mouse.getMouseCoords());
            rrtContainer.contained.addObstacles(List.of(new Obstacle(point.x, point.y, 30 ,30, true)));
        }));

        keyboard.keyPressed("s").onTrue(Commands.runOnce(()->{
            if (rrtContainer.contained instanceof BetterPreloadRRT){
                ((BetterPreloadRRT)rrtContainer.contained).removeLastDynamic();
            }
            // rrtContainer.contained.removeObstacles(List.of(rrtContainer.contained.getObstacles().get(0)));
        }));

        keyboard.keyPressed("Space").onTrue(Commands.runOnce(()->{
            state++;
            acknowledgeState();
        }));
        keyboard.keyPressed("r").onTrue(Commands.runOnce(()->{
            state--;
            acknowledgeState();
        }));

        

        // Commands.timed(()->{
        //     vis.setFrame(mouse.getMouseCoords().x-vis.WIDTH/2, mouse.getMouseCoords().y-vis.HEIGHT/2);
        //     vis.setZoom(mouse.getMouseCoords().y/1000);
        // },10).schedule();

        // TypeButton typeButton = new TypeButton(100, 100, 100, 100, Color.GRAY, true, vis);
        // typeButton.assignTextDetails("Arial", Font.PLAIN, 30, Color.GREEN, Vector2D.of(10,10));
        // vis.add(typeButton);
        
        // mouse.addEvent(()->{
        //     typeButton.isClicked(mouse.getMouseCoords(),vis);
        // },MouseInputs.MOUSE_PRESSED, MouseSide.LEFT);
        
        

        mouse.addEvent(()->{
            if (mouse.getMouseWheelPosition() < 0){
                vis.setZoom(vis.getZoom()/1.05);
                return;
            }
            vis.setZoom(vis.getZoom()*1.05);
        }, MouseInputs.MOUSE_WHEEL_MOVED, MouseSide.LEFT);

        
        

        MouseMovingHelper mouseHelperLeft = new MouseMovingHelper();
        mouse.addEvent(()->{
            mouseHelperLeft.hasReleased = true;
        },MouseInputs.MOUSE_RELEASED, MouseSide.LEFT);

        mouse.addEvent(()->{
            if (mouseHelperLeft.hasReleased){
                mouseHelperLeft.mouseMarkCoords = mouse.getMouseCoords();
                mouseHelperLeft.startMarkCoords = Vector2D.of(rrtContainer.contained.getGoal());
                mouseHelperLeft.hasReleased = false;
                mouseHelperLeft.startFrameMove = vis.getFrameMove();
            }
            rrtContainer.contained.scheduleGoal(Vector2D.of(mouseHelperLeft.startMarkCoords.x-(mouseHelperLeft.mouseMarkCoords.x-mouse.getMouseCoords().x + (vis.getFrameMove().x - mouseHelperLeft.startFrameMove.x))*1/vis.getZoom()
                ,mouseHelperLeft.startMarkCoords.y-(mouseHelperLeft.mouseMarkCoords.y-mouse.getMouseCoords().y)*1/vis.getZoom()));
            // vis.setFrame(startMarkCoords.x-(mouseMarkCoords.x-mouse.getMouseCoords().x)*1/vis.getZoom(),startMarkCoords.y-(mouseMarkCoords.y-mouse.getMouseCoords().y)*1/vis.getZoom());
        },MouseInputs.MOUSE_DRAGGED, MouseSide.LEFT);

        MouseMovingHelper mouseHelperRight = new MouseMovingHelper();
        mouse.addEvent(()->{
            mouseHelperRight.hasReleased = true;
        },MouseInputs.MOUSE_RELEASED, MouseSide.RIGHT);

        mouse.addEvent(()->{
            if (mouseHelperRight.hasReleased){
                mouseHelperRight.mouseMarkCoords = mouse.getMouseCoords();
                mouseHelperRight.startMarkCoords = Vector2D.of(rrtContainer.contained.getStart());
                mouseHelperRight.hasReleased = false;
            }
            rrtContainer.contained.scheduleStart(Vector2D.of(mouseHelperRight.startMarkCoords.x-(mouseHelperRight.mouseMarkCoords.x-mouse.getMouseCoords().x)*1/vis.getZoom(),mouseHelperRight.startMarkCoords.y-(mouseHelperRight.mouseMarkCoords.y-mouse.getMouseCoords().y)*1/vis.getZoom()));
            // vis.setFrame(startMarkCoords.x-(mouseMarkCoords.x-mouse.getMouseCoords().x)*1/vis.getZoom(),startMarkCoords.y-(mouseMarkCoords.y-mouse.getMouseCoords().y)*1/vis.getZoom());
            // vis.setZoom(mouse.getMouseCoords().y/1000);
        },MouseInputs.MOUSE_DRAGGED, MouseSide.RIGHT);
        
        // mouse.addEvent(()->{
        //     if (hasReleased){
        //         mouseMarkCoords = mouse.getMouseCoords();
        //         startMarkCoords = vis.getFrameMove();
        //         hasReleased = false;
        //     }
        //     vis.setFrame(startMarkCoords.x-(mouseMarkCoords.x-mouse.getMouseCoords().x)*1/vis.getZoom(),startMarkCoords.y-(mouseMarkCoords.y-mouse.getMouseCoords().y)*1/vis.getZoom());
        //     // vis.setZoom(mouse.getMouseCoords().y/1000);
        // },MouseInputs.MOUSE_DRAGGED, MouseSide.LEFT);

        defineShapes(vis);
        // mouse.addEvent(()->keyboard.beginGatherAll(),MouseInputs.MOUSE_CLICKED,MouseSide.LEFT);
        // mouse.addEvent(()->keyboard.endGatherAll(),MouseInputs.MOUSE_CLICKED,MouseSide.RIGHT);
    }

    public static void acknowledgeState(){
        List<Obstacle> obstacles1 = rrtContainer.contained.getObstacles();
        Vector2D goal = rrtContainer.contained.getGoal();
        Vector2D start = rrtContainer.contained.getStart();
        rrtContainer.contained.delete();
        switch (state) {
            case 0:
                rrtContainer.contained = new RRT(vis, new Field(0, 0, 1700, 900));
                break;
            case 1:
                rrtContainer.contained = new RRTStar(vis, new Field(0, 0, 1700, 900));
                break;
            case 2:
                rrtContainer.contained = new InformedRRTStar(vis, new Field(0, 0, 1700, 900));
                break;
            case 3:
                rrtContainer.contained = new BetterPreloadRRT(vis, new Field(0, 0, 1700, 900), new ArrayList<>());
                break;
            case 4:
                rrtContainer.contained = new MomentumRRT(vis, new Field(0, 0, 1700, 900));
                break;
            case 5:
                state = 0;
                rrtContainer.contained = new RRT(vis, new Field(0, 0, 1700, 900));
                break;
            default:
                rrtContainer.contained = new RRT(vis, new Field(0, 0, 1700, 900));
                break;
        }
        
        rrtContainer.contained.scheduleObstacles(obstacles1);
        rrtContainer.contained.scheduleStart(start);
        rrtContainer.contained.scheduleGoal(goal);
    }

    public static void defineShapes(VisualJ vis){
        
        
        // for (int i=0;i<1;i++){
        //     int x = 0;//Random.randInt(0, vis.WIDTH);
        //     int y = 0;//Random.randInt(0, vis.HEIGHT);

        //     // List<Vector2D> list = List.of(Vector2D.of(-100,-100),Vector2D.of(100,100));
        //     // Line line = new Line(100, -0, list, Color.blue, 10);
            
        //     Circle circ1 = new Circle(100, 100, 5, Color.RED, true);
        //     Circle circ2 = new Circle(-100, -100, 5, Color.RED, true);
        //     Polygon polygon = new Polygon(0, 0, new double[]{0,0,50},new double[]{0,50,0}, Color.RED, false);

        //     PolyShape poly = new PolyShape(x, y, List.of(circ1,circ2,polygon));
        //     vis.add(poly);
        //     vis.add(new Circle(x, y, 500, Color.RED, false));
            
        //     Circle circ = new Circle(0, 0, 20, Color.BLUE, false);
        //     vis.add(circ);
        // }
        // vis.add(addingText);
    }
}