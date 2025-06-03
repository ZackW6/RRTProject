package Canvas.Pathing.RRT;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;
import java.util.function.Supplier;

import Canvas.Commands.Command;
import Canvas.Commands.CommandBase;
import Canvas.Shapes.Circle;
import Canvas.Shapes.Obj;
import Canvas.Shapes.PolyShape;
import Canvas.Shapes.VisualJ;
import Canvas.Util.Vector2D;

public class MomentumRRT extends RRTStar{

    private PolyShape momentumDrawing;

    public static class MomentumNode extends Node{
        ArrayList<Node> children = new ArrayList<>();
        //Meters per second
        protected double localVel;
        //Rotations per second
        protected double localTurnVel;

        protected double localRot;

        public MomentumNode(double x, double y){
            super(x,y);
        }

        public void setLocalVelocity(double speed){
            localVel = speed;
        }

        public double getLocalVelocity(){
            return localVel;
        }

        public void setLocalTurnVelocity(double rot){
            localTurnVel = rot;
        }

        public double getLocalTurnVelocity(){
            return localTurnVel;
        }

        public void setLocalRotation(double rot){
            localRot = rot;
        }

        public double getLocalRotation(){
            return localRot;
        }
    }
    CommandBase drawThread;

    public MomentumRRT(VisualJ vis, Field field) {
        super(vis, field);
        momentumDrawing = new PolyShape(0, 0);
        //Threaded because the action of calculating the curves and drawing slows main thread of pathing significantly
        drawThread = new Command(()->{
            drawAction();
        }).finallyDo(()->{
            vis.remove(momentumDrawing);
        });
        drawThread.schedule();
    }

    /**
     * Using given points of the InformedRRTStar, curve the path using bezier curves, and plot using circles
     */
    protected void drawAction(){

        PolyShape other = new PolyShape(0, 0);
        List<Function<Double, Vector2D>> bezierPath = new ArrayList<>();

        Node past = null;
        Node current = goal;

        while (current.getParent() != null) {
            Node next = current.getParent();
            Vector2D n1 = Vector2D.of(current);
            Vector2D n0 = Vector2D.of(next);
        
            Vector2D dir = n1.subtract(n0).normalize();
            Vector2D normal = Vector2D.of(-dir.y, dir.x);
        
            Vector2D prevVec = (past != null) ? Vector2D.of(past) : n1.add(n1.subtract(n0));
            Vector2D prevDir = n0.subtract(prevVec).normalize();
        
            double cross = prevDir.x * dir.y - prevDir.y * dir.x;
            if (cross > 0) {
                normal = normal.multiply(-1);
            }
        
            double segLen = Math.pow(n0.distanceTo(n1),1.2)/5;//Math.pow(n0.distanceTo(n1),2)/200;
            double tension = segLen * 0.4;
            double angle = Math.acos(Math.max(-1, Math.min(1, prevDir.dot(dir))));
            double angleFactor = Math.sin(angle) * 0.5 + 0.5;
        
            double curveAmount = tension * 0.5 * angleFactor;
        
            Vector2D c1 = n0.add(dir.multiply(tension)).add(normal.multiply(curveAmount));
            Vector2D c2 = n1.subtract(dir.multiply(tension)).add(normal.multiply(curveAmount));
        
            Function<Double, Vector2D> bezierCurve = (t) -> {
                double u = 1 - t;
                return n0.multiply(u * u * u)
                    .add(c1.multiply(3 * u * u * t))
                    .add(c2.multiply(3 * u * t * t))
                    .add(n1.multiply(t * t * t));
            };
        
            bezierPath.add(bezierCurve);
            past = current;
            current = next;
        }
        
        for (Function<Double, Vector2D> piece : bezierPath){
            for (int i = 0; i < 100; i++){
                Vector2D place  = piece.apply(i/100.0);
                Circle c = new Circle(place.x, place.y, 5, Color.RED, false);
                
                other.add(c);
            }
        }
        
        if (vis.shapes.indexOf(momentumDrawing) == -1){
            vis.add(momentumDrawing);
        }
        momentumDrawing.shapes = (ArrayList<Obj>) other.getArray();
    }

    @Override
    public void delete() {
        super.delete();
        drawThread.cancel();
    }
}
