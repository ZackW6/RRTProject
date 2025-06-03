package Canvas.Pathing.RRT;
import java.awt.Color;

import Canvas.Shapes.Rectangle;
import Canvas.Util.Vector2D;
import Canvas.Util.Maps.Point;

public class PathFollower extends Rectangle implements Point{

    //degrees
    private double targetRotation = 0;
    //degrees
    private double yaw = 0;
    //degrees per second
    private double maxRotationSpeed = 30;

    //meters
    private Vector2D position = new Vector2D(200,200);
    //meters per second
    private double targetVelocity = 0;
    //meters per second
    private double velocity = 0;
    //meters per second
    private double maxVelocity = 50;
    //meters per second per second
    private double maxAcceleration = 1;
    
    //milliseconds
    private long lastTime = 0;

    //Front marker
    private final Rectangle front;

    private boolean disabled = false;

    public PathFollower(double X, double Y, double Width, double Height, Color Color, boolean tf) {
        super(X, Y, Width, Height, Color, tf);
        front = new Rectangle(X + Width/2, Y, Width/10, Height/10, Color, tf);
        applyPosition(Vector2D.of(X,Y), 0);
        lastTime = System.currentTimeMillis();
    }

    public void disable(boolean disable){
        this.disabled = disable;
        if (disabled){
            this.coords = new Vector2D(-1000,-1000);
        }else{
            applyPosition(position, yaw);
        }
    }

    public void setPosition(Vector2D vec, double rotation){
        if (disabled){
            return;
        }
        position = vec;
        yaw = rotation;
        velocity = 0;
        applyPosition(vec, rotation);
    }

    /**
     * meters per second
     * @param max
     */
    public void setMaxVelocity(double max){
        maxVelocity = max;
    }
    /**
     * meters per second per second
     * @param max
     */
    public void setMaxAcceleration(double max){
        maxAcceleration = max;
    }

    /**
     * degrees per second
     * @param max
     */
    public void setMaxRotationSpeed(double max){
        maxRotationSpeed = max;
    }

    /**
     * Should be run in a main loop, respect time passed and will attempt to follow momentum that may be seen in real life.
     * This is the simulated version of an input for the tractor or other such follower, in this case only forward motion.
     * @param goalNode
     */
    public void acceptVector(Node goalNode){
        if (disabled){
            return;
        }
        if (goalNode == null || goalNode.getParent() == null){
            return;
        }
        long currentTime = System.currentTimeMillis();
        double timeChange = (currentTime - lastTime)/1000.0;
        lastTime = currentTime;

        double rotation = goalNode.calculateAngleBetweenPoints(goalNode.parent);
        if (goalNode.distanceTo(goalNode.parent) < 2){
            rotation = yaw;
        }
        targetRotation = rotation;
        yaw = correctRotation(yaw);
        double wantedRotChange = wantedChange(yaw, rotation);

        yaw += clamp(wantedRotChange, -maxRotationSpeed * timeChange, maxRotationSpeed * timeChange);

        targetVelocity = Math.abs(Math.pow(Math.cos(Math.toRadians(wantedRotChange)),10))*maxVelocity * timeChange;
        velocity += Math.min(targetVelocity - velocity, maxAcceleration * timeChange);
        if (Math.abs(wantedRotChange) > 90){
            velocity = 0;
        }
        position = position.add(Vector2D.of(velocity * Math.cos(Math.toRadians(yaw)),velocity * Math.sin(Math.toRadians(yaw))));

        applyPosition(position, yaw);
    }

    private double clamp(double n, double min, double max){
        return Math.max(Math.min(n,max),min);
    }

    private double correctRotation(double rot){
        rot = rot % 360;
        rot = rot > 180 ? rot -360 : (rot < -180 ? rot +360 : rot);
        return rot;
    }

    private double wantedChange(double init, double wanted){
        init = correctRotation(init);
        wanted = correctRotation(wanted);
        double change = wanted - init;
        if (Math.abs(change) > 180){
            if (change > 0){
                change -= 360;
            }else{
                change += 360;
            }
        }
        return change;
    }

    /**
     * safely apply position and rotation to the drawings as well
     * @param pos
     * @param yaw
     */
    private void applyPosition(Vector2D pos, double yaw){
        if (disabled){
            return;
        }
        front.setPosition(pos.x,pos.y - height/2);
        front.rotate(yaw);
        setPosition(pos.x - width/2,pos.y - height/2);
        rotate(yaw);
    }

    @Override
    public double getX() {
        return position.x;
    }

    @Override
    public double getY() {
        return position.y;
    }

    public double getRotation(){
        return yaw;
    }

    public double getVelocity(){
        return velocity;
    }

    public boolean isDisabled(){
        return disabled;
    }
}
