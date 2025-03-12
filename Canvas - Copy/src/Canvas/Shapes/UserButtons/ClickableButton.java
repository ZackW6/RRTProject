package Canvas.Shapes.UserButtons;

import java.awt.Color;

import Canvas.Shapes.Rectangle;
import Canvas.Shapes.VisualJ;
import Canvas.Util.Vector2D;

public interface ClickableButton{

    public abstract void runOnClick();

    public abstract Vector2D getCoords();

    public abstract Vector2D getDimensions();

    public default boolean isClicked(Vector2D point, VisualJ vis){
        double zoom = vis.getZoom();

        double xp = (-vis.WIDTH/2 + vis.getFrameMove().x + getCoords().x) * zoom + vis.WIDTH/2;
        double yp = (-vis.HEIGHT/2 + vis.getFrameMove().y + getCoords().y) * zoom + vis.HEIGHT/2;

        Vector2D newPoint = point;
        Vector2D newPose = Vector2D.of(xp,yp);
        Vector2D newDimensions = this.getDimensions().multiply(zoom);
        if (newPoint.x > newPose.x && newPoint.x < newPose.x + newDimensions.x && newPoint.y > newPose.y && newPoint.y < newPose.y + newDimensions.y){
            runOnClick();
            return true;
        }
        return false;
    }
    
}
