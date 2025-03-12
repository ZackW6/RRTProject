package Canvas.Shapes.UserButtons;

import java.awt.Color;
import java.util.List;

import Canvas.Shapes.Rectangle;
import Canvas.Util.Vector2D;

public class ClickButton extends Rectangle implements ClickableButton{

    private Runnable run = ()->{};

    public ClickButton(double X, double Y, double Width, double Height, Color Color, boolean tf) {
        super(X, Y, Width, Height, Color, tf);
    }

    public void setRunWhenClicked(Runnable run){
        this.run = run;
    }

    @Override
    public void runOnClick() {
        run.run();
    }

    @Override
    public Vector2D getCoords() {
        return this.coords;
    }

    @Override
    public Vector2D getDimensions() {
        return Vector2D.of(this.width, this.height);
    }
    
}
