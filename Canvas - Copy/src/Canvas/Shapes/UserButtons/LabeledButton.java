package Canvas.Shapes.UserButtons;

import java.awt.Color;

import Canvas.Shapes.PolyShape;
import Canvas.Shapes.Rectangle;
import Canvas.Shapes.Text;
import Canvas.Util.Vector2D;

public class LabeledButton extends PolyShape implements ClickableButton{

    private Runnable run = ()->{};
    
    public LabeledButton(double X, double Y, double Width, double Height, String text, Color buttonColor, Color textColor, boolean tf) {
        super(X, Y);
        this.add(new Rectangle(0, 0, Width, Height, buttonColor, tf));
        Text label = new Text((int)X+10, (int)Y+(int)Height/2+(int)Height/5, (int)Height/2, textColor, text);
        this.add(label);
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
        return Vector2D.of(this.get(0).getWidth(), this.get(0).getHeight());
    }
    
}
