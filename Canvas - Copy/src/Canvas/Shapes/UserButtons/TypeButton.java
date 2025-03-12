package Canvas.Shapes.UserButtons;

import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics2D;
import java.util.List;

import Canvas.Commands.Command;
import Canvas.Commands.CommandBase;
import Canvas.Commands.Commands;
import Canvas.Inputs.KeyInput;
import Canvas.Shapes.Obj;
import Canvas.Shapes.PolyShape;
import Canvas.Shapes.Rectangle;
import Canvas.Shapes.Text;
import Canvas.Shapes.VisualJ;
import Canvas.Util.Vector2D;

public class TypeButton extends PolyShape implements ClickableButton{

    private List<String> str = List.of();
    private final KeyInput keyInput;
    private final CommandBase addStringCommand;

    private double textSize = 10;
    private String textType = "Arial";
    private int textStyle = Font.BOLD;

    private Vector2D textStartPose = Vector2D.of(0,0);
    private Color textColor = Color.BLUE;

    private int lastListLength = 0;
    public TypeButton(double X, double Y, double Width, double Height, Color Color, boolean tf, VisualJ receiver) {
        super(X, Y, List.of(new Rectangle(0, 0, Width, Height, Color, tf)));
        this.keyInput = new KeyInput(receiver);

        this.addStringCommand = Commands.timed(()->{
            str = List.copyOf(keyInput.getCurrentGather());
            if (str.size() > lastListLength){
                lastListLength = str.size();
                while(this.getArray().size()-1 < str.size()){
                    add(new Text(0, 0, textSize, textColor, "").withFont(textType,textStyle,textSize));
                }
            }else if(str.size() < lastListLength){
                lastListLength = str.size();
                while(this.getArray().size()-1 > str.size()){
                    remove();
                }
            }
            try {
                for (int i = 1; i < this.getArray().size() || i < str.size(); i++){
                    Text txt = (Text)this.getArray().get(i);
                    txt.setPosition(textStartPose.x, textStartPose.y + (i-1)*txt.getHeight()*1);
                    txt.setText(str.get(i-1));
                }
            } catch (Exception e) {
                //TODO I shouldn't need a try-catch, I should fix this, honestly the whole thing is rather stupid
            }
            
            
        },10);
    }

    /**
     * set the font, size and style of the text graphic
     * @param fontType
     * @param styleType use already defined Font.BOLD or others
     * @param fontSize
     */
    public void assignTextDetails(String fontType, int styleType, double fontSize, Color color, Vector2D textStartPose){
        this.textSize = fontSize;
        this.textType = fontType;
        this.textStyle = styleType;
        this.textColor = color;
        this.textStartPose = textStartPose;
    }

    public List<String> getString(){
        return List.copyOf(str);
    }

    @Override
    public void runOnClick(){
        if (str.size()>0){
            keyInput.setCurrentGather(getString());
        }else{
            keyInput.setCurrentGather(List.of(""));
        }
        keyInput.beginGatherAll();
        
        addStringCommand.schedule();
    }

    public void runOnNoClick(){
        addStringCommand.cancel();
        keyInput.endGatherAll();
    }

    @Override
    public boolean isClicked(Vector2D point, VisualJ vis){
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
        runOnNoClick();
        return false;
    }

    @Override
    public Vector2D getCoords() {
        try {
            return this.coords;
        } catch (Exception e) {
            return Vector2D.of(-10, -10);
        }
    }

    @Override
    public Vector2D getDimensions() {
        try {
            return Vector2D.of(this.getArray().get(0).getWidth(), this.getArray().get(0).getHeight());
        } catch (Exception e) {
            return Vector2D.of(-10, -10);
        }
    }
}
