package Canvas.Shapes;
import java.awt.*;
import java.awt.font.FontRenderContext;
import java.awt.geom.AffineTransform;

import Canvas.Util.Vector2D;

public class Text extends Obj{

    private String str="";
    private Font font = new Font("Arial", Font.BOLD, (int)15);

    public Text(double X,double Y,double Height, Color Color,String text){

        super(X,Y,0,0,Color,false);

        this.str = text;

        withFont(fixFontHeight(Height,new Font("Arial", Font.BOLD, (int)Height)));
        recheck();
    }

    /**
     * set the font, size and style of the text graphic
     * @param fontType
     * @param styleType use already defined Font.BOLD or others
     * @param fontSize
     */
    public Text withFont(String fontType, int styleType, double fontSize){
        this.font = fixFontHeight(fontSize, new Font(fontType, styleType, (int)fontSize));

        recheck();
        return this;
    }

    /**
     * 
     * @param font
     */
    private Text withFont(Font font){
        this.font = font;

        recheck();
        return this;
    }

    public String getText(){
        return str;
    }

    public void setText(String newText){
        this.str=newText;
        recheck();
    }

    private void recheck(){
        Canvas c = new Canvas();
        FontMetrics fontMetrics = c.getFontMetrics(font);
        this.width = fontMetrics.stringWidth(this.str);
        this.height = fontMetrics.getHeight();
    }
    
    /**
     * set the font, size and style of the text graphic
     * @param fontType
     * @param styleType use already defined Font.BOLD or others
     * @param fontSize
     */
    public static Vector2D predictSize(String str, Font font){
        Canvas c = new Canvas();
        FontMetrics fontMetrics = c.getFontMetrics(font);
        double width = fontMetrics.stringWidth(str);
        double length = fontMetrics.getHeight();
        return new Vector2D(width, length);

    }

    /**
     * set the font, size and style of the text graphic
     * @param fontType
     * @param styleType use already defined Font.BOLD or others
     * @param fontSize
     */
    private static double predictHeight(Font font){
        Canvas c = new Canvas();
        FontMetrics fontMetrics = c.getFontMetrics(font);
        double length = fontMetrics.getHeight();
        return length;

    }

    public Font getFont(){
        return font;
    }

    public static Font fixFontHeight(double desiredHeight, Font font){
        Font fixedFont = font;
        if ((int)desiredHeight > predictHeight(fixedFont)){
            while((int)desiredHeight > predictHeight(fixedFont)){
                fixedFont = new Font(fixedFont.getFontName(), fixedFont.getStyle(),fixedFont.getSize()+1);
            }
            if ((int)desiredHeight == predictHeight(fixedFont)){
                return fixedFont;
            }
            return new Font(fixedFont.getFontName(), fixedFont.getStyle(),fixedFont.getSize()-1);
        }else{
            while((int)desiredHeight < predictHeight(fixedFont)){
                fixedFont = new Font(fixedFont.getFontName(), fixedFont.getStyle(),fixedFont.getSize()-1);
            }
            if ((int)desiredHeight == predictHeight(fixedFont)){
                return fixedFont;
            }
            return new Font(fixedFont.getFontName(), fixedFont.getStyle(),fixedFont.getSize()+1);
        }
    }

    /**
     * The Text application of the show method, draws a Text String based on instance data
     */
    @Override
    public void show(Graphics2D g2dBuffer, double zoomRatio){
        double xtra = -width/2;
        double ytra = height/4;

        g2dBuffer.setFont(fixFontHeight(height*zoomRatio, this.getFont()));
        g2dBuffer.drawString(this.str,(int)(xtra*zoomRatio),(int)(ytra*zoomRatio));
    }
}