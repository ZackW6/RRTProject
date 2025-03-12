package Canvas.Util;
import java.awt.*;
/**
 * Color extension class that allows for certain specific color getters
 */
public class ColorEXT {
    public static Color getColorBasedXY(int x, int y, int width, int height){
        int xval=(int)(((double)x/width)*255);
        int yval=(int)(((double)y/height)*255);
        int red = xval;
        int green = 0;
        int blue = yval;

        
        Color randomColor = new Color(red, green, blue);

        return randomColor;
    }
    public static Color getRandomColor() {
        

        
        int red = Random.randInt(0,256);
        int green = Random.randInt(0,256);
        int blue = Random.randInt(0,256);
        
        
        Color randomColor = new Color(red, green, blue);

        return randomColor;
    }
    public static Color getRandomGrayScale(){

        int col = Random.randInt(0,256);

        
        Color randomColor = new Color(col, col, col);

        return randomColor;
    }
}
