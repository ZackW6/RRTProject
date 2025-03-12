package Canvas.Shapes;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.AffineTransform;
import java.util.ArrayList;
import java.util.List;

import Canvas.Util.Vector2D;

public class PolyShape extends Obj{

    private ArrayList<Obj> shapes = new ArrayList<>();

    /**
     * This class takes in a list of other shapes, and uses their coords to orient the other shapes around
     * the given coord origin, remember the coord origin is based in the top left of the shape.
     * @param xcoord
     * @param ycoord
     * @param width
     * @param height
     * @param col
     * @param fill
     */
    public PolyShape(double xcoord, double ycoord, List<Obj> shapes) {
        super(xcoord, ycoord, 0, 0, Color.BLACK, false);

        this.shapes = new ArrayList<>(shapes);
        
        reconfigure();
    }

    public PolyShape(double xcoord, double ycoord) {
        super(xcoord, ycoord, 0, 0, Color.BLACK, false);

    }

    private void reconfigure(){
        Vector2D dimensions = getDimensions(shapes);

        this.width = dimensions.x;
        this.height = dimensions.y;
        addedCoords.x = -width/2;
        addedCoords.y = -height/2;
    }

    private static Vector2D getDimensions(List<Obj> shapes){
        if (shapes.size() <= 0){
            return Vector2D.of(0,0);
        }
        double greatx = shapes.get(0).coords.x + shapes.get(0).width;
        double greaty = shapes.get(0).coords.y + shapes.get(0).height;
        double leastx = shapes.get(0).coords.x;
        double leasty = shapes.get(0).coords.y;
        for (int i = 1; i < shapes.size(); i++){
            if (shapes.get(i).coords.x < leastx){
                leastx = shapes.get(i).coords.x;
            }
            if (shapes.get(i).coords.x + shapes.get(i).width > greatx){
                greatx = shapes.get(i).coords.x + shapes.get(i).width;
            }
            if (shapes.get(i).coords.y < leasty){
                leasty = shapes.get(i).coords.y;
            }
            if (shapes.get(i).coords.y + shapes.get(i).height > greaty){
                greaty = shapes.get(i).coords.y + shapes.get(i).height;
            }
        }
        double width = greatx - leastx;
        double height = greaty - leasty;
        return Vector2D.of(width, height);
    }

    private static Vector2D getTranslation(List<Obj> shapes){
        if (shapes.size() <= 0){
            return Vector2D.of(0,0);
        }
        double greatx = shapes.get(0).coords.x + shapes.get(0).width;
        double greaty = shapes.get(0).coords.y + shapes.get(0).height;
        double leastx = shapes.get(0).coords.x;
        double leasty = shapes.get(0).coords.y;
        for (int i = 1; i < shapes.size(); i++){
            if (shapes.get(i).coords.x < leastx){
                leastx = shapes.get(i).coords.x;
            }
            if (shapes.get(i).coords.x + shapes.get(i).width > greatx){
                greatx = shapes.get(i).coords.x + shapes.get(i).width;
            }
            if (shapes.get(i).coords.y < leasty){
                leasty = shapes.get(i).coords.y;
            }
            if (shapes.get(i).coords.y + shapes.get(i).height > greaty){
                greaty = shapes.get(i).coords.y + shapes.get(i).height;
            }
        }
        double width = (greatx + leastx)/2;
        double height = (greaty + leasty)/2;
        return Vector2D.of(width, height);
    }

    public Obj get(int index){
        return shapes.get(index);
    }

    public void add(Obj object){
        shapes.add(object);
        reconfigure();
    }

    public void add(int index, Obj object){
        shapes.add(index, object);
        reconfigure();
    }

    public boolean remove(){
        if (shapes.size() <= 0){
            return false;
        }
        shapes.remove(shapes.size()-1);
        reconfigure();
        return true;
    }

    public boolean remove(Obj object){
        if (shapes.indexOf(object) == -1){
            return false;
        }
        shapes.remove(object);
        reconfigure();
        return true;
    }

    public boolean remove(int index){
        if (shapes.size() <= index || index < 0){
            return false;
        }
        shapes.remove(index);
        reconfigure();
        return true;
    }

    public void set(int index, Obj object){
        shapes.set(index, object);
        reconfigure();
    }

    public boolean moveIndex(Obj object, int index){
        if (object == null){
            return false;
        }
        int currentIndex = shapes.indexOf(object);
        if (currentIndex==-1){
            return false;
        }
        Obj temp = shapes.get(index);
        shapes.set(index, object);
        shapes.set(currentIndex, temp);
        reconfigure();
        return true;
    }

    public int getIndex(Obj object){
        return shapes.indexOf(object);
    }

    public List<Obj> getArray(){
        return shapes;
    }
    
    @Override
    public void show(Graphics2D g2dBuffer, double zoomRatio) {
        // g2dBuffer.translate((width/2) * zoomRatio,(height/2) * zoomRatio);
        for (int i = 0; i < shapes.size(); i++){

            double xp = (shapes.get(i).coords.x + shapes.get(i).addedCoords.x)*zoomRatio;
            double yp = (shapes.get(i).coords.y + shapes.get(i).addedCoords.y)*zoomRatio;
            double rotX = xp + (shapes.get(i).width/2) * zoomRatio;
            double rotY = yp + (shapes.get(i).height/2) * zoomRatio;
            double radians = (Math.toRadians(shapes.get(i).degree));

            g2dBuffer.setColor(shapes.get(i).col);
            g2dBuffer.translate((int)rotX,(int)rotY);
            g2dBuffer.rotate(radians);
            shapes.get(i).show(g2dBuffer, zoomRatio);
            g2dBuffer.rotate(-radians);
            g2dBuffer.translate((int)-rotX,(int)-rotY);
        }
        // g2dBuffer.translate(-(width/2) * zoomRatio,-(height/2) * zoomRatio);
    }

    public int indexOf(Obj obj) {
        return shapes.indexOf(obj);
    }
}
