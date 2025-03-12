package Canvas.Inputs;

import java.util.ArrayList;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.MouseMotionAdapter;
import java.awt.event.MouseWheelEvent;
import javax.swing.SwingUtilities;
import javax.swing.event.MouseInputAdapter;

import Canvas.Commands.Trigger;
import Canvas.Shapes.VisualJ;
import Canvas.Util.Vector2D;

public class MouseInput{

    public Vector2D mouseCoords = new Vector2D(0, 0);
    public double mouseWheel = 0;
    public boolean leftPressed = false;
    public boolean rightPressed = false;

    private VisualJ vis;
    public enum MouseInputs {
        MOUSE_PRESSED(0),
        MOUSE_RELEASED(1),
        MOUSE_CLICKED(2),
        MOUSE_ENTERED(3),
        MOUSE_EXITED(4),
        MOUSE_WHEEL_MOVED(5),
        MOUSE_DRAGGED(6),
        MOUSE_MOVED(7);
        private int val;
        private MouseInputs(int val) {
            this.val = val;
        }
        protected int get(){
            return val;
        }
    }
    public enum MouseSide {
        RIGHT(1),
        LEFT(0);
        private int val;
        private MouseSide(int val) {
            this.val = val;
        }
        protected int get(){
            return val;
        }
    }
    @SuppressWarnings("unchecked")
    private ArrayList<Runnable>[][] events = new ArrayList[2][8];
    
    /**
     * Mouse side should be left for any that are not intended to moniter clicks
     * @param run
     * @param typeOfListener
     * @param rightOrLeft
     */
    public void addEvent(Runnable run, MouseInputs typeOfListener, MouseSide rightOrLeft){
        events[rightOrLeft.get()][typeOfListener.get()].add(run);
    }

    public ArrayList<Runnable> getEventList(MouseInputs typeOfListener, MouseSide rightOrLeft){
        return events[rightOrLeft.get()][typeOfListener.get()];
    }

    public void removeEvent(Runnable run, MouseInputs typeOfListener, MouseSide rightOrLeft){
        events[rightOrLeft.get()][typeOfListener.get()].remove(run);
    }

    public MouseInput(VisualJ canvas){
        this.vis = canvas;
        for (int i = 0; i<events.length;i++){
            for (int y = 0; y<events[i].length;y++){
                events[i][y] = new ArrayList<Runnable>();
            }
        }
        canvas.addMouseListener(new MouseAdapter() {
            @Override
            public void mousePressed(MouseEvent e) {
                if (SwingUtilities.isLeftMouseButton(e)) {
                    leftPressed = true;
                    for (Runnable run : events[0][0].toArray(new Runnable[events[0][0].size()])){
                        run.run();
                    }
                }
                if (SwingUtilities.isRightMouseButton(e)) {
                    rightPressed = true;
                    for (Runnable run : events[1][0].toArray(new Runnable[events[1][0].size()])){
                        run.run();
                    }
                }
            }
            @Override
            public void mouseReleased(MouseEvent e) {
                if (SwingUtilities.isLeftMouseButton(e)) {
                    leftPressed = false;
                    for (Runnable run : events[0][1].toArray(new Runnable[events[0][1].size()])){
                        run.run();
                    }
                }
                if (SwingUtilities.isRightMouseButton(e)) {
                    rightPressed = false;
                    for (Runnable run : events[1][1].toArray(new Runnable[events[1][1].size()])){
                        run.run();
                    }
                }
            }
            @Override
            public void mouseClicked(MouseEvent e) {
                if (SwingUtilities.isLeftMouseButton(e)) {
                    for (Runnable run : events[0][2].toArray(new Runnable[events[0][2].size()])){
                        run.run();
                    }
                }
                if (SwingUtilities.isRightMouseButton(e)) {
                    for (Runnable run : events[1][2].toArray(new Runnable[events[1][2].size()])){
                        run.run();
                    }
                }
            }
            @Override
            public void mouseEntered(MouseEvent e) {
                for (Runnable run : events[0][3].toArray(new Runnable[events[0][3].size()])){
                    run.run();
                }
            }
            @Override
            public void mouseExited(MouseEvent e) {
                for (Runnable run : events[0][4].toArray(new Runnable[events[0][4].size()])){
                    run.run();
                }
            }
            @Override
            public void mouseWheelMoved(MouseWheelEvent e){
                mouseWheel = e.getPreciseWheelRotation();
                for (Runnable run : events[0][5].toArray(new Runnable[events[0][5].size()])){
                    run.run();
                }
            }
        });
        canvas.addMouseWheelListener(new MouseInputAdapter() {
            @Override
            public void mouseWheelMoved(MouseWheelEvent e){
                mouseWheel = e.getPreciseWheelRotation();
                for (Runnable run : events[0][5].toArray(new Runnable[events[0][5].size()])){
                    run.run();
                }
            }
        });
        canvas.addMouseMotionListener(new MouseMotionAdapter() {
            @Override
            public void mouseDragged(MouseEvent e){
                mouseCoords.x = e.getX();
                mouseCoords.y = e.getY();
                if (SwingUtilities.isLeftMouseButton(e)) {
                    
                    for (Runnable run : events[0][6].toArray(new Runnable[events[0][6].size()])){
                        run.run();
                    }
                }
                if (SwingUtilities.isRightMouseButton(e)) {
                    for (Runnable run : events[1][6]){
                        run.run();
                    }
                }
            }
            @Override
            public void mouseMoved(MouseEvent e){
                mouseCoords.x = e.getX();
                mouseCoords.y = e.getY();
                for (Runnable run : events[0][7]){
                    run.run();
                }
            }
        });
    }

    public Vector2D getMouseCoords() {
        return Vector2D.of(mouseCoords.x, vis.HEIGHT - mouseCoords.y);
    }

    /**
     * returns 1, 0 or -1 per call telling direction
     * @return
     */
    public double getMouseWheelPosition(){
        return mouseWheel;
    }

    public boolean isLeftMousePressed(){
        return leftPressed;
    }

    public boolean isRightMousePressed(){
        return rightPressed;
    }

    public Trigger leftPressed(){
        return new Trigger(()->isLeftMousePressed());
    }

    public Trigger rightPressed(){
        return new Trigger(()->isRightMousePressed());
    }
}

