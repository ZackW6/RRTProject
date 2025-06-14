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
    public boolean middlePressed = false;

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
        MIDDLE(2),
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
    private ArrayList<Runnable>[][] events = new ArrayList[3][8];
    
    /**
     * Mouse side should be left for any that are not intended to moniter clicks
     * @param run
     * @param typeOfListener
     * @param rightOrLeft
     */
    public void addEvent(Runnable run, MouseInputs typeOfListener, MouseSide side){
        events[side.get()][typeOfListener.get()].add(run);
    }

    public ArrayList<Runnable> getEventList(MouseInputs typeOfListener, MouseSide side){
        return events[side.get()][typeOfListener.get()];
    }

    public void removeEvent(Runnable run, MouseInputs typeOfListener, MouseSide side){
        events[side.get()][typeOfListener.get()].remove(run);
    }

    /**
     * starts checks whenever any event triggers on mouse presses, movements, drags, etc...
     * any corrosponding Runnables are run
     * @param canvas
     */
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
                    for (Runnable run : events[0][0]){
                        run.run();
                    }
                }
                if (SwingUtilities.isRightMouseButton(e)) {
                    rightPressed = true;
                    for (Runnable run : events[1][0]){
                        run.run();
                    }
                }
                if (SwingUtilities.isMiddleMouseButton(e)) {
                    middlePressed = true;
                    for (Runnable run : events[2][0]){
                        run.run();
                    }
                }
            }
            @Override
            public void mouseReleased(MouseEvent e) {
                if (SwingUtilities.isLeftMouseButton(e)) {
                    leftPressed = false;
                    for (Runnable run : events[0][1]){
                        run.run();
                    }
                }
                if (SwingUtilities.isRightMouseButton(e)) {
                    rightPressed = false;
                    for (Runnable run : events[1][1]){
                        run.run();
                    }
                }
                if (SwingUtilities.isMiddleMouseButton(e)) {
                    middlePressed = false;
                    for (Runnable run : events[2][1]){
                        run.run();
                    }
                }
            }
            @Override
            public void mouseClicked(MouseEvent e) {
                if (SwingUtilities.isLeftMouseButton(e)) {
                    for (Runnable run : events[0][2]){
                        run.run();
                    }
                }
                if (SwingUtilities.isRightMouseButton(e)) {
                    for (Runnable run : events[1][2]){
                        run.run();
                    }
                }
                if (SwingUtilities.isMiddleMouseButton(e)) {
                    for (Runnable run : events[2][2]){
                        run.run();
                    }
                }
            }
            @Override
            public void mouseEntered(MouseEvent e) {
                for (Runnable run : events[0][3]){
                    run.run();
                }
            }
            @Override
            public void mouseExited(MouseEvent e) {
                for (Runnable run : events[0][4]){
                    run.run();
                }
            }
            @Override
            public void mouseWheelMoved(MouseWheelEvent e){
                mouseWheel = e.getPreciseWheelRotation();
                for (Runnable run : events[0][5]){
                    run.run();
                }
            }
        });
        canvas.addMouseWheelListener(new MouseInputAdapter() {
            @Override
            public void mouseWheelMoved(MouseWheelEvent e){
                mouseWheel = e.getPreciseWheelRotation();
                for (Runnable run : events[0][5]){
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
                    
                    for (Runnable run : events[0][6]){
                        run.run();
                    }
                }
                if (SwingUtilities.isRightMouseButton(e)) {
                    for (Runnable run : events[1][6]){
                        run.run();
                    }
                }
                if (SwingUtilities.isMiddleMouseButton(e)) {
                    for (Runnable run : events[2][6]){
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

    public boolean isMiddleMousePressed(){
        return rightPressed;
    }

    public Trigger leftPressed(){
        return new Trigger(()->isLeftMousePressed());
    }

    public Trigger rightPressed(){
        return new Trigger(()->isRightMousePressed());
    }

    public Trigger middlePressed(){
        return new Trigger(()->isMiddleMousePressed());
    }
}

