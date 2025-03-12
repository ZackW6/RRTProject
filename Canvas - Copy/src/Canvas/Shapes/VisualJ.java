package Canvas.Shapes;
import javax.swing.*;

import Canvas.Inputs.UserInput;
import Canvas.Util.Profile;
import Canvas.Util.Vector2D;

import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.image.BufferedImage;
import java.util.ArrayList;

public class VisualJ extends JFrame{

    private Profile profile = new Profile();
    public ArrayList<Obj> shapes=new ArrayList<Obj>();
    //shapes encompasses everything that will be included in the graphics project
    public int WIDTH;
    public int HEIGHT;

    private BufferedImage buffer;

    private double zoomRatio = 1;
    private Vector2D moveScreen;
    private double timeStep;
    private double lastIterationTime = 0;
    @Override
    public void paint(Graphics g) {
        
        profile.start();
        if (buffer == null || buffer.getWidth() != getWidth() || buffer.getHeight() != getHeight()) {
            // Create a new buffer if not initialized or if the size has changed
            buffer = new BufferedImage(getWidth(), getHeight(), BufferedImage.TYPE_INT_ARGB);
        }
        // Get the graphics context of the buffer
        Graphics2D g2dBuffer = buffer.createGraphics();
        
        g2dBuffer.setColor(getBackground());
        g2dBuffer.translate(0, HEIGHT);
        g2dBuffer.scale(1, -1);
        g2dBuffer.fillRect(0, 0, WIDTH, HEIGHT);
        g2dBuffer.translate(WIDTH/2,HEIGHT/2);
        try {
            for (int i = 0; i < shapes.size(); i++) {
                if (shapes.get(i)!=null){
                    double xp = (-WIDTH/2 + moveScreen.x + shapes.get(i).coords.x + shapes.get(i).addedCoords.x) * zoomRatio;
                    double yp = (-HEIGHT/2 + moveScreen.y + shapes.get(i).coords.y + shapes.get(i).addedCoords.y) * zoomRatio;
                    double radians = (Math.toRadians(shapes.get(i).degree));
                    double rotX = xp + (shapes.get(i).width/2) * zoomRatio;
                    double rotY = yp + (shapes.get(i).height/2) * zoomRatio;
                    g2dBuffer.setColor(shapes.get(i).col);
                    g2dBuffer.translate((int)rotX,(int)rotY);
                    g2dBuffer.rotate(radians);
                    shapes.get(i).show(g2dBuffer, zoomRatio);
                    g2dBuffer.rotate(-radians);
                    g2dBuffer.translate((int)-rotX,(int)-rotY);
                }
            }
        } catch (Exception e) {
            System.out.println("CaughtVIS");
        }
        
        g2dBuffer.setColor(Color.RED);
        lastIterationTime = profile.getTime();
        g.drawImage(buffer, 0, 0, this);
        profile.stop();
        timeStep++;
    }

    public void createWorld(String title,int width, int height,Color color){
        setTitle(title);
        setSize(width, height);
        setBackground(color);
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        buffer = new BufferedImage(WIDTH, HEIGHT, BufferedImage.TYPE_INT_ARGB);
    }

    /**
     * Get the array of objects currently being presented
     * @return
     */
    public ArrayList<Obj> getObjArray(){
        return shapes;
    }

    /**
     * Remove an object from the current array of objects
     * @param object
     */
    public void remove(Obj object){
        shapes.remove(object);
    }

    /**
     * Add an object to the current array of objects
     * @param object
     */
    public void add(Obj object){
        shapes.add(object);
    }

    /**
     * Start the graphics thread, which runs as fast as possible
     */
    public void startThread() {
        Thread animationThread = new Thread(() -> {
            while (true) {
                repaint(); // Request a repaint of the component
                try {
                    Thread.sleep((long) 0);
                } catch (InterruptedException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                }
            }
        });
        animationThread.start(); // Start the animation thread
    }


    public int getBackgroundHeight(){
        return HEIGHT;
    }

    public int getBackgroundWidth(){
        return WIDTH;
    }

    public VisualJ(String title,int width, int height, Color background){
        WIDTH=width;
        HEIGHT=height;
        moveScreen = new Vector2D(0, 0);
        SwingUtilities.invokeLater(() -> setVisible(true));
        setTitle(title);
        setSize(width, height);
        setBackground(background);
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        buffer = new BufferedImage(WIDTH, HEIGHT, BufferedImage.TYPE_INT_ARGB);
    }

    public double getTimeStep(){
        return timeStep;
    }

    public void moveFrame(double x, double y){
        moveScreen.x += x;
        moveScreen.y += y;
    }

    public void setFrame(double x, double y){
        moveScreen.x = x;
        moveScreen.y = y;
    }

    public Vector2D getFrameMove(){
        return Vector2D.of(moveScreen.x, moveScreen.y);
    }

    public void setZoom(double zoomRatio){
        this.zoomRatio = zoomRatio;
        if (this.zoomRatio < 0){
            this.zoomRatio = 0;
        }
    }

    public double getZoom(){
        return zoomRatio;
    }

    public boolean moveIndex(Obj object, int index){
        int currentIndex = shapes.indexOf(object);
        if (currentIndex==-1){
            return false;
        }
        Obj temp = shapes.get(index);
        shapes.set(index, object);
        shapes.set(currentIndex, temp);
        return true;
    }

    public int getIndex(Obj object){
        return shapes.indexOf(object);
    }

    public double getIterationTime(){
        return lastIterationTime;
    }

    public Vector2D screenRelativePoint(Vector2D point){
        return Vector2D.of((point.x-WIDTH/2)*1/getZoom()+WIDTH/2 - getFrameMove().x,(point.y-HEIGHT/2)*1/getZoom()+HEIGHT/2 - getFrameMove().y);
    }
}