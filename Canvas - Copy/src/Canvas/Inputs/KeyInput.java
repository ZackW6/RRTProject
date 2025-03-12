package Canvas.Inputs;

import javax.swing.JFrame;
import javax.swing.JPanel;

import Canvas.Commands.Trigger;
import Canvas.Shapes.VisualJ;

import java.awt.Toolkit;
import java.awt.datatransfer.Clipboard;
import java.awt.datatransfer.DataFlavor;
import java.awt.datatransfer.Transferable;
import java.awt.datatransfer.UnsupportedFlavorException;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;
import java.util.function.BooleanSupplier;

public class KeyInput implements KeyListener {
    @SuppressWarnings("unchecked")
    ArrayList<Object>[] pressedKeys = new ArrayList[2];

    private boolean isGathering = false;
    private ArrayList<String> currentGather = new ArrayList<>();
    private boolean ctrlPressed = false;


    public KeyInput(VisualJ vis) {
        currentGather.add("");
        pressedKeys[0] = new ArrayList<Object>();
        pressedKeys[1] = new ArrayList<Object>();

        vis.addKeyListener(this);
    }

    @Override
    public void keyTyped(KeyEvent e) {
        // Invoked when a key is typed. Uses KeyChar, a character output
        // System.out.println("Key Typed: " + e.getKeyChar());
    }

    @Override
    public void keyPressed(KeyEvent e) {
        // Invoked when a physical key is pressed down. Uses KeyCode, an int
        // System.out.println("Key Pressed: " + KeyEvent.getKeyText(e.getKeyCode()));
        for (int i  = 0; i < pressedKeys[0].size(); i++){
            if (KeyEvent.getKeyText(e.getKeyCode()).equals(pressedKeys[0].get(i)) || String.valueOf(e.getKeyChar()).equals(pressedKeys[0].get(i))){
                pressedKeys[1].set(i, true);
                break;
            }
        }
        if (isGathering){
            if (e.getKeyCode() == KeyEvent.VK_CONTROL) {
                ctrlPressed = true;
            }
            if (ctrlPressed && e.getKeyCode() == KeyEvent.VK_V){
                // lastWasCtrl = true;
                Clipboard clipboard = Toolkit.getDefaultToolkit().getSystemClipboard();
                // Get the clipboard content as a string
                Transferable contents = clipboard.getContents(null);
                if (contents != null && contents.isDataFlavorSupported(DataFlavor.stringFlavor)) {
                    try {
                        String clipboardText = (String) contents.getTransferData(DataFlavor.stringFlavor);
                        currentGather.set(currentGather.size()-1, currentGather.get(currentGather.size()-1)+clipboardText);
                    } catch (UnsupportedFlavorException e1) {
                        // TODO Auto-generated catch block
                        e1.printStackTrace();
                    } catch (IOException e1) {
                        // TODO Auto-generated catch block
                        e1.printStackTrace();
                    }
                }
                return;
            }

            if (KeyEvent.getKeyText(e.getKeyCode()).equals("Backspace")){
                if (currentGather.get(currentGather.size()-1).length()>0){
                    currentGather.set(currentGather.size()-1, currentGather.get(currentGather.size()-1).substring(0,currentGather.get(currentGather.size()-1).length()-1));
                }else{
                    if (currentGather.size()>1){
                        currentGather.remove(currentGather.size()-1);
                    }
                }
                return;
            }

            if (e.getKeyCode() == KeyEvent.VK_ENTER){
                currentGather.add("");
                return;
            }

            if ((int)e.getKeyChar() == 65535) {
                return;
            }
            currentGather.set(currentGather.size()-1, currentGather.get(currentGather.size()-1)+e.getKeyChar());
        }

    }

    @Override
    public void keyReleased(KeyEvent e) {
        // Invoked when a physical key is released.
        // System.out.println("Key Released: " + KeyEvent.getKeyText(e.getKeyCode()));
        for (int i  = 0; i < pressedKeys[0].size(); i++){
            if (KeyEvent.getKeyText(e.getKeyCode()).equals(pressedKeys[0].get(i)) || String.valueOf(e.getKeyChar()).equals(pressedKeys[0].get(i))){
                pressedKeys[1].set(i, false);
                break;
            }
        }
        if (e.getKeyCode() == KeyEvent.VK_CONTROL) {
            ctrlPressed = false;
        }
    }

    private boolean checkKeyPressed(String key){
        return (boolean)pressedKeys[1].get(pressedKeys[0].indexOf(key));
    }

    public Trigger keyPressed(String key){
        if (pressedKeys[0].contains(key)){
            return new Trigger(()->checkKeyPressed(key));
        }
        pressedKeys[0].add(key);
        pressedKeys[1].add(false);
        return new Trigger(()->checkKeyPressed(key));
    }

    public void beginCheck(String key){
        if (pressedKeys[0].contains(key)){
            return;
        }
        pressedKeys[0].add(key);
        pressedKeys[1].add(false);
    }

    public void beginGatherAll(){
        isGathering = true;
    }

    public List<String> getCurrentGather(){
        return currentGather;
    }

    public void setCurrentGather(List<String> list){
        this.currentGather = new ArrayList<>(list);
    }

    public void endGatherAll(){
        isGathering = false;
    }
}