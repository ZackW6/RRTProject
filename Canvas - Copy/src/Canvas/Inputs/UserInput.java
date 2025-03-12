package Canvas.Inputs;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.MouseWheelEvent;
import java.util.ArrayList;
import java.util.Scanner;

import javax.swing.SwingUtilities;
/**
 * Class for all user input actions
 */
public class UserInput {
    
    public static String getNextLine() {
        Scanner scanner = new Scanner(System.in);
        String line = scanner.nextLine();
        return line;
    }

    public static int getNextInt() {
        Scanner scanner = new Scanner(System.in);
        int line = scanner.nextInt();
        return line;
    }

    public static double getNextDouble() {
        Scanner scanner = new Scanner(System.in);
        double line = scanner.nextDouble();
        return line;
    }
}