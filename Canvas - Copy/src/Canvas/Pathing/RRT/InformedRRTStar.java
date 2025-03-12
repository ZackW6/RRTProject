package Canvas.Pathing.RRT;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import Canvas.Shapes.VisualJ;
import Canvas.Util.Vector2D;

public class InformedRRTStar extends RRTStar{

    public InformedRRTStar(VisualJ vis, Field field) {
        super(vis, field);
    }

    /**
     * In this case bias only affect initial goal search, not Informed optimization
     */
    @Override
    public Node getRandomPoint(double bias) {
        if (bestCost == Double.POSITIVE_INFINITY) {
            return super.getRandomPoint(bias); // Initial random sampling
        }

        // Distance between start and goal
        double c = getStart().distanceTo(getGoal());

        // Semi-major axis (half of the current best path length)
        double a = bestCost / 2;

        // Semi-minor axis (calculated)
        double b = Math.sqrt(a * a - (c * c) / 4);

        // Midpoint between start and goal
        double midX = (getStart().x + getGoal().x) / 2;
        double midY = (getStart().y + getGoal().y) / 2;

        // Sampling within the unit circle
        double theta = 2 * Math.PI * Math.random(); // Angle
        double r = Math.sqrt(Math.random()); // Radius (uniform sampling)
        double xUnit = r * Math.cos(theta);
        double yUnit = r * Math.sin(theta);

        // Scale the unit circle to the ellipsoid dimensions
        double xEllipse = a * xUnit;
        double yEllipse = b * yUnit;

        // Rotate the ellipse to align with the start-goal line
        double angle = Math.atan2(getGoal().y - getStart().y, getGoal().x - getStart().x);
        double xRotated = xEllipse * Math.cos(angle) - yEllipse * Math.sin(angle);
        double yRotated = xEllipse * Math.sin(angle) + yEllipse * Math.cos(angle);

        // Translate the ellipse to be centered between start and goal
        double xFinal = midX + xRotated;
        double yFinal = midY + yRotated;

        // Return a new node within the ellipsoid
        return new Node(xFinal, yFinal);
    }
    
}
