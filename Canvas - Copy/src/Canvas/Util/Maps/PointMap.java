package Canvas.Util.Maps;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.function.Consumer;

import Canvas.Pathing.RRT.Node;
import Canvas.Shapes.PolyShape;
import Canvas.Shapes.VisualJ;
import Canvas.Util.DrawingAccessable;

public class PointMap <T extends Point> implements PointBase<T> {
    private final List<T> points = new ArrayList<>();

    @Override
    public void addAll(List<T> points){
        this.points.addAll(points);
    }
    @Override
    public void add(T point) {
        points.add(point);
    }

    @Override
    public void removeAll(List<T> points){
        for (T point : points){
            remove(point);
        }
    }

    @Override
    public void remove(T point) {
        points.removeIf(p -> p.getX() == point.getX() && p.getY() == point.getY());
    }

    @Override
    public T findNearest(T target) {
        T best = null;
        double bestDist = Double.MAX_VALUE;

        for (T p : points) {
            double dist = target.distanceTo(p);
            if (dist < bestDist) {
                bestDist = dist;
                best = p;
            }
        }

        return best;
    }

    @Override
    public List<T> findKNearest(T target, int k) {
        return points.stream()
            .sorted(Comparator.comparingDouble(target::distanceTo))
            .limit(k)
            .toList();
    }

    @Override
    public <V extends Point> List<T> findInRange(V lowerBound, V upperBound) {
        List<T> result = new ArrayList<>();
        for (T p : points) {
            if (p.getX() >= lowerBound.getX() && p.getX() <= upperBound.getX() &&
                p.getY() >= lowerBound.getY() && p.getY() <= upperBound.getY()) {
                result.add(p);
            }
        }
        return result;
    }

    @Override
    public void clear() {
        points.clear();
    }

    @Override
    public List<T> toList() {
        return new ArrayList<>(points);
    }
    @Override
    public void clear(PolyShape drawing) {
        for (Point p : points){
            if (p instanceof DrawingAccessable){
                drawing.remove(((DrawingAccessable)p).getObj());
            }
        }
    }
    @Override
    public void clear(VisualJ vis) {
        for (Point p : points){
            if (p instanceof DrawingAccessable){
                vis.remove(((DrawingAccessable)p).getObj());
            }
        }
    }
    @Override
    public T getRoot() {
        return points.get(0);
    }
    @Override
    public void traverseNodes(Consumer<T> action) {
        points.forEach(action);
    }
}
