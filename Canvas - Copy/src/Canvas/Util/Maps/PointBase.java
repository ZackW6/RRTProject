package Canvas.Util.Maps;

import java.util.List;
import java.util.function.Consumer;

import Canvas.Shapes.PolyShape;
import Canvas.Shapes.VisualJ;

public interface PointBase <T extends Point> {
    public void addAll(List<T> points);

    public void add(T point);

    public void removeAll(List<T> points);

    public void remove(T point);

    public T findNearest(T target);

    public List<T> findKNearest(T target, int k);

    public <V extends Point> List<T> findInRange(V lowerBound, V upperBound);

    public void clear();

    public void clear(PolyShape drawing);

    public void clear(VisualJ vis);

    public List<T> toList();

    public T getRoot();

    public void traverseNodes(Consumer<T> action);
}
