package Canvas.Util;

import java.util.List;
import java.util.TreeSet;

public class RangeList<T extends Comparable<T>>{
    private final TreeSet<T> set = new TreeSet<>();

    public boolean add(T element) {
        return set.add(element);
    }

    public boolean addAll(List<T> list){
        return set.addAll(list);
    }

    public boolean remove(T element) {
        return set.remove(element);
    }

    public boolean removeAll(List<T> list){
        return set.removeAll(list);
    }

    public List<T> findInRange(T start, T end) {
        return List.copyOf(set.subSet(start, true, end, true));
    }
}
