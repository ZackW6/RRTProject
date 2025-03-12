package Canvas.Util;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.PriorityQueue;
import java.util.function.Consumer;

import Canvas.Pathing.RRT.Node;
import Canvas.Shapes.PolyShape;
import Canvas.Shapes.VisualJ;

public class KDTree<T extends Point> {
    public class KDNode {
        T point;
        KDNode left, right;
        boolean vertical;

        public KDNode(T point, boolean vertical) {
            this.point = point;
            this.vertical = vertical;
        }

        public T getCorrisponding(){
            return point;
        }
    }

    private KDNode root;

    public KDNode getRoot(){
        return root;
    }

    public void addAll(List<T> list){
        for (T vec : list){
            add(vec);
        }
    }

    public void add(T point) {
        root = add(root, point, true);
    }

    private KDNode add(KDNode node, T point, boolean vertical) {
        if (node == null) {
            return new KDNode(point, vertical);
        }

        if (vertical) {
            if (point.getX() < node.point.getX()) {
                node.left = add(node.left, point, !vertical);
            } else {
                node.right = add(node.right, point, !vertical);
            }
        } else {
            if (point.getY() < node.point.getY()) {
                node.left = add(node.left, point, !vertical);
            } else {
                node.right = add(node.right, point, !vertical);
            }
        }

        return node;
    }

    public void removeAll(List<T> list){
        for (T vec : list){
            remove(vec);
        }
    }

    public void remove(T point) {
        root = remove(root, point, true);
    }

    private KDNode remove(KDNode node, T point, boolean vertical) {
        if (node == null) {
            return null;
        }

        if (node.point.equals(point)) {
            // Node found, remove it
            if (node.right != null) {
                // Find the minimum node in the right subtree
                KDNode minNode = findMin(node.right, vertical);
                node.point = minNode.point;
                node.right = remove(node.right, minNode.point, !vertical);
            } else if (node.left != null) {
                // Find the minimum node in the left subtree
                KDNode minNode = findMin(node.left, vertical);
                node.point = minNode.point;
                node.right = remove(node.left, minNode.point, !vertical);
                node.left = null; // Ensure left subtree is removed
            } else {
                // Node is a leaf, remove it
                return null;
            }
        } else {
            if ((vertical && point.getX() < node.point.getX()) || (!vertical && point.getY() < node.point.getY())) {
                node.left = remove(node.left, point, !vertical);
            } else {
                node.right = remove(node.right, point, !vertical);
            }
        }
        return node;
    }

    private KDNode findMin(KDNode node, boolean vertical) {
        if (node == null) {
            return null;
        }

        KDNode minNode = node;
        if (vertical) {
            if (node.left != null) {
                KDNode leftMin = findMin(node.left, vertical);
                if (leftMin.point.getX() < minNode.point.getX()) {
                    minNode = leftMin;
                }
            }
        } else {
            if (node.left != null) {
                KDNode leftMin = findMin(node.left, vertical);
                if (leftMin.point.getY() < minNode.point.getY()) {
                    minNode = leftMin;
                }
            }
        }

        return minNode;
    }

    public T search(T target) {
        return search(root, target, true);
    }

    private T search(KDNode node, T target, boolean vertical) {
        if (node == null) {
            return null; // Target not found
        }

        if (target == null){
            return null;
        }

        if (node.point.equals(target)) {
            return node.point; // Target found
        }

        if (vertical) {
            if (target.getX() < node.point.getX()) {
                return search(node.left, target, !vertical);
            } else {
                return search(node.right, target, !vertical);
            }
        } else {
            if (target.getY() < node.point.getY()) {
                return search(node.left, target, !vertical);
            } else {
                return search(node.right, target, !vertical);
            }
        }
    }

    // Find the k nearest KDNodes to the target point
    public List<T> findKNearest(T target, int k) {
        PriorityQueue<KDNode> pq = new PriorityQueue<>(k, Comparator.comparingDouble(n -> -target.distanceTo(n.point)));
        findKNearest(root, target, pq, k, true);

        List<T> nearestPoints = new ArrayList<>();
        while (!pq.isEmpty()) {
            nearestPoints.add(pq.poll().point);
        }

        return nearestPoints;
    }

    private void findKNearest(KDNode node, T target, PriorityQueue<KDNode> pq, int k, boolean vertical) {
        if (node == null) {
            return;
        }

        double dist = target.distanceTo(node.point);
        if (pq.size() < k) {
            pq.offer(node);
        } else if (dist < target.distanceTo(pq.peek().point)) {
            pq.poll();
            pq.offer(node);
        }

        KDNode goodSide, badSide;
        if (vertical) {
            if (target.getX() < node.point.getX()) {
                goodSide = node.left;
                badSide = node.right;
            } else {
                goodSide = node.right;
                badSide = node.left;
            }
        } else {
            if (target.getY() < node.point.getY()) {
                goodSide = node.left;
                badSide = node.right;
            } else {
                goodSide = node.right;
                badSide = node.left;
            }
        }

        findKNearest(goodSide, target, pq, k, !vertical);

        if (vertical && Math.abs(target.getX() - node.point.getX()) < target.distanceTo(pq.peek().point) ||
            !vertical && Math.abs(target.getY() - node.point.getY()) < target.distanceTo(pq.peek().point)) {
            findKNearest(badSide, target, pq, k, !vertical);
        }
    }

    public <V extends Point> List<T> findInRange(V lowerBound, V upperBound) {
        List<T> result = new ArrayList<>();
        findInRange(root, lowerBound, upperBound, result);
        return result;
    }
    
    private <V extends Point> void findInRange(KDNode node, V lowerBound, V upperBound, List<T> result) {
        if (node == null) {
            return;
        }
    
        // Check if current node is within the bounding box
        if (node.point.getX() >= lowerBound.getX() && node.point.getX() <= upperBound.getX() &&
            node.point.getY() >= lowerBound.getY() && node.point.getY() <= upperBound.getY()) {
            result.add(node.point);
        }
    
        // Traverse left and right subtrees based on the dimension and bounds
        if (node.vertical) {
            if (lowerBound.getX() <= node.point.getX()) {
                findInRange(node.left, lowerBound, upperBound, result);
            }
            if (upperBound.getX() >= node.point.getX()) {
                findInRange(node.right, lowerBound, upperBound, result);
            }
        } else {
            if (lowerBound.getY() <= node.point.getY()) {
                findInRange(node.left, lowerBound, upperBound, result);
            }
            if (upperBound.getY() >= node.point.getY()) {
                findInRange(node.right, lowerBound, upperBound, result);
            }
        }
    }

    public void traverseNodes(Consumer<T> action) {
        traverseNodes(root, action);
    }

    private void traverseNodes(KDNode node, Consumer<T> action) {
        if (node == null) {
            return;
        }

        // Traverse left subtree
        traverseNodes(node.left, action);

        // Perform action on current node
        action.accept(node.point);

        // Traverse right subtree
        traverseNodes(node.right, action);
    }

    public List<T> toList() {
        List<T> result = new ArrayList<>();
        traverseNodes(point -> result.add(point));
        return result;
    }

    public void clear(){
        root = null;
    }

    /**
     * MUST ONLY BE USED FOR DrawingAccessable IMPLIMENTATIONS!!! will break otherwise
     * @param poly
     */
    public void clear(PolyShape poly){
        traverseNodes(point -> poly.remove(((DrawingAccessable) (point)).getObj()));
        root = null;
    }

    /**
     * MUST ONLY BE USED FOR DrawingAccessable IMPLIMENTATIONS!!! will break otherwise
     * @param poly
     */
    public void clear(VisualJ vis){
        traverseNodes(point -> vis.remove(((DrawingAccessable) (point)).getObj()));
        root = null;
    }
}
