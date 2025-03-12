package Canvas.Util;
public class Random {
    public static int randInt(int start, int end){
        return (int)((Math.random()*(end-start))+start);
    }

    public static double randDouble(double start, double end){
        return ((Math.random()*(end-start))+start);
    }
}
