package Canvas.Util;
public class ArrMath {
    /**
     * add two arrays together
     * @param x
     * @param y
     * @return
     */
    public static int[] addArrs(int[] x,int[] y){
        int[] a=y;
        int[] z=x;
        for (int i=0;i<z.length;i++){
            z[i]+=a[i];
        }
        return z;
    }
    /**
     * subtract one array from another
     * @param x
     * @param y
     * @return
     */
    public static int[] minusArrs(int[] x,int[] y){
      int[] a=y;
      int[] z=x;
      for (int i=0;i<z.length;i++){
          z[i]-=a[i];
      }
      return z;
    }

    public static double[] intToDoubleArray(int[] arr) {
        double[] temp = new double[arr.length];

        for (int i = 0; i < arr.length; i++) {
            temp[i] = arr[i];
        }
        return temp;
    }
    
    public static int[] doubleToIntArray(double[] arr) {
        int[] temp = new int[arr.length];

        for (int i = 0; i < arr.length; i++) {
            temp[i] = (int)arr[i];
        }

        return temp;
    }
}
