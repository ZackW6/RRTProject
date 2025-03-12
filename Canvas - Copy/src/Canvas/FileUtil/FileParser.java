package Canvas.FileUtil;

import java.util.ArrayList;
import java.util.List;

import Canvas.Pathing.RRT.Obstacle;
import Canvas.Pathing.RRT.RRTBase;
import Canvas.Pathing.RRT.RRTHelperBase;
import Canvas.Shapes.VisualJ;

public class FileParser {

    private final String path;

    /**
     * give the file path to the containing folder, not the whole file
     * @param path
     */
    public FileParser(String path){
        String temp = path.replace("\\","/");
        this.path = temp.replace("\"","");
    }

    public List<Obstacle> loadSquares(RRTBase rrt, String fileName){

        FileWriter fileWriter = new FileWriter(path);

        List<String> contents = fileWriter.readFile(fileName+".json");
        if (contents.size()!=0){

            ArrayList<Obstacle> obstacles = new ArrayList<>();

            String content = contents.get(0);

            int xSizeIndex = content.indexOf("x\":");
            int ySizeIndex = content.indexOf("y\":");
            int nodeSizeIndex = content.indexOf("s\":");
            double xSize = Double.parseDouble(content.substring(xSizeIndex, ySizeIndex).replaceAll("[^0-9.]", ""));
            double ySize = Double.parseDouble(content.substring(ySizeIndex, nodeSizeIndex).replaceAll("[^0-9.]", ""));
            double nodeSize = Double.parseDouble(content.substring(nodeSizeIndex,nodeSizeIndex+10).replaceAll("[^0-9.]", ""));
            int xNum = (int)(xSize/nodeSize);
            int yNum = (int)(ySize/nodeSize);
            double almostActualX = ((double)rrt.getField().width())/(double)xNum;
            double almostActualY = ((double)rrt.getField().height())/(double)yNum;
            double actualXSize = almostActualX - ((double)almostActualX)/(double)xNum;
            double actualYSize = almostActualY - ((double)almostActualY)/(double)yNum;

            int row = 0;
            int col = 0;
            
            while(content.contains("true") || content.contains("false")){
                int trueIndex = content.indexOf("true");
                int falseIndex = content.indexOf("false");
                int usedIndex = -1;
                if (trueIndex<0){
                    trueIndex = content.length()-1;
                }
                if (falseIndex<0){
                    falseIndex = content.length()-1;
                }
                if (trueIndex<falseIndex){
                    usedIndex = trueIndex;
                    Obstacle obstacleSquare = new Obstacle(rrt.getField().x()+(col*actualXSize), rrt.getField().y()+(row*actualYSize), actualXSize, actualYSize, true);
                    obstacles.add(obstacleSquare);
                }else{
                    usedIndex = falseIndex;
                    Obstacle obstacleSquare = new Obstacle(rrt.getField().x()+(col*actualXSize), rrt.getField().y()+(row*actualYSize), actualXSize, actualYSize, false);
                    obstacles.add(obstacleSquare);
                }
                

                col++;
                if (col == xNum+1){
                    col = 0;
                    row++;
                }
                content = content.substring(0, usedIndex) + content.substring(usedIndex+2);
            }
            return obstacles;
        }
        return List.of();
        
    }
}
