package Canvas.FileUtil;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.Scanner;

public class FileWriter {

    private String path;
    /**
     * just give the filepath down to your last folder, not the file name though
     * @param path
     */
    public FileWriter(String path){
        this.path = path.replace("\\","/");
        this.path = this.path.replace("\"","");
    }

    public void deleteFile(String fileName){
        new File(path+"/"+fileName).delete();
    }

    /**
     * returns list each index of the list is a line
     * @param fileName
     * @return
     */
    public List<String> readFile(String fileName){
        Path filesToRead = Paths.get(path, fileName);
        try {
            return  Files.readAllLines(filesToRead);
        } catch (IOException e) {
            return null;
        }
    }

    /**
     * each index of the content list is one line
     * @param fileName
     * @param content
     */
    public void writeFile(String fileName, List<String> content){
        Path filesToWrite = Paths.get(path, fileName);
        
        try {
            Files.write(filesToWrite, content);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    public void changePath(String newPath){
        this.path = newPath;
    }

    // public void savePath(String callBack, String savePath){



    //     writeFile("C:/GitHub/VisualProject/Canvas - Copy/src/Canvas/FileUtil",List.of(callBack));
    // }
}