package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.File;
import java.util.Map;
import java.io.IOException;
import com.fasterxml.jackson.databind.ObjectMapper;

public class tankplanner extends SubsystemBase{
    private ObjectMapper mapper;
    private String defaultFilePath;
    private File pathFile = null;

    //temporary import before .this is used in the drive class
    private drive tank = drive.getInstance();
    private double currentHeading = tank.calcGyro;
    

    private tankplanner(){
        mapper = new ObjectMapper();
        defaultFilePath = "C:/Users/shahr/AppData/Roaming/com.example/tankplanner/TankPlannerPaths/";
    }

    public void loadPath(String pathName) throws IOException{
        pathFile = new File(defaultFilePath + pathName + ".json");
        try {
            @SuppressWarnings("unchecked")
            Map<String, Object> data = mapper.readValue(pathFile, Map.class);
            data.get(data);

        } catch (IOException e) {
            throw new IOException("This path file not found, check spelling: " + pathFile.getAbsolutePath(), e);
        }    
    }
}