package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.io.IOException;
import com.fasterxml.jackson.databind.ObjectMapper;

public class tankplanner extends SubsystemBase{
    private ObjectMapper mapper;
    private String defaultFilePath;
    private File pathFile = null;
    private Path currentPath;   

    //temporary import before .this is used in the drive class
    private drive tank = drive.getInstance();
    private double currentHeading = tank.calcGyro;

    private tankplanner() {
        mapper = new ObjectMapper();
        //Migrate this to use System. to get the correct path to appdata
        defaultFilePath = "C:/Users/shahr/AppData/Roaming/com.example/tankplanner/TankPlannerPaths/";
    }

    private class Path {
        List<Trajectory> trajectories = new ArrayList<>();

        public void addTrajectory(Trajectory newTrajectory){
            trajectories.add(newTrajectory);
        }

        public Trajectory getTrajectory(int i){
            Trajectory validTraj = trajectories.get(i);

            if(validTraj != null) {
                return validTraj;
            }
            else {
                return null;
            }
        }
    }

    private class Trajectory {
       Point startPoint;
       Point endPoint;

       private double getlength(){
            double exp = 2;
            double xComp = Math.pow((endPoint.x - startPoint.x), exp);
            double yComp = Math.pow((endPoint.y - startPoint.y), exp);
            return Math.sqrt(xComp + yComp);
       }

       private double getRot(boolean startingPoint){
            double rot = startingPoint ? startPoint.rot : endPoint.rot;
            return rot;
       }
    }

    private class Point {
        double x;
        double y;
        double rot;
    }

    @SuppressWarnings("unchecked")
    public void loadPath(String pathName) throws IOException {
        pathFile = new File(defaultFilePath + pathName + ".json");
        currentPath = new Path();

        try {
            Map<String, Object> data = mapper.readValue(pathFile, Map.class);
            List<Map<String, Object>> trajectoriesData = (List<Map<String, Object>>) data.get("trajectories");

            for (Map<String, Object> trajectoryData : trajectoriesData){
                Map<String, Object> start = (Map<String, Object>) trajectoryData.get("startPoint");
                Map<String, Object> end = (Map<String, Object>) trajectoryData.get("endPoint");

                Point newStartPoint = buildPoint(start);
                Point newEndPoint = buildPoint(end);
                Trajectory newTraj = buildTrajectory(newStartPoint, newEndPoint);
                currentPath.addTrajectory(newTraj);
            }

            followPath();
        } catch (IOException e) {
            throw new IOException("This path file not found, check spelling: " + pathFile.getAbsolutePath(), e);
        }    
    }

    private Point buildPoint(Map<String, Object> pointData){
        Point point = new Point();
        point.x = ((Number) pointData.get("x")).doubleValue();
        point.y = ((Number) pointData.get("y")).doubleValue();
        point.rot = ((Number) pointData.get("rot")).doubleValue();
        return point;
    }

    private Trajectory buildTrajectory(Point point1, Point point2){
        Trajectory traj = new Trajectory();
        traj.startPoint = point1;
        traj.endPoint = point2;
        return traj;
    }

    private double optimizeDelta(double currentDeg, double targetDeg) {
        double optimizedAngle;
        double delta = targetDeg - currentDeg;
        double optDelta = targetDeg + (360 - currentDeg);
        optimizedAngle = (delta > 180) ? -optDelta : delta;
        return optimizedAngle;
    }

    private void followPath() { 
        for (Trajectory traj : currentPath.trajectories){
            double trajectoryLength = traj.getlength();
            double initialRot = optimizeDelta(currentHeading, traj.getRot(true));
            double finalRot = optimizeDelta(currentHeading, traj.getRot(false));
            
            tank.rotateByDeg(initialRot);
            tank.driveToSimDistance(trajectoryLength);
            tank.rotateByDeg(finalRot);
        }   
    }
}