package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class telemetry extends SubsystemBase{
    private static telemetry tele = null;
    private PowerDistribution pdp;
    private Field2d field2d;
    private int pdpID = 6;
    private drive tank = drive.getInstance();

    private telemetry(){
        pdp = new PowerDistribution(pdpID, ModuleType.kCTRE);
        field2d = new Field2d();
        //Send field2d to the dashboard
        SmartDashboard.putData("Field ", field2d);
    }

    public void update(){
        //Send match time to the dashboard
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

        //Send current gear to the dashboard
        double currentGear = tank.gear;
        SmartDashboard.putNumber("Gear ", currentGear);

        //Send current linear velocity to the dashboard
        double currentSpeed = tank.speedMetersPerSecond;
        SmartDashboard.putNumber("Velocity ", currentSpeed);

        //Send current rotational velocity to the dashboard
        double currentRotation = tank.calcGyro;
        SmartDashboard.putNumber("Rotation ", currentRotation);
        
        //Send current voltage to the dashboard
        double voltage = pdp.getVoltage();
        SmartDashboard.putNumber("Voltage ", voltage);

        //Update robot position on the 2d field inside the dashboard
        field2d.setRobotPose(tank.getPose());
    }

    public static telemetry getInstance(){
        if (tele == null){
            tele = new telemetry();
        }
        return tele;
    }
}   