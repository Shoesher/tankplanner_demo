package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class operatorinterface extends SubsystemBase{
    private static operatorinterface oi = null;
    private drive tank = drive.getInstance();
    private XboxController controller1;
    private int driverPort = 0;

    private operatorinterface(){
        controller1 = new XboxController(driverPort);
    }

    private void updateDrive(){
        tank.runDrive(controller1.getRawAxis(1), controller1.getRawAxis(2));
        tank.speedMode(controller1.getLeftBumperButtonPressed(), controller1.getRightBumperPressed());
    }

    @Override
    public void periodic() {
        updateDrive();
    }

    public static operatorinterface getInstance(){
        if (oi == null){
            oi = new operatorinterface();
        }
        return oi;
    }
}