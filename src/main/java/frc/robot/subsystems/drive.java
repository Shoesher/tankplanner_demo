package frc.robot.subsystems;

import frc.robot.constants.Constants;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class drive extends SubsystemBase{
    private static drive tank = null;

    //Electronics
    private SparkMax frontLeft;
    private SparkMax frontRight;
    private SparkMax rearLeft;
    private SparkMax rearRight;
    private RelativeEncoder rightEncoder;
    private RelativeEncoder leftEncoder;
    private Pigeon2 gyro;

    //Odometry
    private DifferentialDriveOdometry odometry;
    public double calcGyro = 0;
    private double speed = 1;
    private Rotation2d fakeHeading;
    private int motorRPM = 5676; //Assuming you are using sims
    private double driveGearing = 8.46; //Your gear ratio may be different, check your chassis
    private double wheelCirc = 0.48; //Wheel circumference in meters

    //Telemetry
    private double distance = 0;
    public double gear = 4;
    public double speedMetersPerSecond = 0;

    //Advantage scope
    private StructArrayPublisher<SwerveModuleState> publisher;
    private StructPublisher<Rotation2d> publisher2d;
    private StructPublisher<Pose3d> publisher3d;
    private StructPublisher<ChassisSpeeds> publisherSpeed;
    private Pose2d currentPose2d;
    private Pose3d currentPose3d;

    //CAN Device IDs
    private int frontLeftID = 1;
    private int frontRightID = 2;
    private int rearLeftID = 3;
    private int rearRightID = 4;
    private int pigeonID = 5;

    //Configs
    private SparkMaxConfig config1;
    private SparkMaxConfig config2;
    private SparkMaxConfig config3;
    private SparkMaxConfig config4;

    private drive(){
        //CAN initializations
        frontLeft = new SparkMax(frontLeftID, MotorType.kBrushless);
        frontRight = new SparkMax(frontRightID, MotorType.kBrushless);
        rearLeft = new SparkMax(rearLeftID, MotorType.kBrushless);
        rearRight = new SparkMax(rearRightID, MotorType.kBrushless);
        gyro = new Pigeon2(pigeonID);

        leftEncoder = frontLeft.getEncoder();
        rightEncoder = frontRight.getEncoder();

        //Motor configuration
        config1 = new SparkMaxConfig();
        config2 = new SparkMaxConfig();
        config3 = new SparkMaxConfig();
        config4 = new SparkMaxConfig();

        config1.inverted(true);
        config2.inverted(true).follow(frontLeftID);
        config3.inverted(false);
        config4.inverted(false).follow(frontRightID);

        frontLeft.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        frontLeft.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rearLeft.configure(config3, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rearLeft.configure(config4, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //Advantage Scope
        publisher = NetworkTableInstance.getDefault().getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();
        publisher2d = NetworkTableInstance.getDefault().getStructTopic("MyRotation", Rotation2d.struct).publish();
        publisherSpeed = NetworkTableInstance.getDefault().getStructTopic("MyChassisSpeed", ChassisSpeeds.struct).publish();
        publisher3d = NetworkTableInstance.getDefault().getStructTopic("/AdvantageScope/Robot/Pose", Pose3d.struct).publish();
        fakeHeading = new Rotation2d();
        odometry = new DifferentialDriveOdometry(fakeHeading, leftEncoder.getPosition()/driveGearing*wheelCirc, rightEncoder.getPosition()/driveGearing*wheelCirc);
    }

    public Pose2d getPose(){
        Pose2d pose = odometry.getPoseMeters();
        return pose;
    }

    public void runDrive(double leftJoy, double rightJoy){
        if(Math.abs(leftJoy) > 0.1 || Math.abs(rightJoy) > 0.1){
            frontLeft.set(leftJoy + rightJoy * speed);
            frontRight.set(leftJoy - rightJoy * speed);
            //Simulates linear velocity and rotational velocity of your robot in advantage scope
            fakeHeading = Rotation2d.fromDegrees(simulateGyro(rightJoy));
            publisher2d.set(fakeHeading);
            simulateDistance(leftJoy, rightJoy, motorRPM, wheelCirc, driveGearing, speed);
        }
        else{
            stopDrive();
        }
    }

    private void stopDrive(){
        frontLeft.set(0);
        frontRight.set(0);
    }

    public void speedMode(boolean leftTrigger, boolean rightTrigger){
        double increment = 0.25;
        double minSpeed = 0.25;
        int maxSpeed = 1;
        if(leftTrigger){
            speed = (speed > 0.25) ? speed - increment : minSpeed;
            gear -= 1;
        }
        if(rightTrigger){
            speed = (speed < 1) ? speed + increment : maxSpeed;
            gear += 1;
        }
    }

    private double simulateGyro(double joystick){
        if(Math.abs(joystick) > 0.1){
            calcGyro = calcGyro - (5*joystick);
        }
        return calcGyro;
    }

    private void simulateDistance(double joyInput, double joyInput2, int maxRPM, double wheelCirc, double driveGear, double speedLimit){
        int minute = 60;
        speedMetersPerSecond = ((((maxRPM*joyInput)/driveGear)/minute)*wheelCirc)*speedLimit;
        if(Math.abs(joyInput) > 0.1){
            distance += speedMetersPerSecond;
            rightEncoder.setPosition(-distance);
            leftEncoder.setPosition(-distance);
        }
    }

    public void driveToSimDistance(double distanceMeters){
        distance += distanceMeters;
        rightEncoder.setPosition(-distance);
        leftEncoder.setPosition(-distance);
    }

    public void rotateByDeg(double rotDelta){
        calcGyro += rotDelta;
    }

    @Override
    public void periodic() {
        odometry.update(fakeHeading, leftEncoder.getPosition()/driveGearing*wheelCirc, rightEncoder.getPosition()/driveGearing*wheelCirc);
        //Generate a pose 3d for advantage scope
        currentPose2d = getPose();
        currentPose3d = new Pose3d(currentPose2d.getTranslation().getX(), currentPose2d.getTranslation().getY(), 0, new Rotation3d(fakeHeading));
        //Send updated pose to advantage scope
        publisher3d.set(currentPose3d);
    }

    public static drive getInstance(){
        if (tank == null){
            tank = new drive();
        }
        return tank;
    }
}   