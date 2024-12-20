package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;


public class RockPaperScissors extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005;
    private double m_lastSimTime;
    private Notifier m_simNotifier = null;
    private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
    private double MaxAngularRate = 1.5 * Math.PI;
    // just put my drive in the bag bro
        
    private final SwerveRequest.PointWheelsAt pointWheels = new SwerveRequest.PointWheelsAt();

      private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

          private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 
      
      private final SwerveRequest.RobotCentric robotDrive = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.1)
      .withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);


    // this will be my eventaul command swerve drive class


    TalonFX motor;



     public RockPaperScissors(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    public RockPaperScissors(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }


     private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

       // Run sum at faster rate
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            // get battery voltage from WP lib and use delta time
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public Command defenseMode() {
        return run(() -> {
            // Point all wheels inward to make the robot harder to push
            setControl(pointWheels.withModuleDirection(new Rotation2d(Math.PI/4)));
        });
    }

    // please tell me all this works and doesnt diqualify for "custom math"
   
    public Command robotCentricDrive(double xInput, double yInput, double rInput) {
        return run(() -> robotDrive
            .withVelocityX(xInput)
            .withVelocityY(yInput)
            .withRotationalRate(rInput));
    }

    // robot centric drive just a new feature ig may or may not be used
    

     public Command drive(double xInput, double yInput, double rInput){
        return run(() -> drive.withVelocityX(xInput).withVelocityY(yInput).withRotationalRate(rInput));
    }
    // to be used for field centric drive
    public Command driveSlow(double xInput, double yInput, double rInput, double speedMultiplier) {
        return run(() -> drive
            .withVelocityX(xInput * speedMultiplier)  // Scale the X velocity
            .withVelocityY(yInput * speedMultiplier)  // Scale the Y velocity
            .withRotationalRate(rInput * speedMultiplier));  // Scale the rotation rate
    }


     public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command brake(){
        return applyRequest(()->brake);
    }

    

   

    
    //gyro to track the robot's heading for field-centric control
    //periodic update of the robot's headin
    @Override
    public void periodic() {
    // Update the field-relative orientation
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
        this.setOperatorPerspectiveForward(
            alliance.get() == Alliance.Red ? 
                Rotation2d.fromDegrees(180) : 
                Rotation2d.fromDegrees(0)
        );
    }
}

    

    
    
}
