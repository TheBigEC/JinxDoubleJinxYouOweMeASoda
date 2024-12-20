package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RockPaperScissors extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005;
    private double m_lastSimTime;
    private Notifier m_simNotifier = null;
    // just put my drive in the bag bro
      private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
      private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();


    // this will be my eventaul command swerve drive class


    TalonFX motor;
    private static RockPaperScissors instance = null;



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
   

// This bad boys set Power
    public void run(double percent){
        motor.set(percent);
    }

     public Command drive(double xInput, double yInput, double rInput){
        return run(() -> drive.withVelocityX(xInput).withVelocityY(yInput).withRotationalRate(rInput));
    }


     public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command brake(){
        return applyRequest(()->brake);
    }

    

    
    
}
