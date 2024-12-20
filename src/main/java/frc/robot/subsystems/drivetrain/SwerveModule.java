package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.ESPN.E_Motor;

public class SwerveModule {


 private final String MODULE_NAME;
  private final Translation2d LOCATION;
  private final double GEAR_RATIO;
  private final int SPEED_ID;
  private final int ANGLE_ID;
  private final int ENCODER_ID;
  private final double ZERO;

  private final E_Motor speed;
  private final E_Motor angle;
  private final CANcoder encoder;

  private final PIDController anglePID;
  private final SimpleMotorFeedforward angleFF;
  private final PIDController speedPID;
  private final SimpleMotorFeedforward speedFF;


  public SwerveModule(String moduleName, Translation2d location, double gearRatio, int speedID, int angleID, int encoderID, double zero) {
    this.MODULE_NAME = moduleName;
    this.LOCATION = location;
    this.GEAR_RATIO = gearRatio;
    this.SPEED_ID = speedID;
    this.ANGLE_ID = angleID;
    this.ENCODER_ID = encoderID;
    this.ZERO = zero;
    // Just means that the variables in this object of this name will be the variables declared above, clears naming confusion to the compilier
    // I think

    this.anglePID = Constants.drivetrain.ANGLE_PID;
    this.angleFF = Constants.drivetrain.ANGLE_FF;
    this.speedPID = Constants.drivetrain.SPEED_PID;

    
    this.speedFF = new SimpleMotorFeedforward(Constants.drivetrain.SPEED_FF.ks, Constants.drivetrain.SPEED_FF.kv);

    this.speed = new E_Motor(SPEED_ID, "canivore");
    this.angle = new E_Motor(ANGLE_ID, "canivore");
    this.encoder = new CANcoder(ENCODER_ID, "canivore");
    // CanBus got

    // TempManager.addMotor(this.speed);
    // TempManager.addMotor(this.angle);

    
  }
  // TOTD - proper module initialization for can coder, try catch initialitazion.

 
  // Initlatialize method to be run in the constructer for every object of SwereModule created
  public void init(){
    
    try{

    // Changes the state of the motor 
    speed.setBrake(false);
    angle.setBrake(false);
    speed.setInverted(true);
    angle.setInverted(true);
     // Reset encoders and set initial positions
    speed.setPosition(0);

    // Set the angle motor to the absolute position from CANcoder

    double absolutePosition = encoder.getAbsolutePosition().getValue();
    angle.setPosition(absolutePosition);


    speed.getVelocity().setUpdateFrequency(50.0);
    speed.getRotorPosition().setUpdateFrequency(50.0);
    speed.getDeviceTemp().setUpdateFrequency(4.0);
    
    // Configures the Cancoders
    MagnetSensorConfigs sensorConfigs = new MagnetSensorConfigs();
    sensorConfigs.MagnetOffset = -(ZERO / 360.0);
    sensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    sensorConfigs.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoder.getConfigurator().apply(sensorConfigs);

    angle.getVelocity().setUpdateFrequency(50.0);
    angle.getRotorPosition().setUpdateFrequency(50.0);
    angle.getDeviceTemp().setUpdateFrequency(4.0);
    

    // update variables based on the freuquency of usuage and importance
    // 

    encoder.getAbsolutePosition().setUpdateFrequency(100.0);

    speed.optimizeBusUtilization();
    angle.optimizeBusUtilization();
    encoder.optimizeBusUtilization();



   // Optimizes the device's bus utilization by reducing the update frequencies of its status signals.
  // clears telementary 
    speed.clearStickyFaults();
    angle.clearStickyFaults();
  // Verify configurations were applied succesfuly
    boolean speedConfig = speed.getConfigurator().apply(new TalonFXConfiguration()) == StatusCode.OK;
        boolean angleConfig = angle.getConfigurator().apply(new TalonFXConfiguration()) == StatusCode.OK;
        
        if (!speedConfig || !angleConfig) {
            throw new RuntimeException("Failed to configure motors");

   
   // new methods to be added later but this should be enough to initilize
          // would add current limits but they are commented out here

  }}



     catch (Exception e) {
        DriverStation.reportError("Failed to initialize " + MODULE_NAME + ": " + e.getMessage(), e.getStackTrace());}

    // get any errors for intilitialize here instead of manisfesting as weird behavior at runtime hopefully



  }
// Gets module state
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        speed.getVelocity().getValue(),
        Rotation2d.fromRotations(encoder.getAbsolutePosition().getValue())
    );
}
// status monituring
public boolean isModuleInitialized() {
  return BaseStatusSignal.refreshAll(
      speed.getDeviceTemp(),
      angle.getDeviceTemp(),
      encoder.getAbsolutePosition()
  ).isOK();
}
}

    

