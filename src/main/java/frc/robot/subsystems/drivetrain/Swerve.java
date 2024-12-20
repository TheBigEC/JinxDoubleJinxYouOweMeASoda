package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {

    private static Swerve instance = null;

    public PIDController rotPID = Constants.drivetrain.ROT_PID;
    public PIDController correctionPID = Constants.drivetrain.CORRECTION_PID;


    public static synchronized Swerve getInstance() {
        
        if (instance == null) instance = new Swerve();
        
        return instance;


    }
    // reference code

    private SwerveModule[] moduleList = {
        new SwerveModule(
        "Front Lest",
        Constants.drivetrain.FL_LOCATION,
        Constants.drivetrain.DRIVE_GEARING,
        Constants.ids.FL_SPEED,
        Constants.ids.FL_ANGLE,
        Constants.ids.FL_ENCODER,
        Constants.drivetrain.FL_ZERO
        ),

        new SwerveModule(
            "Front Right",
        Constants.drivetrain.FR_LOCATION,
        Constants.drivetrain.DRIVE_GEARING,
        Constants.ids.FR_SPEED,
        Constants.ids.FR_ANGLE,
        Constants.ids.FR_ENCODER,
        Constants.drivetrain.FR_ZERO),

        new SwerveModule(
        "Back Left",
        Constants.drivetrain.BL_LOCATION,
        Constants.drivetrain.DRIVE_GEARING,
        Constants.ids.BL_SPEED,
        Constants.ids.BL_ANGLE,
        Constants.ids.BL_ENCODER,
        Constants.drivetrain.BL_ZERO
        ),

        new SwerveModule(
            "Back Right",
            Constants.drivetrain.BR_LOCATION,
            Constants.drivetrain.DRIVE_GEARING,
            Constants.ids.BR_SPEED,
            Constants.ids.BL_ANGLE,
            Constants.ids.BL_ENCODER,
            Constants.drivetrain.BR_ZERO
            
            )




    };
    // long variable declaration that creates swerve modules with the arugments of constants in a list
    // Allows for multiple operations to be preformed on all modules





    }
    

