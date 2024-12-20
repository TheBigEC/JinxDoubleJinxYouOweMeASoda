// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.ESPN.S_ProperController;
import frc.robot.ESPN.S_ProperController.Scale;
import frc.robot.subsystems.drivetrain.RockPaperScissors;
import frc.robot.subsystems.drivetrain.TunerConstants;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  Trigger drivingInput = new Trigger(() -> (pilot.getCorrectedLeft().getNorm() != 0.0 || pilot.getCorrectedRight().getX() != 0.0));
  private final RockPaperScissors drivetrain = TunerConstants.DriveTrain;
  // The robot's subsystems and commands are defined here...
  private boolean useFieldCentric = true;  // Track which drive mode we're in

  public static S_ProperController pilot = new S_ProperController(Constants.controller.PILOT_PORT);
  public static S_ProperController copilot = new S_ProperController(Constants.controller.COPILOT_PORT);
  // Replace with CommandPS4Controller or CommandJoystick if needed


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    SetDefaultCommands();
  }
private void SetDefaultCommands(){
  drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
  drivetrain.brake()
);
}


  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

       pilot.b().whileTrue(drivetrain.defenseMode());
    

        // default
   
        pilot.getStartButton().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

      
      

      pilot.x().onTrue(Commands.runOnce(() -> useFieldCentric = !useFieldCentric));

      // Use the appropriate drive mode based on the toggle
      drivingInput.onTrue(Commands.either(
        // Field-centric mode branch
        Commands.either(
            // Normal speed field-centric
            drivetrain.drive(
                -pilot.getCorrectedLeft().getX(),
                -pilot.getCorrectedLeft().getY(),
                pilot.getRightX(Scale.SQUARED) * 0.7
            ),
            // Slow speed field-centric
            drivetrain.drive(
                -pilot.getCorrectedLeft().getX() * 0.3,  // 30% speed for precise control
                -pilot.getCorrectedLeft().getY() * 0.3,
                pilot.getRightX(Scale.SQUARED) * 0.7 * 0.3
            ),
            () -> !pilot.leftBumper().getAsBoolean()  // Left bumper toggles slow mode
        ),
        // Robot-centric mode branch
        Commands.either(
            // Normal speed robot-centric
            drivetrain.robotCentricDrive(
                -pilot.getCorrectedLeft().getX(),
                -pilot.getCorrectedLeft().getY(),
                pilot.getRightX(Scale.SQUARED) * 0.7
            ),
            // Slow speed robot-centric
            drivetrain.robotCentricDrive(
                -pilot.getCorrectedLeft().getX() * 0.3,  // 30% speed for precise control
                -pilot.getCorrectedLeft().getY() * 0.3,
                pilot.getRightX(Scale.SQUARED) * 0.7 * 0.3
            ),
            () -> !pilot.leftBumper().getAsBoolean()  // Left bumper toggles slow mode
        ),
        () -> useFieldCentric  // X button toggles this boolean
    ));}



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
