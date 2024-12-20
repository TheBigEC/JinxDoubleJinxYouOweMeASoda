// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.ESPN.S_ProperController;
import frc.robot.ESPN.S_ProperController.Scale;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.drivetrain.RockPaperScissors;
import frc.robot.subsystems.drivetrain.TunerConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  public static S_ProperController pilot = new S_ProperController(Constants.controller.PILOT_PORT);
  public static S_ProperController copilot = new S_ProperController(Constants.controller.COPILOT_PORT);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }
private void SetDefaultCommands(){
  drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
  drivetrain.brake()
);
}


  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(new Shoot());

    drivingInput.onTrue(RockPaperScissors.drive(-pilot.getCorrectedLeft().getX() * 1.0, // Drive forward with
                                                                                           // negative Y (forward)
            -pilot.getCorrectedLeft().getY() * 1.0, // Drive left with negative X (left)
            pilot.getRightX(Scale.SQUARED) * 0.7 // Drive counterclockwise with negative X (left)
        ));}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
