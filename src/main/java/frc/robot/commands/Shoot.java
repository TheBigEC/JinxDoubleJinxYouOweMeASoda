package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.RockPaperScissors;

public class Shoot extends Command {


  

    RockPaperScissors shooter = RockPaperScissors.getInstance();


      public Shoot(){
        addRequirements(shooter);
      }


    


    public void execute(){
        shooter.run(0.25);

    }

    public void end(boolean interrupted){
        shooter.run(0.25);

    }

    public boolean isFinished(){
        return false;
    }

    
}
