package frc.robot.ESPN;


import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;



// I tried to change the name and rename the file but it messed up
// classes whole purpose is to just change the name of the method to set brake lol
public class E_Motor extends TalonFX {

    public E_Motor(int deviceid, String canBus) {
        super(deviceid, canBus);

        
    } 


    public void setBrake( boolean braked){
        super.setNeutralMode(braked ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }
    

   
    
}
