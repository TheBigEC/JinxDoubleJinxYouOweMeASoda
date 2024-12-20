package frc.robot.ESPN;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;


import com.ctre.phoenix6.hardware.TalonFX;
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
