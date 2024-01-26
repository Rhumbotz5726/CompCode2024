package frc.robot.subsystems;

//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRollerSub extends SubsystemBase {
    // Comment back in when ready to run with intake otherwise the code will not run 
   // private final CANSparkMax intakeRoller = new CANSparkMax(13, MotorType.kBrushed); 


 public IntakeRollerSub(){

 }

@Override
public void periodic(){
    
}

public void setMotor(double speed){
//intakeRoller.set(speed);
    
}

}
