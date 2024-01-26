package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    //Spark spark = new Spark(9);
   private final static CANSparkMax neoMotor = new CANSparkMax(11, MotorType.kBrushed);
    private final static CANSparkMax neoMotor2 = new CANSparkMax(10,MotorType.kBrushed); 


 public Shooter(){

 }

@Override
public void periodic(){
    
}

public void setMotor(double speed){
   neoMotor.set(speed);
    neoMotor2.set(speed); 
    
}

}
