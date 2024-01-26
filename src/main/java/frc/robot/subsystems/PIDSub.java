package frc.robot.subsystems;



//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


//import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PIDSub extends SubsystemBase {

// Here declare the motor type that will be being used and then the Can ID 


//private final static CANSparkMax elevatorMotor = new CANSparkMax(15,MotorType.kBrushless);

// THis is where the encoder will be plugged into and declared 
 private static DutyCycleEncoder encoder = new DutyCycleEncoder(0);
 public static final double tick2Feet = 1.0 / 4096 * 6  * Math.PI / 12;

 public PIDSub(){

 }

@Override
public void periodic(){
    SmartDashboard.putNumber("Encoder Position", getEncoderFeet());
    
}

public static void setMotor(double speed){
  //  elevatorMotor.set(speed);
    
}

public static double getEncoderFeet(){
    return (encoder.getDistance() );//*  tick2Feet);
}

}