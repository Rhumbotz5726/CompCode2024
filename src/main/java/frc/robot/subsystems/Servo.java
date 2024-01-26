package frc.robot.subsystems;



import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Servo extends SubsystemBase {
  private final edu.wpi.first.wpilibj.Servo servo = new edu.wpi.first.wpilibj.Servo(1); 
//  private final edu.wpi.first.wpilibj.Servo servo2 = new edu.wpi.first.wpilibj.Servo(0); 


 public Servo(){

 }

@Override
public void periodic(){
    
}
public void setServo(double angle){
    servo.setAngle(angle);
    //servo2.setAngle(angle);

}



}
