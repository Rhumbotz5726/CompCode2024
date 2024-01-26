package frc.robot.subsystems;



import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ServoL extends SubsystemBase {
  private final edu.wpi.first.wpilibj.Servo servo2 = new edu.wpi.first.wpilibj.Servo(0); 


 public ServoL(){

 }

@Override
public void periodic(){
    
}
public void setServo(double angle){
    servo2.setAngle(angle);

}



}
