package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterCmd2 extends Command{
    private final frc.robot.subsystems.Shooter2 shooter2;
    private final double speed; 

    public ShooterCmd2(frc.robot.subsystems.Shooter2 shooter2, double speed){
        this.shooter2 = shooter2;
        this.speed = speed;
        addRequirements(shooter2);
    }
    private void addRequirements(frc.robot.subsystems.Shooter2 shooter2){

    }
    @Override 
    public void initialize(){
        //System.out.println("NeotryCmd started! ");
    }
  
    @Override 
    public void execute(){
        shooter2.setMotor(speed);
       // shooter.setMotor2(speed);
      
    }
  
    @Override 
    public void end(boolean interrupted){
        shooter2.setMotor(0);
      
  
    }
  
    @Override 
    public boolean isFinished(){
        return false;
    }


}