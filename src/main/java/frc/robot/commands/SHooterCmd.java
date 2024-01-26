package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class SHooterCmd extends Command{
    private final frc.robot.subsystems.Shooter shooter;
    private final double speed; 
    //Trying to use a timer to run auto 
    private final Timer timer = new Timer();

    private final double time; 

    public SHooterCmd(frc.robot.subsystems.Shooter shooter, double shootingTime, double speed){
        this.shooter = shooter;
        this.speed = speed;

        time = shootingTime;
        addRequirements(shooter);
    }
    private void addRequirements(frc.robot.subsystems.Shooter shooter){

    }
    @Override 
    public void initialize(){
       // System.out.println("NeotryCmd started! ");
       timer.reset();
       timer.start();
    }
  
    @Override 
    public void execute(){
        shooter.setMotor(speed);
       // shooter.setMotor2(speed);
      
    }
  
    @Override 
    public void end(boolean interrupted){
        shooter.setMotor(0);
        timer.reset();

      
  
    }
  
    @Override 
    public boolean isFinished(){
       return  timer.get() >= time;
        //return false;
    }

 


}