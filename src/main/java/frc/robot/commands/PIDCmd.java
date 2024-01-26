package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PIDSub;

// What this COmmand does is using PID all needed is a Setpoint in the robot container and will go there 

public class PIDCmd extends Command{

    private final PIDController pidController; 
   

    public PIDCmd(PIDSub pidSub, double setpoint){
        // THis PID controller will need to be tuned for each different encoder that will be used 
        this.pidController = new PIDController(0.09,0.03,0.002);
        pidController.setSetpoint(setpoint);
        addRequirements(pidSub);
    }

   

    @Override
    public void initialize(){
        pidController.reset();

    }
    @Override 
    public void execute(){
        double speed = pidController.calculate(PIDSub.getEncoderFeet());
        PIDSub.setMotor(speed);
   
    }
    @Override 
    public void end(boolean interrupted){
        PIDSub.setMotor(0);
    
    }
    @Override
    public boolean isFinished(){
            return false;
    }

   
    
}