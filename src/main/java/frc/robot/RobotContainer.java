// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeOut;
import frc.robot.commands.IntakeRollerCmd;
import frc.robot.commands.PIDCmd;
import frc.robot.commands.PIDteleopCmd;
import frc.robot.commands.SHooterCmd;
import frc.robot.commands.ServoCmd;
import frc.robot.commands.ServoLCmd;
import frc.robot.commands.ShooterCmd2;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakePiston;
import frc.robot.subsystems.IntakeRollerSub;
import frc.robot.subsystems.PIDSub;
import frc.robot.subsystems.Servo;
import frc.robot.subsystems.ServoL;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter2;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.commands.PathPlannerAuto;





import com.pathplanner.lib.path.PathPlannerTrajectory;


public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  // For the shooter 
  private final Shooter shooter = new Shooter();
  private final Shooter2 shooter2 = new Shooter2();
  //Servos for the shooter 
  private final Servo servo = new Servo();
  private final ServoL servo2 = new ServoL();
  // Intake 
  private final IntakePiston intakePiston = new IntakePiston();
  private final IntakeRollerSub roller = new IntakeRollerSub();

  // THis is for the PID 
  private final PIDSub pidSub = new PIDSub();
  
// THis is for the compressor 
  //private final Compressor m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);



  // The driver's controller
  //XboxController m_driverController = new XboxController(0);
  PS4Controller m_driverController = new PS4Controller(3); 
 // XboxController m_driverController2 = new XboxController(0);

  //SendableChooser<Command> autoChooser = new SendableChooser<>();

 // SendableChooser<List<PathPlannerTrajectory>> autoChooser = new SendableChooser<>();
  //SwerveAutoBuilder autoBuilder;
  List<PathPlannerTrajectory> selectedAuto;

  HashMap<String, Command> pathPlannerEventMap = new HashMap<>();
    private final SendableChooser<Command> autoChooser;

    //CHooser for the other auto 
    private final SendableChooser<Command> groupChooser = new SendableChooser<>();

  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);

  

          // THis is the driving command For field orientation hold right bumper or button 6 
              /*   m_robotDrive.setDefaultCommand(new SwerveJoystickCmd(m_robotDrive, 
               () -> -m_driverController.getRawAxis(1),
               () -> -m_driverController.getRawAxis(0),
                () -> -m_driverController.getRightX(), // 4 for the xbox controller 
               () -> m_driverController.getRawButton(6) ));
*/
               m_robotDrive.setDefaultCommand(new SwerveJoystickCmd(m_robotDrive, 
               () -> -m_driverController.getRawAxis(1),
               () -> -m_driverController.getRawAxis(0),
                () -> -m_driverController.getRawAxis(2), // 4 for the xbox controller 
               () -> m_driverController.getRawButton(11) ));
  }

  
  private void configureButtonBindings() {


    new JoystickButton(m_driverController, 5).whileTrue(new RunCommand (() -> m_robotDrive.reset_gyro())); 
// Sets the swerve wheels into an x formation
    new JoystickButton(m_driverController,1)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    new JoystickButton(m_driverController,2).whileTrue(new ServoCmd(servo, 100));
    new JoystickButton(m_driverController,3).whileTrue(new ServoCmd(servo, 0));

    new JoystickButton(m_driverController,2).whileTrue(new ServoLCmd(servo2, 0));
    new JoystickButton(m_driverController, 3).whileTrue(new ServoLCmd(servo2, 100));
    

    

            //  WHen the shooter is being used 
// For the Can ID 10 
// Time is set really high so that it can run trhough the whole match without stopping 
            new JoystickButton(m_driverController,7).whileTrue(new SHooterCmd(shooter,5000000, 1 ));
        new JoystickButton(m_driverController, 8).whileTrue(new SHooterCmd(shooter,5000000, -1)); 

        // FOr the CAN id 11 

        new JoystickButton(m_driverController,7).whileTrue(new ShooterCmd2(shooter2, 1));
        new JoystickButton(m_driverController, 8).whileTrue(new ShooterCmd2(shooter2, -1));
// Intake Roller Motor 
// Set to a lot of seconds so that it can run through the whole match withot stopping 
// The timer is mainly for autonomous 
    new JoystickButton(m_driverController,10).whileTrue(new IntakeRollerCmd(roller, 9000000, 0.5));
    new JoystickButton(m_driverController, 9).whileTrue(new IntakeRollerCmd(roller, 9000000, -0.5));


         /// This is for teh pneumatics try 
         new JoystickButton(m_driverController, 6).toggleOnTrue(new IntakeIn(intakePiston));
         // Release the hatch when the  button is pressed.
         new JoystickButton(m_driverController,  4).toggleOnTrue(new IntakeOut(intakePiston));  

        // THis will be for the Telop and the PID commands for the subsystem that will be needed. 
        // The setpoint is where we want the system to move to 

// This is using the second Controller 
       /*  new JoystickButton(m_driverController2,1).onTrue(new PIDCmd(pidSub, 5));
        new JoystickButton(m_driverController2,2).onTrue(new PIDCmd(pidSub, 2));
        new JoystickButton(m_driverController2,3).whileTrue(new PIDteleopCmd(pidSub, 0.5));
        new JoystickButton(m_driverController2,4).whileTrue(new PIDteleopCmd(pidSub, -0.5));
*/


// This is for the chooser when trying autos to work will need to change return type in the getAutonomous command below 
    SmartDashboard.putData("Example Auto", new PathPlannerAuto("Auto1"));
    SmartDashboard.putData("Another one " , new PathPlannerAuto("NewAuto2"));
    SmartDashboard.putData("Auto3", new PathPlannerAuto("New Auto3"));


//This is the chooser to run the command groups 
// This chooser is for the full autos 
    groupChooser.setDefaultOption("none ", null);
    groupChooser.addOption("ShootPickUp", shootpickUp() );
    groupChooser.addOption("SHoot 2 ", shootBackAgain());
    groupChooser.addOption("PID Test", pid());
    groupChooser.addOption("AutoTry", autoTry());
    SmartDashboard.putData("Auto choices", groupChooser);
  }

 public Command autoTry(){
  return new SequentialCommandGroup(
      new SHooterCmd(shooter, 2, -1),
       new ServoCmd(servo,100),

     new PathPlannerAuto("ScoreBackUP")



/*new SequentialCommandGroup(
     new ParallelCommandGroup(
      new ServoCmd(servo,100),
      new ServoLCmd(servo2, 0),
      new SHooterCmd(shooter, 1, -1)
      )),
      new PathPlannerAuto("ScoreBackUP"),
      new SHooterCmd(shooter, 1, -1)*/


  );

 }
 
  //This auto Shoots the preload then backs up to pick the next piece
public Command shootpickUp(){

   return new SequentialCommandGroup(

   //new PathPlannerAuto("ScoreBackUP"),
    
    // Will spin to wind up
    // SHooter has to be set to a negative value in order for it to shoot up
      new SHooterCmd(shooter, 1,-1),
        new ServoCmd(servo,100),
        new ServoLCmd(servo2,0),
             new ServoCmd(servo,0),
        new ServoLCmd(servo2,100),

  new PathPlannerAuto("ScoreBackUP")

      //Servo Sends the note in
      
     /*new ParallelCommandGroup(
        new ServoCmd(servo,100),
        new ServoLCmd(servo2,0)
        ),*/

 /*new SequentialCommandGroup(
        new ServoCmd(servo,0),
        new ServoLCmd(servo2,100),

  new PathPlannerAuto("ScoreBackUP")*/
 );
   
        //Path planner comes in to back up 
      //  new PathPlannerAuto("ScoreBackUP"),
        // Should have the roller rolling

        // INtake roller things might need to be commented out if not plugged in since
        // COde will not run if the Can ID set to somethign is not found 
       // new IntakeRollerCmd(roller, 2, .5)

        
        
        
    
}
// This auto will Shoot the preload then pick up another one and go back to shoot it again 

public Command shootBackAgain(){
  return new SequentialCommandGroup(
  // Will spin to wind up 
  // Shooter speed has to be negative to shoot forward
      new SHooterCmd(shooter, 1,-1),
      //Servo Sends the note in
      new ParallelRaceGroup(
        new ServoCmd(servo,100),
        new ServoLCmd(servo2,100)
        ),
      new ParallelRaceGroup(
        new ServoCmd(servo,0),
        new ServoLCmd(servo2,0)

      ),

        new PathPlannerAuto("ScoreBackUP"),
        //Here the intake would pick up 

        // then the path planner gets in position again
        new PathPlannerAuto("ScoreAgain"),
        // Motors Wind up again and servo sends it 
        new SHooterCmd(shooter, 1, -1),

        new ParallelRaceGroup(
          new ServoCmd(servo,100),
          new ServoLCmd(servo2,100)
        ),
      new ParallelRaceGroup(
        new ServoCmd(servo,0),
        new ServoLCmd(servo2,0)

      )
        
        );
}
// THis is for testing the PID in autonomous 
public Command pid(){
  return new SequentialCommandGroup(

    new PIDCmd(pidSub, 1)

  );
}


  public Command getAutonomousCommand() {
// THis returns the full command 
    return groupChooser.getSelected();

//TO return just path planner use this 
  // return autoChooser.getSelected(); 

  
 

// Good for testing trajectorys that are programmed with no path planner only trajectory following  

   // Drives in S Pattern for test trial in trajectory following 
/* 
    // Create config for trajectory
     TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    //Trajectory trajectory2 =  TrajectoryGenerator.generateTrajectory()

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    //Run path following command, then stop at the end.

 return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0,false, false));
//return null;*/
  
}

}