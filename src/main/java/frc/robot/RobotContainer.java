package frc.robot;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.UpdateManager;
import org.frcteam2910.common.robot.input.XboxController;
//import org.frcteam2910.common.robot.input.XboxController;
import org.frcteam2910.common.util.HolonomicDriveSignal;

import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.C_Drive;
import frc.robot.subsystems.SS_Drivetrain;

public class RobotContainer {
    public static XboxController controller = new XboxController(Constants.XBOX_CONTROLLER);

    private SS_Drivetrain ss_drive = SS_Drivetrain.getInstance();

    private final UpdateManager updateManager = new UpdateManager(ss_drive);


    public RobotContainer(){
       // ss_drive.resetGyroAngle(Rotation2.fromDegrees(180));
        CommandScheduler.getInstance().setDefaultCommand(ss_drive, new C_Drive());
        updateManager.startLoop(5.0e-3);
    
    } 

    public static XboxController getDriveController(){
        return controller;
    }
    // make an autonomous command to get to a random point on the field.
    // go forward and turn so that when u run drivers station it will run 
    // write a command to take a point and get to there like C_AutoDrive

     //public SequentialCommandGroup getAutonomousCommand() {
        /*
        return new SequentialCommandGroup(
            new InstantCommand(() -> ss_drive.drive(new HolonomicDriveSignal(new Vector2(0, 1), 90.0, true)))
            new C_AutoDrive(new Vector2(-36.0, 0.0), 1.0, Math.toRadians(180), 1.0)
        );
            ); 
            */
            
     //}

     public void configButtonBindings(){
         controller.getBackButton().whenPressed(() -> ss_drive.resetGyroAngle(Rotation2.ZERO), ss_drive); 
     }

}
