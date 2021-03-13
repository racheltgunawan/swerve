// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.input.XboxController;
//import org.frcteam2910.common.robot.input.XboxController;
import org.frcteam2910.common.util.HolonomicDriveSignal;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SS_Drivetrain;

public class C_Drive extends CommandBase {
  /** Creates a new C_Drive. */
  private SS_Drivetrain ss_drive = SS_Drivetrain.getInstance();
  private XboxController controller = (XboxController) RobotContainer.getDriveController();

  public C_Drive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ss_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double forward = deadband(controller.getLeftYAxis().get(true)); 
    double strafe = -deadband(controller.getLeftXAxis().get(true));
    double rotation = -deadband(controller.getRightXAxis().get(true)) * .05; //TODO: make deadband constant
    
    HolonomicDriveSignal drivesignal = new HolonomicDriveSignal(new Vector2(forward, strafe), rotation, true); //TODO: make variable for field oriented
    ss_drive.drive(drivesignal);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private double deadband(double input) {
    if(Math.abs(input) < 0.05) {
      return 0;
    } 
    return input;
  }
}
