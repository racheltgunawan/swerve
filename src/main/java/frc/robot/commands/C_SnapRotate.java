// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.util.HolonomicDriveSignal;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SS_Drivetrain;

public class C_SnapRotate extends CommandBase {
  /** Creates a new C_SnapRotate. */
  private SS_Drivetrain ss_drive = SS_Drivetrain.getInstance();
  private double toAngle;
  private double currentAngle;

  public C_SnapRotate(int toAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ss_drive);
    this.toAngle = Math.toRadians(toAngle * 45);
    this.currentAngle = ss_drive.getPose().rotation.toRadians();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    HolonomicDriveSignal drivesignal = new HolonomicDriveSignal(new Vector2(0, 0), -0.5, true); 
    ss_drive.drive(drivesignal);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(currentAngle == toAngle){
      return true;
    }else{
      return false;
    }
  }
}
