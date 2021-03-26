// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.PidController;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.input.XboxController;
import org.frcteam2910.common.util.HolonomicDriveSignal;

import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SS_Drivetrain;

public class C_SnapRotate extends CommandBase {
  /** Creates a new C_SnapRotate. */
  private SS_Drivetrain ss_drive = SS_Drivetrain.getInstance();
  private XboxController controller = (XboxController) RobotContainer.getDriveController();
  private double toAngle;
 private double currentAngle;

  private double rotationSpeed;

  private double lastTimeStamp;
  
  private PidController rotationController = new PidController(new PidConstants(0.0475, 0, 0));
  // rotate = -0.5
  public C_SnapRotate(int toAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ss_drive);
    this.toAngle = Math.toRadians(toAngle);

    rotationController.setSetpoint(this.toAngle);
    rotationController.setInputRange(0, 2 * Math.PI);
    rotationController.setContinuous(true);
    rotationController.setOutputRange(-0.15, 0.15);

    //negative = clockwise positive = counter clockwise
    //    double initialAngle = ss_drive.getPose().rotation.toRadians();
    // if(Math.abs(initialAngle - toAngle) > Math.PI){
    //   rotate = 0.5;
    // }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     double currentTime = Timer.getFPGATimestamp();
     double dt = currentTime - lastTimeStamp;
     lastTimeStamp = currentTime;

    this.currentAngle = ss_drive.getPose().rotation.toRadians();
    double forward = deadband(controller.getLeftYAxis().get(true));
    double strafe = -deadband(controller.getLeftXAxis().get(true));
    rotationSpeed = rotationController.calculate(currentAngle, dt);

    // if(Math.abs(toAngle - currentAngle) < 360 - Math.abs(toAngle - currentAngle)){
    //   rotationSpeed = 0.01;
    // }

    // if(Math.abs(ss_drive.getPose().rotation.toRadians() - Math.toRadians(toAngle * 45)) > 360 - Math.abs(ss_drive.getPose().rotation.toRadians() - Math.toRadians(toAngle * 45))){
    //   rotationSpeed = 0.01;
    // }else{
    //   rotationSpeed = -0.01;
    // }

    HolonomicDriveSignal drivesignal = new HolonomicDriveSignal(new Vector2(forward, strafe), -rotationSpeed, true); 
    ss_drive.drive(drivesignal); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*
    if(currentAngle == toAngle){
      return true;
    }
    return false;*/
    // if((Math.abs(toAngle - currentAngle) <= 2 * Math.PI * .01 || currentAngle == toAngle)){
    //       return true;
    // }else{
    //   return false;
    // }

    // if(Math.abs(ss_drive.getPose().rotation.toRadians() - Math.toRadians(toAngle * 45)) <= 2 * Math.PI * .01){
    //   return true;
    // }else{
    //   return false;
    // }
    return false;
  }

  private double deadband(double input) {
    if(Math.abs(input) < 0.05) {
      return 0;
    } 
    return input;
  }
}
