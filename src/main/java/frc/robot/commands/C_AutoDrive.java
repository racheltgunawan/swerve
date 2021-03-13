// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.PidController;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.util.HolonomicDriveSignal;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SS_Drivetrain;

public class C_AutoDrive extends CommandBase {
  /** Creates a new C_AutoDrive. */

  private SS_Drivetrain ss_drive = new SS_Drivetrain();
  private Vector2 translation;
  private double rotate;

  private RigidTransform2 currentPose;
  private double currentAngle;

  private double lastTimeStamp;

  private PidController translationController = new PidController(new PidConstants(0, 0, 0));
  private PidController rotationController = new PidController(new PidConstants(0, 0, 0));

  private double translationSpeed;
  private double rotationSpeed;

  private double translationPercentTolerance = .025;
  private double rotationPercentTolerance = .01;
  
  public C_AutoDrive(Vector2 translation, double rotate, double translationPercentOutput, double rotationPercentOutput) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.translation = translation;
    this.rotate = rotate;
    addRequirements(ss_drive);

    translationController.setSetpoint(translation.length);
    translationController.setOutputRange(-translationPercentOutput, translationPercentOutput);

    rotationController.setSetpoint(rotate);
    rotationController.setInputRange(0, 2 * Math.PI);
    rotationController.setContinuous(true);
    rotationController.setOutputRange(-rotationPercentOutput, rotationPercentOutput);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ss_drive.resetPoseTranslation();
  } 

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentTime = Timer.getFPGATimestamp();
    double dt = currentTime - lastTimeStamp;
    lastTimeStamp = currentTime;

    currentPose = ss_drive.getPose();
    currentAngle = currentPose.rotation.toRadians();

    //translationSpeed = translationController.calculate(Math.hypot(currentPose.translation.x, currentPose.translation.y), dt);
    translationSpeed = translationController.calculate(currentPose.translation.length, dt);

    Vector2 translationVector = Vector2.fromAngle(translation.getAngle()).normal().scale(translationSpeed);

    rotationSpeed = rotationController.calculate(currentAngle, dt);

    ss_drive.drive(new HolonomicDriveSignal(translationVector, rotationSpeed, true));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ss_drive.drive(new HolonomicDriveSignal(new Vector2(0, 0), 0, true));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return  (Math.abs(rotate - currentAngle) <= 2 * Math.PI * rotationPercentTolerance || rotate == 0.0) &&
            (Math.abs(translation.y - currentPose.translation.y) <= translation.y * translationPercentTolerance || translation.y == 0.0) &&
            (Math.abs(translation.x - currentPose.translation.x) <= translation.x * translationPercentTolerance || translation.x == 0.0) ||
            (translationSpeed <= .001 && rotationSpeed <= .0005);
  }
}
