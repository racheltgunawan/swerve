// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.PidController;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.input.XboxController;
import org.frcteam2910.common.robot.input.DPadButton.Direction;
//import org.frcteam2910.common.robot.input.DPadButton.Direction;
//import org.frcteam2910.common.robot.input.XboxController;
import org.frcteam2910.common.util.HolonomicDriveSignal;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SS_Drivetrain;

public class C_Drive extends CommandBase {
  /** Creates a new C_Drive. */
  private SS_Drivetrain ss_drive = SS_Drivetrain.getInstance();
  private XboxController controller = (XboxController) RobotContainer.getDriveController();

  private PidController rotationController = new PidController(new PidConstants(0.3, 0, 0));

  public C_Drive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ss_drive);
    rotationController.setInputRange(0, 2 * Math.PI);
    rotationController.setContinuous(true);
    rotationController.setOutputRange(-1.0, 1.0);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  // boolean snapUp = false;
  // boolean snapUpleft = false;
  // boolean snapleft = false;
  // boolean snapdownleft = false;
  // boolean snapdown = false;
  // boolean snapdownright = false;
  // boolean snapright = false;
  // boolean snapupright = false;

  double toAngle = 0;
  boolean snap = false;
  
 private double lastTimeStamp;
  @Override
  public void execute() {
    //  double currentTime = Timer.getFPGATimestamp();
    //  double dt = currentTime - lastTimeStamp;
    //  lastTimeStamp = currentTime;

    
    double forward = deadband(controller.getLeftYAxis().get(true)); 
    double strafe = -deadband(controller.getLeftXAxis().get(true));
    double rotation = -deadband(controller.getRightXAxis().get(true)) * .05; //TODO: make deadband constant
    //rotationSpeed = rotationController.calculate(currentAngle, dt) * 0.01;

    //double rotation;
    
    // if(controller.getDPadButton(Direction.UP).get()){
    //   toAngle = 0;
    //   snap = true;
    // }else if(controller.getDPadButton(Direction.UPRIGHT).get()){
    //   toAngle = 1; 
    //   snap = true;
    // }else if(controller.getDPadButton(Direction.RIGHT).get()){
    //   toAngle = 2; 
    //   snap = true;
    // }else if(controller.getDPadButton(Direction.DOWNRIGHT).get()){
    //   toAngle = 3; 
    //   snap = true;
    // }else if(controller.getDPadButton(Direction.DOWN).get()){
    //   toAngle = 4; 
    //   snap = true;
    // }else if(controller.getDPadButton(Direction.DOWNLEFT).get()){
    //   toAngle = 5; 
    //   snap = true;
    // }else if(controller.getDPadButton(Direction.LEFT).get()){
    //   toAngle = 6; 
    //   snap = true;
    // }else if(controller.getDPadButton(Direction.UPLEFT).get()){
    //   toAngle = 7; 
    //   snap = true;
    // }
    
    //  Direction[] direction = {Direction.UP, Direction.UPRIGHT, Direction.RIGHT, Direction.DOWNRIGHT, Direction.DOWN, Direction.DOWNLEFT, Direction.LEFT, Direction.UPLEFT};
    //  for(int i = 0; i < direction.length; i++){
    //   if(controller.getDPadButton(direction[i]).get()){
    //     toAngle = Math.toRadians(i * 45);
    //     snap = true;
    //   }
    // }

   // if(snap && Math.abs(ss_drive.getPose().rotation.toRadians() - Math.toRadians(toAngle * 45)) > 2 * Math.PI * .01){
    // if(snap){
    // if(Math.abs(ss_drive.getPose().rotation.toRadians() - Math.toRadians(toAngle * 45)) > 360 - Math.abs(ss_drive.getPose().rotation.toRadians() - Math.toRadians(toAngle * 45))){
      //   rotation = 0.1;
      // }else{
      //   rotation = -0.1;
      // }
    //   rotationController.setSetpoint(this.toAngle);
    //   rotation = rotationController.calculate(ss_drive.getPose().rotation.toRadians(), dt) * 0.15;
    // }else{
    //   snap = false;
    //   rotation = -deadband(controller.getRightXAxis().get(true)) * .05;
    // }

    //double rotation;
    //   if(controller.getDPadButton(Direction.UP).get() && ss_drive.getPose().rotation.toRadians() != Math.toRadians(0)) {
    //     rotation = 0.3;
    //   } else if(controller.getDPadButton(Direction.UPRIGHT).get() && ss_drive.getPose().rotation.toRadians() != Math.toRadians(45)){
    //    rotation = 0.3;
    //  } else if(controller.getDPadButton(Direction.RIGHT).get() && ss_drive.getPose().rotation.toRadians() != Math.toRadians(90)){
    //    rotation = 0.3;
    //  }else if(controller.getDPadButton(Direction.DOWNRIGHT).get() && ss_drive.getPose().rotation.toRadians() != Math.toRadians(135)){
    //    rotation = 0.3;
    //  }else if(controller.getDPadButton(Direction.DOWN).get() && ss_drive.getPose().rotation.toRadians() != Math.toRadians(180)){
    //    rotation = 0.3;
    //  }else if(controller.getDPadButton(Direction.DOWNLEFT).get() && ss_drive.getPose().rotation.toRadians() != Math.toRadians(225)){
    //    rotation = 0.3;
    //  }else if(controller.getDPadButton(Direction.LEFT).get() && ss_drive.getPose().rotation.toRadians() != Math.toRadians(270)){
    //    rotation = 0.3;
    //  }else if(controller.getDPadButton(Direction.UPLEFT).get() && ss_drive.getPose().rotation.toRadians() != Math.toRadians(315)){
    //    rotation = 0.3;
    //  }else{
    //    rotation = -deadband(controller.getRightXAxis().get(true)) * .05;
    //  }
  //   if((controller.getDPadButton(Direction.UP).get() && ss_drive.getPose().rotation.toRadians() != Math.toRadians(0)) || 
  //     (controller.getDPadButton(Direction.UPRIGHT).get() && ss_drive.getPose().rotation.toRadians() != Math.toRadians(45)) || 
  //     (controller.getDPadButton(Direction.RIGHT).get() && ss_drive.getPose().rotation.toRadians() != Math.toRadians(90)) || 
  //     (controller.getDPadButton(Direction.DOWNRIGHT).get() && ss_drive.getPose().rotation.toRadians() != Math.toRadians(135)) || 
  //     (controller.getDPadButton(Direction.DOWN).get() && ss_drive.getPose().rotation.toRadians() != Math.toRadians(180)) || 
  //     (controller.getDPadButton(Direction.DOWNLEFT).get() && ss_drive.getPose().rotation.toRadians() != Math.toRadians(225)) || 
  //     (controller.getDPadButton(Direction.LEFT).get() && ss_drive.getPose().rotation.toRadians() != Math.toRadians(270)) || 
  //     (controller.getDPadButton(Direction.UPLEFT).get() && ss_drive.getPose().rotation.toRadians() != Math.toRadians(315))){
  //     rotation = 0.1;
  //   }else{
  //    rotation = -deadband(controller.getRightXAxis().get(true)) * .05;
  //  }

  // if(controller.getDPadButton(Direction.UP).get()){
  //   snapUp = true;
  //   rotation = .05;
  // }else{
  //   if(snapUp == true && Math.abs(ss_drive.getPose().rotation.toRadians() - Math.toRadians(0)) > 2 * Math.PI * .01){
  //     snapUp = true;
  //     rotation = .1;
  //   }else{
  //     snapUp = false;
  //     rotation = -deadband(controller.getRightXAxis().get(true));
  //   }
  // }

  // if(controller.getDPadButton(Direction.UPRIGHT).get()){
  //   snapupright = true;
  //   rotation = .05;
  // }else{
  //   if(snapupright == true && Math.abs(ss_drive.getPose().rotation.toRadians() - Math.toRadians(45)) > 2 * Math.PI * .01){
  //     snapupright = true;
  //     rotation = .1;
  //   }else{
  //     snapupright = false;
  //     rotation = -deadband(controller.getRightXAxis().get(true));
  //   }
  // }

  // if(controller.getDPadButton(Direction.LEFT).get()){
  //   snapleft = true;
  //   rotation = .05;
  // }else{
  //   if(snapleft == true && Math.abs(ss_drive.getPose().rotation.toRadians() - Math.toRadians(90)) > 2 * Math.PI * .01){
  //     snapleft = true;
  //     rotation = .1;
  //   }else{
  //     snapleft = false;
  //     rotation = -deadband(controller.getRightXAxis().get(true));
  //   }
  // }
  
  // if(controller.getDPadButton(Direction.DOWNLEFT).get()){
  //   snapdownleft = true;
  //   rotation = .05;
  // }else{
  //   if(snapdownleft == true && Math.abs(ss_drive.getPose().rotation.toRadians() - Math.toRadians(135)) > 2 * Math.PI * .01){
  //     snapdownleft = true;
  //     rotation = .1;
  //   }else{
  //     snapdownleft = false;
  //     rotation = -deadband(controller.getRightXAxis().get(true));
  //   }
  // }

  // if(controller.getDPadButton(Direction.DOWN).get()){
  //   snapdownleft = true;
  //   rotation = .05;
  // }else{
  //   if(snapdown == true && Math.abs(ss_drive.getPose().rotation.toRadians() - Math.toRadians(180)) > 2 * Math.PI * .01){
  //     snapdown = true;
  //     rotation = .1;
  //   }else{
  //     snapdown = false;
  //     rotation = -deadband(controller.getRightXAxis().get(true));
  //   }
  // }

  // if(controller.getDPadButton(Direction.DOWNRIGHT).get()){
  //   snapdownright = true;
  //   rotation = .05;
  // }else{
  //   if(snapdownright == true && Math.abs(ss_drive.getPose().rotation.toRadians() - Math.toRadians(225)) > 2 * Math.PI * .01){
  //     snapdownright = true;
  //     rotation = .1;
  //   }else{
  //     snapdownright = false;
  //     rotation = -deadband(controller.getRightXAxis().get(true));
  //   }
  // }

  // if(controller.getDPadButton(Direction.RIGHT).get()){
  //   snapright = true;
  //   rotation = .05;
  // }else{
  //   if(snapright == true && Math.abs(ss_drive.getPose().rotation.toRadians() - Math.toRadians(270)) > 2 * Math.PI * .01){
  //     snapright = true;
  //     rotation = .1;
  //   }else{
  //     snapright = false;
  //     rotation = -deadband(controller.getRightXAxis().get(true));
  //   }
  // }

  // if(controller.getDPadButton(Direction.UPRIGHT).get()){
  //   snapupright = true;
  //   rotation = .05;
  // }else{
  //   if(snapupright == true && Math.abs(ss_drive.getPose().rotation.toRadians() - Math.toRadians(315)) > 2 * Math.PI * .01){
  //     snapupright = true;
  //     rotation = .1;
  //   }else{
  //     snapupright = false;
  //     rotation = -deadband(controller.getRightXAxis().get(true));
  //   }
  // }
  
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
