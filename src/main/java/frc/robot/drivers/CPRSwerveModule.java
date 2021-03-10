// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.PidController;
import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.Constants;

/** Add your docs here. */
public class CPRSwerveModule extends SwerveModule {

    private TalonFX angleMotor;
    private TalonFX driveMotor;

   // private final PIDController anglePIDController;
    //private PIDController drivePIDController;

    private PidController anglePIDController;
    private PidController drivePIDController;

    private TalonFXSensorCollection angleEncoder;
    private double angleEncoderPosition;
    private TalonFXSensorCollection driveEncoder;

    private double offsetAngle;

    private ControlMode driveControlMode;
    //private TalonFXPIDSetConfiguration drivePIDController;

    private double wheelRevolutionsPerUnit = 0.076; // 2048 units per rotation
    private double ticksPerRevolution = 2048;
    private double DRIVE_GEAR_RATIO = 8.16;
    private double ANGLE_GEAR_RATIO = 12.8;
    
    private CANCoder angleAbsoluteEncoder;

    public CPRSwerveModule(Vector2 modulePosition, double offsetAngle, TalonFX angleMotor, TalonFX driveMotor, CANCoder canEncoder) {
        super(modulePosition);
        this.angleMotor = angleMotor;
        this.driveMotor = driveMotor;
        this.offsetAngle = offsetAngle;

        // angle motor encoder
        this.angleEncoder = angleMotor.getSensorCollection();
        this.angleEncoderPosition = angleEncoder.getIntegratedSensorPosition();
        angleAbsoluteEncoder = canEncoder;

        angleEncoder.setIntegratedSensorPosition(readAngle(), 10); // not quite sure lol


        // PID controllers
        //anglePIDController = new PIDController(1, 1, 1);
        //drivePIDController = new PIDController(1, 1, 1);
        drivePIDController = new PidController(new PidConstants(0.75, 0, 0));
        drivePIDController.setOutputRange(-1.0, 1.0);

        anglePIDController = new PidController(new PidConstants(0.5, 0, 0));
        anglePIDController.setContinuous(true);
        anglePIDController.setInputRange(0, 2 * Math.PI);
        anglePIDController.setOutputRange(-1.0, 1.0);

        //drivePIDController = new TalonFXPIDSetConfiguration();
        //driveMotor.getPIDConfigs(drivePIDController);

        // drive motor control mode??? 
        driveControlMode = driveMotor.getControlMode();
        driveEncoder = driveMotor.getSensorCollection();

    }

    @Override
    public double readAngle() {
       // double angle = (angleEncoderPosition / 2048) * 2.0 * Math.PI + offsetAngle; 
        //double angle = angleAbsoluteEncoder.getPosition() / 180 * Math.PI + offsetAngle; no
       //double angle = angleAbsoluteEncoder.getPosition() * 2.0 * Math.PI + offsetAngle; 
       double angle = Math.toRadians(angleAbsoluteEncoder.getAbsolutePosition()) + offsetAngle; 
        angle %= 2.0 * Math.PI;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }
        return angle;
    }
    
    @Override
    protected double readDistance() {
        return driveEncoder.getIntegratedSensorPosition() * wheelRevolutionsPerUnit * ticksPerRevolution * DRIVE_GEAR_RATIO;
    }
    
    double lastDrivePidTimestamp;
    @Override
    protected void setDriveOutput(double output) {
        double timestamp = Timer.getFPGATimestamp();
        double dt = timestamp - lastDrivePidTimestamp;
        lastDrivePidTimestamp = timestamp;
        drivePIDController.setSetpoint(output);
        driveMotor.set(TalonFXControlMode.PercentOutput, drivePIDController.calculate(driveMotor.getMotorOutputPercent(), dt));
    }

    double lastAnglePidTimestamp;
    @Override
    protected void setTargetAngle(double angle) {
        double timestamp = Timer.getFPGATimestamp();
        double dt = timestamp - lastAnglePidTimestamp;
        lastAnglePidTimestamp = timestamp;
        anglePIDController.setSetpoint(angle);
        angleMotor.set(TalonFXControlMode.PercentOutput, anglePIDController.calculate(readAngle(), dt));
    } 

    public double getCurrentVelocity() {
        return driveEncoder.getIntegratedSensorVelocity() * 10.0 / 2048.0 / DRIVE_GEAR_RATIO / wheelRevolutionsPerUnit;
        //return driveEncoder.getIntegratedSensorVelocity() / wheelRevolutionsPerUnit;
    }

    public double getIntegratedEncoderAngle() {
        return Math.toDegrees(angleEncoderPosition);
    }

    public void resetIntegratedEncoder() {
        angleEncoder.setIntegratedSensorPosition(readAngle(), 10);
    }

    public double getEncoderPosition() {
        return angleAbsoluteEncoder.getPosition();
    }

    public void setDriveTicksPerUnit(double driveTicksPerUnit) {
        this.wheelRevolutionsPerUnit = driveTicksPerUnit;
    }
}
