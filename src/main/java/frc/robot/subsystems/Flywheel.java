// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;

public class Flywheel extends SubsystemBase implements Loggable {
  private double gearRatio = 1;
  private CANSparkMax neo;
  private TalonSRX cims;
  private int neoCANId = 14;
  private int talonCANId = 5;
  private SparkMaxPIDController pid;
  private boolean running;
  double kP, kI, kD, kF, kMaxOutput, kMinOutput;
  int rpmGoal;

  // ctre sensor units to rpm
  public double sensorUnitsToRPM(double units) {
    return (600 * units * gearRatio) / 2048;
  }

  public  double rpmToSensorUnits(int rpm) {
    double units = (rpm / 600) * (2048 / gearRatio);
    return units;
  }

  /** Creates a new Flywheel. */
  public Flywheel() 
  {
    neo = new CANSparkMax(neoCANId, MotorType.kBrushless);
    pid = neo.getPIDController();

    cims = new TalonSRX(talonCANId);
    
    // default pid gains, adjust in hardware client?
    kP = 0.0001;
    kI = 0;
    kD = 0;
    kF = 0;
    kMinOutput = -1;
    kMaxOutput = 0;
    rpmGoal = 0;

    pid.setP(kP);
    pid.setI(kI);
    pid.setD(kD);
    pid.setFF(kF);
    pid.setOutputRange(kMinOutput, kMaxOutput);

    neo.setIdleMode(IdleMode.kCoast);
    neo.setOpenLoopRampRate(1.5);
    neo.setClosedLoopRampRate(1.5);
  }

  public void UpdatePID(double p, double i, double d, double f, double max, double min)
  {
    if((p != kP)) { pid.setP(p); kP = p; }
    if((i != kI)) { pid.setI(i); kI = i; }
    if((d != kD)) { pid.setD(d); kD = d; }
    if((f != kF)) { pid.setFF(f); kF = f; }
    if((max != kMaxOutput) || (min != kMinOutput)) 
    { 
      pid.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
  }

  public void setSpeedPID(int setpoint)
  {
    setpoint = 0;
    rpmGoal = setpoint;
    setMotor(rpmGoal);
  }

  @Config
  public void setMotor(int demand)
  {
    //demand = 1000;
    pid.setReference(-demand, ControlType.kVelocity);
    if (demand < 0.05) {
      cims.set(ControlMode.PercentOutput, 0);
    }
    else {
      cims.set(ControlMode.PercentOutput, 0.1);
    }
    
  }

  @Config
  public void setPercent(double percent) {
    pid.setReference(-percent, ControlType.kDutyCycle);
    cims.set(ControlMode.PercentOutput, 0.5);
  }

  @Config.ToggleButton
  public void setRunning(boolean run) {
    running = run;

  }
}
