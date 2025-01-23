// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsytem extends SubsystemBase {
  /** Creates a new ClimbSubsytem. */
  TalonFX climbMotor;
  DigitalInput ClimbLimitSwitch;
  
  public ClimbSubsytem() {
  this.climbMotor= new TalonFX(Constants.ClimbConstants.climbMotorPort);
  this.ClimbLimitSwitch = new DigitalInput(Constants.ClimbConstants.LimitSwitchPort);
}
//sets climbMotor to the desired power
public void setMotor(double power){
  climbMotor.set(power);
}
//returns the value of ClimbLimitSwitch
public boolean getClimbLimitSwitch(){
   boolean valueOfLimitSwitch= ClimbLimitSwitch.get();
   return valueOfLimitSwitch;
}
// starts climbing by using setMotor() to set climbMotor's power
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
