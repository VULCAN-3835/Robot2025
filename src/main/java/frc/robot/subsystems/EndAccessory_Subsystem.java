// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndAccessory_SubsystemConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Joystick;

public class EndAccessory_Subsystem extends SubsystemBase {

    // Motors
    private TalonFX angleMotor;
    private TalonFX powerMotor;

    // Switches
    private DigitalInput lowLimitSwitch;
    private DigitalInput highLimitSwitch;

    // Sensors
    private DutyCycleEncoder angleEncoder;
    private AnalogInput pieceDetector; 


    // Joystick
    private Joystick joystick;

    // Constructor
    public EndAccessory_Subsystem() {
      angleMotor = new TalonFX(EndAccessory_SubsystemConstants.AngleMotorPort);  // Motor to control the angle
      powerMotor  = new TalonFX(EndAccessory_SubsystemConstants.PowerMotorPort); // Motor to control the power
      lowLimitSwitch = new DigitalInput(EndAccessory_SubsystemConstants.LowLimitSwitchPort);// Switch for the low position
      highLimitSwitch  = new DigitalInput(EndAccessory_SubsystemConstants.HighLimitSwitchPort);// Switch for the high position
      angleEncoder = new DutyCycleEncoder(EndAccessory_SubsystemConstants.AngleEncoderPort);// Sensor to measure the angle
      pieceDetector = new AnalogInput(EndAccessory_SubsystemConstants.PieceDetectorPort);// Sensor to detect the piece
      joystick = new Joystick(EndAccessory_SubsystemConstants.JoystickPort);// Joystick to control the motors


    }
// PID constants
private static final double kP = 0;// Proportional constant


// PID controller
private PIDController pidController = new PIDController(kP, 0, 0);

// Target angle for drop position
private static final double targetDropAngle = 0.0;// Adjust this value as needed

  public void setDropAngle() {
    // Set the PID controller's setpoint to the target drop angle
    pidController.setSetpoint(targetDropAngle);
  
    // Use the PID controller to calculate the output value to the motor
    double pidOutput = pidController.calculate(getAngle());
  
    // Apply the PID output to the motor (adjust the scaling factor as needed)
    angleMotor.set(pidOutput * EndAccessory_SubsystemConstants.KAngleSpeed);
  }

    // Set the angle for intake (sets motor to intake position)
    public void setIntakeAngle() {
        angleMotor.set(EndAccessory_SubsystemConstants.KAngleSpeed);// Set motor speed for intake
    }

    // Turn on the gripper to take the piece
    public void gripperIntake() {
        powerMotor.set(EndAccessory_SubsystemConstants.KPowerSpeed);// Run the power motor to grab the piece
    }

    // Release the gripper
    public void gripperRelease() {
        powerMotor.set(EndAccessory_SubsystemConstants.KPowerSpeed);// Run the power motor in reverse to release the piece
    }

    // Stop the gripper
    public void gripperRest() {
        powerMotor.set(0);// Stop the power motor
    }

    // Get the current angle from the angle sensor
    public double getAngle() {
        return angleEncoder.get();// Get the angle from the sensor
    }

    // Check if the high position switch is pressed
    public boolean getHighLimitSwitch() {
        return highLimitSwitch.get();// Return true if the high position switch is pressed
    }

    // Check if the low position switch is pressed
    public boolean getLowLimitSwitch() {
        return lowLimitSwitch.get();// Return true if the low position switch is pressed
    }

    // Check if a piece is detected by the sensor
    public boolean hasPiece() {
        return pieceDetector.getVoltage() > EndAccessory_SubsystemConstants.KThreshold;// Return true if the sensor detects a piece
    }

    @Override
    public void periodic() {
        // Get sensor values
        double angle = getAngle();// Get the angle from the sensor
        double infraredValue = pieceDetector.getVoltage();// Get the value from the infrared sensor

        // Control the angle motor with limit switches
        if (getLowLimitSwitch()) {
            angleMotor.set(0);// Stop motor if low position is reached
        } else if (getHighLimitSwitch()) {
            angleMotor.set(0);// Stop motor if high position is reached
        } else {
            // Otherwise, control the angle motor with the joystick input
            double joystickInput = joystick.getY();// Get joystick Y-axis input for motor control
            angleMotor.set(joystickInput * EndAccessory_SubsystemConstants.KAngleSpeed);
        }

        // Set the release angle (min or max position)
    }
}