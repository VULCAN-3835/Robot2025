// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndAccessorySubsystemConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;

public class EndAccessorySubsystem extends SubsystemBase {

    private TalonFX angleMotor;
    private TalonFX powerMotor;

    private DigitalInput lowLimitSwitch;
    private DigitalInput highLimitSwitch;

    private DutyCycleEncoder angleEncoder;
    private AnalogInput pieceDetector;

    public EndAccessorySubsystem() {
        angleMotor = new TalonFX(EndAccessorySubsystemConstants.angleMotorID); // Motor to control the angle
        powerMotor = new TalonFX(EndAccessorySubsystemConstants.powerMotorID); // Motor to control the power
        lowLimitSwitch = new DigitalInput(EndAccessorySubsystemConstants.lowLimitSwitchID);// Switch for the low
                                                                                             // position
        highLimitSwitch = new DigitalInput(EndAccessorySubsystemConstants.highLimitSwitchID);// Switch for the high
                                                                                               // position
        angleEncoder = new DutyCycleEncoder(EndAccessorySubsystemConstants.angleEncoderID);// Sensor to measure the
                                                                                             // angle
        pieceDetector = new AnalogInput(EndAccessorySubsystemConstants.pieceDetectorID);// Sensor to detect the piece
    }

    // PID
    private static final double kP = 0;// Proportional constant
    private PIDController pidController = new PIDController(kP, 0, 0);

    // Set the angle to drop position using PID control
    public void setDropAngle() {
        pidController.setSetpoint(EndAccessorySubsystemConstants.targetDropAngle.in(Degree)); // Set the target drop
                                                                                             
    }


    public void setIntakeAngle() {
        pidController.setSetpoint(EndAccessorySubsystemConstants.targetIntakeAngle.in(Degree)); // Set the target intake
                                                                                                
    }

    public void gripperIntake() {
        powerMotor.set(EndAccessorySubsystemConstants.kPowerSpeed);
    }

    public void gripperRelease() {
        powerMotor.set(-EndAccessorySubsystemConstants.kPowerSpeed);
                                                                   
    }

    public void gripperRest() {
        powerMotor.set(0);
    }

    // Get the current angle from the angle sensor
    public double getAngle() {
        return angleEncoder.get() * 360;// Get the angle from the sensor
    }

    public boolean getHighLimitSwitch() {
        return highLimitSwitch.get();// Return true if the high position switch is pressed
    }

    public boolean getLowLimitSwitch() {
        return lowLimitSwitch.get();// Return true if the low position switch is pressed
    }

    // Check if a piece is detected by the sensor
    public boolean hasPiece() {
        return pieceDetector.getVoltage() > EndAccessorySubsystemConstants.kThreshold;
                                                                                   
    }

    // Stop the motor after each operation
    private void stopMotors() {
        angleMotor.set(0);
        powerMotor.set(0);
    }

    @Override
    public void periodic() {

        double pidOutput = pidController.calculate(getAngle()); // Calculate PID output based on current angle

        // Control the angle motor with limit switches
        if (getLowLimitSwitch() && pidOutput < 0) {
            angleMotor.set(0); // Stop motor if low position is reached and motor is trying to move downward
        } else if (getHighLimitSwitch() && pidOutput > 0) {
            angleMotor.set(0); // Stop motor if high position is reached and motor is trying to move upward
        } else {
            angleMotor.set(pidOutput);
        }
        if (hasPiece()) {
            gripperRest();
        }
    }
 }

