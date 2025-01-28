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
        angleMotor = new TalonFX(EndAccessorySubsystemConstants.angleMotorID);
        powerMotor = new TalonFX(EndAccessorySubsystemConstants.powerMotorID);
        lowLimitSwitch = new DigitalInput(EndAccessorySubsystemConstants.lowLimitSwitchID);
                                                                                  
        highLimitSwitch = new DigitalInput(EndAccessorySubsystemConstants.highLimitSwitchID);
                                                                                       
        angleEncoder = new DutyCycleEncoder(EndAccessorySubsystemConstants.angleEncoderID);
                                                                                           
        pieceDetector = new AnalogInput(EndAccessorySubsystemConstants.pieceDetectorID);
    }

  
    private static final double kP = 0;
    private PIDController pidController = new PIDController(kP, 0, 0);

    public void setDropAngle() {
        pidController.setSetpoint(EndAccessorySubsystemConstants.targetDropAngle.in(Degree));
                                                                                             
    }


    public void setIntakeAngle() {
        pidController.setSetpoint(EndAccessorySubsystemConstants.targetIntakeAngle.in(Degree));
                                                                                                
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


    public double getAngle() {
        return angleEncoder.get() * 360;// Get the angle from the sensor
    }

    public boolean getHighLimitSwitch() {
        return highLimitSwitch.get();
    }

    public boolean getLowLimitSwitch() {
        return lowLimitSwitch.get();
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

        double pidOutput = pidController.calculate(getAngle());

        if (getLowLimitSwitch() && pidOutput < 0) {
            angleMotor.set(0);
        } else if (getHighLimitSwitch() && pidOutput > 0) {
            angleMotor.set(0);
        } else {
            angleMotor.set(pidOutput);
        }
        if (hasPiece()) {
            gripperRest();
        }
    }
 }

