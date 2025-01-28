// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndAccessorySubsystemConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.AnalogInput;

public class EndAccessorySubsystem extends SubsystemBase {

    private TalonFX angleMotor;
    private TalonFX powerMotor;

    private DigitalInput lowLimitSwitch;
    private DigitalInput highLimitSwitch;

    private DutyCycleEncoder angleEncoder;
    private AnalogInput pieceDetector;

    private Timer timer = new Timer();
    private static final double waitTime = 2.0;

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


    public Measure<AngleUnit> getAngle() {
        return Rotations.of(angleEncoder.get());
    }

    public boolean getHighLimitSwitch() {
        return highLimitSwitch.get();
    }

    public boolean getLowLimitSwitch() {
        return lowLimitSwitch.get();
    }

    public boolean hasPiece() {
        return pieceDetector.getVoltage() > EndAccessorySubsystemConstants.khHasPieceVoltageThreshold;
                                                                                   
    }

    @Override
    public void periodic() {

        double pidOutput = pidController.calculate(getAngle().in(Degrees));

        if (getLowLimitSwitch() && pidOutput < 0) {
            angleMotor.set(0);
        } else if (getHighLimitSwitch() && pidOutput > 0) {
            angleMotor.set(0);
        } else {
            angleMotor.set(pidOutput);
        }

        if (hasPiece()) {
            if (timer.get() == 0) {
                timer.start();
            }
            if (timer.get() > waitTime) {
                gripperRest();  
        } else {
            timer.stop();  
            timer.reset();  
            }
        }
    }
}