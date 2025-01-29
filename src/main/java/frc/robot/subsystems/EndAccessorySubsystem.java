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

    private static TalonFX angleMotor;
    private static TalonFX powerMotor;

    private static DigitalInput lowLimitSwitch;
    private static DigitalInput highLimitSwitch;

    private static DutyCycleEncoder angleEncoder;
    private static AnalogInput pieceDetector;

    private static Timer timer = new Timer();

    public EndAccessorySubsystem() {
        angleMotor = new TalonFX(EndAccessorySubsystemConstants.angleMotorID);
        powerMotor = new TalonFX(EndAccessorySubsystemConstants.powerMotorID);
        lowLimitSwitch = new DigitalInput(EndAccessorySubsystemConstants.lowLimitSwitchID);

        highLimitSwitch = new DigitalInput(EndAccessorySubsystemConstants.highLimitSwitchID);

        angleEncoder = new DutyCycleEncoder(EndAccessorySubsystemConstants.angleEncoderID);

        pieceDetector = new AnalogInput(EndAccessorySubsystemConstants.pieceDetectorID);
    }

    private static PIDController pidController = new PIDController(EndAccessorySubsystemConstants.kP, 0, 0);

    public enum srtDropAngle {
        setDropAngleL1, setDropAngleL2L3, setDropAngleL4;

        public void applyDropAngle(PIDController pidController) {
            switch (this) {
                case setDropAngleL1:
                    pidController.setSetpoint(EndAccessorySubsystemConstants.targetDropAngleL1.in(Degree));
                    break;
                case setDropAngleL2L3:
                    pidController.setSetpoint(EndAccessorySubsystemConstants.targetDropAngleL2L3.in(Degree));
                    break;
                case setDropAngleL4:
                    pidController.setSetpoint(EndAccessorySubsystemConstants.targetDropAngleL4.in(Degree));
                    break;
                default:
            }
        }
    }
    public void setIntakeAngle() {
        pidController.setSetpoint(EndAccessorySubsystemConstants.targetIntakeAngle.in(Degree));

    }

    public void gripperIntake() {
        powerMotor.set(EndAccessorySubsystemConstants.kMotorSpeed);
    }

    public void gripperRelease() {
        powerMotor.set(EndAccessorySubsystemConstants.kMotorSpeed);

    }

     public void gripperRest() {
        pidController.setSetpoint(EndAccessorySubsystemConstants.targetAngleRest.in(Degree));
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
            if (timer.get() > EndAccessorySubsystemConstants.waitTime) {
                gripperRest();
            } else {
                timer.stop();
                timer.reset();
            }
        }    
    }
}