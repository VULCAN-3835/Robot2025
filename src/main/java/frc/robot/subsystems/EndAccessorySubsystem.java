// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Millisecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants.EndAccessoryConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogInput;

public class EndAccessorySubsystem extends SubsystemBase {

    private TalonFX angleMotor;
    private TalonFX powerMotor;

    private DutyCycleEncoder angleEncoder;
    private AnalogInput pieceDetector;

    private Timer timer = new Timer();

    private PIDController pidController;
    private ProfiledPIDController profiledPIDController;
    private Constraints constraints;

    private SysIdRoutine sysIdRoutine;
    private Config config;

    public EndAccessorySubsystem() {
        this.angleMotor = new TalonFX(EndAccessoryConstants.angleMotorID);
        this.powerMotor = new TalonFX(EndAccessoryConstants.powerMotorID);

        this.angleEncoder = new DutyCycleEncoder(EndAccessoryConstants.angleEncoderID);
        this.pieceDetector = new AnalogInput(EndAccessoryConstants.pieceDetectorID);

        this.pidController = new PIDController(EndAccessoryConstants.kP, EndAccessoryConstants.kI,
                EndAccessoryConstants.kD);

        this.constraints = new Constraints(EndAccessoryConstants.maxVelocity, EndAccessoryConstants.maxAcceleration);
        this.profiledPIDController = new ProfiledPIDController(EndAccessoryConstants.ProfiledkP,
                EndAccessoryConstants.profiledkI,
                EndAccessoryConstants.profiledkD, constraints);

        this.config = new Config(Volts.of(0.02).per(Millisecond), Volts.of(2), Seconds.of(3));
        this.sysIdRoutine = new SysIdRoutine(config,
            new SysIdRoutine.Mechanism(this::setVoltage,
            Log -> {
                angleMotor.get();
                angleMotor.getMotorVoltage();
                angleMotor.getVelocity();
                    },
            this));

        pidController.setTolerance(EndAccessoryConstants.armAngleTolerence);
        profiledPIDController.setTolerance(EndAccessoryConstants.armAngleTolerence);
    }

    private void setVoltage(Voltage volts) {
        angleMotor.setVoltage(volts.in(Volts));
    }

    public enum DropAngles {
        setDropAngleL1, setDropAngleL2, setDropAngleL3, setDropAngleL4, restingAngle, intakeAngle;
    }

    public Command waitForCoral() {
        return new WaitUntilCommand(() -> timer.get() > EndAccessoryConstants.waitTime);
    }

    public void gripperIntake() {
        powerMotor.set(EndAccessoryConstants.kMotorSpeed);
    }

    public void gripperRelease() {
        powerMotor.set(-EndAccessoryConstants.kMotorSpeed);

    }

    public void gripperStop() {
        powerMotor.set(0);

    }

    public void setAngle(DropAngles dropingLevel) {
        switch (dropingLevel) {
            case setDropAngleL1:
                pidController.setSetpoint(EndAccessoryConstants.targetDropAngleL1.in(Degree));
                profiledPIDController.setGoal(EndAccessoryConstants.targetDropAngleL1.in(Degree));
                break;
            case setDropAngleL2:
                pidController.setSetpoint(EndAccessoryConstants.targetDropAngleL2.in(Degree));
                profiledPIDController.setGoal(EndAccessoryConstants.targetDropAngleL2.in(Degree));
                break;
            case setDropAngleL3:
                pidController.setSetpoint(EndAccessoryConstants.targetDropAngleL3.in(Degree));
                profiledPIDController.setGoal(EndAccessoryConstants.targetDropAngleL3.in(Degree));
                break;
            case setDropAngleL4:
                pidController.setSetpoint(EndAccessoryConstants.targetDropAngleL4.in(Degree));
                profiledPIDController.setGoal(EndAccessoryConstants.targetDropAngleL4.in(Degree));
                break;
            case restingAngle:
                pidController.setSetpoint(EndAccessoryConstants.targetAngleRest.in(Degree));
                profiledPIDController.setGoal(EndAccessoryConstants.targetAngleRest.in(Degree));
                break;
            case intakeAngle:
                pidController.setSetpoint(EndAccessoryConstants.targetIntakeAngle.in(Degree));
                profiledPIDController.setGoal(EndAccessoryConstants.targetIntakeAngle.in(Degree));
        }
    }

    private Measure<AngleUnit> getAngle() {
        return Rotations.of(angleEncoder.get());
    }

    public boolean hasPiece() {
        return pieceDetector.getVoltage() > EndAccessoryConstants.khHasPieceVoltageThreshold;

    }

    public boolean isAtSetpoint() {
        return pidController.atSetpoint() && profiledPIDController.atGoal();
    }

    @Override
    public void periodic() {

        double pidOutput = pidController.calculate(getAngle().in(Degrees));
        double profiledPIDOutput = profiledPIDController.calculate(getAngle().in(Degrees));
        double motorOutput = pidOutput + profiledPIDOutput;

        if (getAngle().in(Degree) < EndAccessoryConstants.kLowestAngle.in(Degree) && motorOutput < 0) {
            angleMotor.set(0);
        } else if (getAngle().in(Degree) > EndAccessoryConstants.kHighestAngle.in(Degree) && motorOutput > 0) {
            angleMotor.set(0);
        } else {
            angleMotor.set(motorOutput);
        }

        if (hasPiece()) {
            if (timer.get() == 0) {
                timer.start();
            }
            if (timer.get() > EndAccessoryConstants.waitTime) {
                gripperStop();
            }
        } else {
            timer.stop();
            timer.reset();
        }

        SmartDashboard.putBoolean("is endAccessory at setPoint?", isAtSetpoint());
        SmartDashboard.putNumber("current angle", getAngle().in(Degrees));
        SmartDashboard.putBoolean("end has piece?", hasPiece());
        SmartDashboard.putNumber("infrared end value", pieceDetector.getVoltage());
    }
}
