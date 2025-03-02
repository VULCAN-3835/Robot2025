// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Collection;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants.EndAccessoryConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogInput;

public class EndAccessorySubsystem extends SubsystemBase {

    private TalonFX angleMotor;
    private TalonFX powerMotor;
    private Orchestra sound;

    private DutyCycleEncoder angleEncoder;
    private AnalogInput pieceDetector;

    private ProfiledPIDController profiledPIDController;
    private Constraints constraints;

    // the SysID routine
    private SysIdRoutine sysIdRoutine;
    private Config config;

    public EndAccessorySubsystem() {
        this.angleMotor = new TalonFX(EndAccessoryConstants.angleMotorID);
        this.powerMotor = new TalonFX(EndAccessoryConstants.powerMotorID);

        this.angleMotor.setNeutralMode(NeutralModeValue.Brake);

        TalonFXConfigurator configurator = angleMotor.getConfigurator();

        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        configurator.apply(motorOutputConfigs);

        this.sound = new Orchestra();
        sound.addInstrument(powerMotor);

        this.angleEncoder = new DutyCycleEncoder(EndAccessoryConstants.angleEncoderID);
        angleEncoder.setInverted(false);

        this.pieceDetector = new AnalogInput(EndAccessoryConstants.pieceDetectorID);

        this.constraints = new Constraints(EndAccessoryConstants.maxVelocity, EndAccessoryConstants.maxAcceleration);
        this.profiledPIDController = new ProfiledPIDController(EndAccessoryConstants.ProfiledkP,
                EndAccessoryConstants.profiledkI,
                EndAccessoryConstants.profiledkD, constraints);
        this.profiledPIDController.enableContinuousInput(0, 360);
        profiledPIDController.setGoal(EndAccessoryConstants.targetAngleRest.in(Degrees));
        profiledPIDController.setTolerance(EndAccessoryConstants.armAngleTolerence.in(Degree));


        // the SysID configs, for explanation on it go to the elevator subsystem in
        // lines 77-79
        // this.config = new Config(Volts.of(1).per(Seconds), Volts.of(2), Seconds.of(6));
        // this.sysIdRoutine = new SysIdRoutine(config,
        //         new SysIdRoutine.Mechanism(this::setVoltage,
        //                 Log -> {
        //                     Log.motor("Angle Motor")
        //                             .voltage(angleMotor.getMotorVoltage().getValue())
        //                             .angularPosition(Degrees.of(getAngle().in(Degrees)))
        //                             .angularVelocity(DegreesPerSecond
        //                                     .of(angleMotor.getVelocity().getValue().in(DegreesPerSecond)));
        //                 },
        //                 this));

    }

    // private void setVoltage(Voltage volts) {
    //     angleMotor.setVoltage(volts.in(Volts));
    // }

    // // sysID command for the quasistatic tests
    // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    //     return this.sysIdRoutine.quasistatic(direction);
    // }

    // // sysID command for the Dynamic tests
    // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    //     return this.sysIdRoutine.dynamic(direction);
    // }

    public enum DropAngles {
        setDropAngleL1, setDropAngleL2, setDropAngleL3, setDropAngleL4, restingAngle, intakeAngle, removeAlgea;
    }

    public WaitUntilCommand waitForCoral() {
        return new WaitUntilCommand(() -> hasPiece());
    }

    public void gripperIntake() {
        powerMotor.set(EndAccessoryConstants.kMotorPowerIntake);
    }

    public double getSpeed(DropAngles angle) {
        switch (angle) {
            case setDropAngleL1:
                return EndAccessoryConstants.kMotorPowerL1;
            case setDropAngleL2:
                return EndAccessoryConstants.kMotorPowerL2;
            case setDropAngleL3:
                return EndAccessoryConstants.kMotorPowerL3;
            case setDropAngleL4:
                return EndAccessoryConstants.kMotorPowerL4;
            case removeAlgea:
                return EndAccessoryConstants.removeAlgeaPower;
            default:
                return 0;
        }
    }

    public void gripperStop() {
        powerMotor.set(0);
    }

    public void setPower(double power) {
        powerMotor.set(power);
    }

    public void setAngle(DropAngles dropingLevel) {
        switch (dropingLevel) {
            case setDropAngleL1:
                profiledPIDController.setGoal(EndAccessoryConstants.targetDropAngleL1.in(Degree));
                break;
            case setDropAngleL2:
                profiledPIDController.setGoal(EndAccessoryConstants.targetDropAngleL2.in(Degree));
                break;
            case setDropAngleL3:
                profiledPIDController.setGoal(EndAccessoryConstants.targetDropAngleL3.in(Degree));
                break;
            case setDropAngleL4:
                profiledPIDController.setGoal(EndAccessoryConstants.targetDropAngleL4.in(Degree));
                break;
            case restingAngle:
                profiledPIDController.setGoal(EndAccessoryConstants.targetAngleRest.in(Degree));
                break;
            case intakeAngle:
                profiledPIDController.setGoal(EndAccessoryConstants.targetIntakeAngle.in(Degree));
                break;

            case removeAlgea:
                profiledPIDController.setGoal(EndAccessoryConstants.targetRemoveAlgea.in(Degree));
                break;
        }
    }

    public Angle getAngle() {
        double curr = angleEncoder.get() * 360;

        curr -= 224;
        if (curr < 0) {
            curr += 360;
        }

        return Degrees.of(curr);
    }

    public boolean hasPiece() {
        return pieceDetector.getVoltage() > EndAccessoryConstants.kHasPieceVoltageThreshold;

    }

    public boolean isAtSetpoint() {
        return Degrees.of(profiledPIDController.getSetpoint().position).minus(getAngle())
                .abs(Degrees) < EndAccessoryConstants.armAngleTolerence.in(Degrees);
    }

    public boolean isAtPidSetpoint() {
        return profiledPIDController.atGoal();
    }

    // end accessory SysID to use this paste it in the configureXboxBinding method
    // in robot container
    // xboxControllerDrive.a().whileTrue(endAccessorySubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // xboxControllerDrive.b().whileTrue(endAccessorySubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // xboxControllerDrive.y().whileTrue(endAccessorySubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // xboxControllerDrive.x().whileTrue(endAccessorySubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

    @Override
    public void periodic() {

        double profiledPIDOutput = profiledPIDController.calculate(getAngle().in(Degrees));

        if (getAngle().in(Degrees) <= EndAccessoryConstants.kMinAngle.in(Degrees) && profiledPIDOutput < 0) {
            angleMotor.set(0);
            System.out.println("error low");
        } else if (getAngle().in(Degrees) >= EndAccessoryConstants.kMaxAngle.in(Degrees) && profiledPIDOutput > 0) {
            angleMotor.set(0);
            System.out.println("error high ");
        } else {
            angleMotor.set(profiledPIDOutput);
        }

        if (getAngle().in(Degrees) == 136) {
            angleMotor.set(0);
            System.out.println("sensor not good");
        }

        SmartDashboard.putNumber("EndAccessory Subsystem/end current angle", getAngle().in(Degrees));
        SmartDashboard.putBoolean("EndAccessory Subsystem/end has piece?", hasPiece());
        SmartDashboard.putNumber("EndAccessory Subsystem/infrared end value", pieceDetector.getVoltage());
        SmartDashboard.putBoolean("EndAccessory Subsystem/ is angle connected", angleEncoder.isConnected());
        SmartDashboard.putNumber("EndAccessory Subsystem/ pid output", profiledPIDOutput);
        SmartDashboard.putBoolean("EndAccessory Subsystem/ is at setpoint", isAtSetpoint());
        SmartDashboard.putNumber("EndAccessory Subsystem/ pid setPoint", profiledPIDController.getGoal().position);

    }
}