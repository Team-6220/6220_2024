// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.*;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ArmPIDSubsystem extends PIDSubsystem {
  /** Creates a new ArmPIDSubsystem. */
  private static ArmPIDSubsystem INSTANCE = null;

  private final CANSparkMax armMotorA, armMotorB;
  private final DutyCycleEncoder armEncoder;
  private ArmPIDSubsystem() {
    super(
        // The PIDController used by the subsystem
        new PIDController(ArmConstants.kP,ArmConstants.kI,ArmConstants.kD));
        // getController().setName("arm");
        getController().setTolerance(0.5);
        this.armMotorA = new CANSparkMax(ArmConstants.armMotorAID, MotorType.kBrushless);
        this.armMotorB = new CANSparkMax(ArmConstants.armMotorBID, MotorType.kBrushless);

        this.armMotorA.setInverted(ArmConstants.motorAInverted);
        this.armMotorB.setInverted(ArmConstants.motorBInverted);

        this.armMotorA.setIdleMode(IdleMode.kBrake);
        this.armMotorB.setIdleMode(IdleMode.kBrake);

        this.armMotorB.follow(armMotorA, true);

        this.armEncoder = new DutyCycleEncoder(ArmConstants.k_ENC_PORT);

        this.armMotorA.burnFlash();
        this.armMotorB.burnFlash();
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    if(output > 0.5)
    {
      output = 0.5;
    }
    else if(output < -0.5){
      output = -0.5;
    }
    this.armMotorA.set(output);
    SmartDashboard.putNumber("test 2", output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return getArmPosition();
  }

      /**
     * Converts a real world value (degrees of the arm) to a usable value (Encoder Value in rotations)
     * @param armPosition Arm Position in Degrees
     * @return the corresponding Encoder Value in rotations
     */
    public double convertArmDegreesToEncoderValue(double armPosition){
      return armPosition / 360;
  }

  /**
   * Converts a Encoder Value to a real world value (degrees of the arm)
   * @param encoderValue NEO motor position (in rotations)
   * @return the corresponding arm position to the given Encoder Value
   */
  public double convertEncoderValueToArmDegrees(double encoderValue){
      return encoderValue * 360;
  }

  public double getArmPosition(){
    return convertEncoderValueToArmDegrees(this.armEncoder.get()) + ArmConstants.armOffset;
}

  @Override
  public void periodic() {
      // TODO Auto-generated method stub
      super.periodic();
       SmartDashboard.putNumber("Arm Angle", getArmPosition());
      //  SmartDashboard.put(super.getController());
      SmartDashboard.putData("PID Controller", super.getController());
      System.out.println(SmartDashboard.getData("PID Controller"));
  }

  public static synchronized ArmPIDSubsystem getInstance() {
    if (INSTANCE == null) {
        INSTANCE = new ArmPIDSubsystem();
    }
    return INSTANCE;
}
}
