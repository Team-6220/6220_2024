package frc.robot.subsystems.AdvantageKitArm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.subsystems.AdvantageKitArm.ArmIO;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants.*;

public class ArmIOSparkMax implements ArmIO {
    private final CANSparkMax leftArmMotor, rightArmMotor;
    private final DutyCycleEncoder armEncoder;

    public ArmIOSparkMax()
    {
        leftArmMotor = new CANSparkMax(ArmConstants.armMotorAID, MotorType.kBrushless);
    }
    

    /**
   * Update the inputs for the arm joint
   *
   * @param inputs The inputs to update
   */
  @Override
  void updateInputs(ArmInputs inputs){}

  /**
   * Set the target angle for the arm joint
   *
   * @param angle Target arm angle
   */
  @Override
  void setTargetAngle(Rotation2d angle){}

  @Override
  void setTargetAngle(Rotation2d angle, double maxVel, double maxAccel)

  /**
   * Set the voltage output to the arm joint motors
   *
   * @param volts Voltage to output
   */
  void setVoltage(double volts);

  void brakeMotors();

  /**
   * Register self check compatible hardware with its associated subsystem
   *
   * @param subsystem The subsystem to register hardware on
   */
  default void registerSelfCheckHardware(AdvancedSubsystem subsystem) {}

  /** Optimize status signals for running sysID */
  default void optimizeForSysID() {}

}
