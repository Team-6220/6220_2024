package frc.robot.subsystems.AdvantageKitArm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.subsystems.AdvantageKitArm.ArmIO;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Robot;
import frc.robot.Constants.*;

public class ArmIOSparkMax implements ArmIO {
    private final CANSparkMax armMotorA = new CANSparkMax(ArmConstants.armMotorAID, MotorType.kBrushless);
    private final CANSparkMax armMotorB = new CANSparkMax(ArmConstants.armMotorBID, MotorType.kBrushless);
    
    private final DutyCycleEncoder armEncoder;
    private final RelativeEncoder builtinEncoderA = armMotorA.getEncoder();
    private final RelativeEncoder builtinEncoderB = armMotorB.getEncoder();
    public ArmIOSparkMax()
    {
        
    }
    

    /**
   * Update the inputs for the arm joint
   *
   * @param inputs The inputs to update
   */
  @Override
  public void updateInputs(ArmInputs inputs)
  {
  }

  /**
   * Set the target angle for the arm joint
   *
   * @param angle Target arm angle
   */
  @Override
  public void setTargetAngle(Rotation2d angle)
  {}

  @Override
  public void setTargetAngle(Rotation2d angle, double maxVel, double maxAccel)
  {

  }

  /**
   * Set the voltage output to the arm joint motors
   *
   * @param volts Voltage to output
   */
  public void setVoltage(double volts)
  {}

  public void brakeMotors()
  {

  }

  /**
   * Register self check compatible hardware with its associated subsystem
   *
   * @param subsystem The subsystem to register hardware on
   */
  default public void registerSelfCheckHardware(AdvancedSubsystem subsystem) {}

  /** Optimize status signals for running sysID */
  default public void optimizeForSysID() {}

}
