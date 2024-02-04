package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.*;
import frc.lib.util.TunableNumber;
// import frc.robot.Constants.ArmConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ArmSubsystem extends SubsystemBase{
    private static ArmSubsystem INSTANCE = null; //Created so that only 1 instance of arm subsystem is 
    // created at all time. Think of it as a "static" call to the subsystem where you can get static variables

    private final TunableNumber armKp = new TunableNumber("Arm kP", 0);
    private final TunableNumber armKi = new TunableNumber("Arm kI", 0);
    private final TunableNumber armKd = new TunableNumber("Arm kD", 0);

    private final CANSparkMax armMotorA, armMotorB;
    private final DutyCycleEncoder armEncoder;
    private final PIDController pid;
    
    /**
     * Initializes the ArmSubsystem
     */
    private ArmSubsystem() {
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

        this.pid = new PIDController(
            armKp.get(),
            armKi.get(),
            armKd.get()
            );
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

    /**
     * Sets the current of the NEO arm motors for manual driving
     * @param  speed  a value from -1 to 1 (sets current of motor)
     */
    public void drive(double speed){
        if(speed > 0.5){
            speed = 0.5;
        }
        else if (speed < -0.5){
            speed = -0.5;
        }
        this.armMotorA.set(speed);
        SmartDashboard.putNumber("testt", speed);
    }

    /**
     * Calculates the output of the arm PID for a given setpoint
     * @param  setpoint desired arm position in degrees
     * @return
     */
    public double calculate(double setpoint){
        return -1 * pid.calculate(getArmPosition(), setpoint);
    }
    /**
     * Gives the position of the arm in degrees
     * @returns the value in degrees of the arm    
     */
    public double getArmPosition(){
        return convertEncoderValueToArmDegrees(this.armEncoder.get()) + ArmConstants.armOffset;
    }

    @Override
    public void periodic() {
         // This method will be called once per scheduler run
        if(armKp.hasChanged()
        || armKi.hasChanged()
        || armKd.hasChanged())
        {
            pid.setPID(armKp.get(),armKi.get(),armKd.get());
        }
         SmartDashboard.putNumber("Arm Angle", getArmPosition());
    }

    /**
     * Accesses the static instance of the ArmSubsystem singleton
     * @return ArmSubsystem Singleton Instance
     */
    public static synchronized ArmSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new ArmSubsystem();
        }
        return INSTANCE;
    }
}
