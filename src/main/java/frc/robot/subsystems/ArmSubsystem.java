package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkRelativeEncoder.Type;


public class ArmSubsystem extends SubsystemBase{
    private static ArmSubsystem INSTANCE = null; //Created so that only 1 instance of arm subsystem is 
    // created at all time. Think of it as a "static" call to the subsystem where you can get static variables

    private final CANSparkMax armMotorA, armMotorB;
    private final SparkPIDController armPidController;
    private final RelativeEncoder armRelativeEncoder;
    
    /**
     * Initializes the ArmSubsystem
     */
    private ArmSubsystem() {
        this.armMotorA = new CANSparkMax(ArmConstants.armMotorAID, MotorType.kBrushless);
        this.armMotorB = new CANSparkMax(ArmConstants.armMotorBID, MotorType.kBrushless);

        this.armMotorA.setInverted(ArmConstants.motorAInverted);
        this.armMotorB.setInverted(ArmConstants.motorBInverted);

        this.armMotorA.setIdleMode(null);
        this.armMotorB.setIdleMode(null);

        this.armMotorB.follow(armMotorA);

        this.armRelativeEncoder = armMotorA.getEncoder(Type.kHallSensor, 42);

        this.armPidController = armMotorA.getPIDController();
        this.armPidController.setP(ArmConstants.kP);
        this.armPidController.setI(ArmConstants.kI);
        this.armPidController.setD(ArmConstants.kD);

        this.armMotorA.burnFlash();
        this.armMotorB.burnFlash();
    }

    /**
     * Converts a real world value (degrees of the arm) to a usable value (NEO Position in rotations)
     * @param armPosition Arm Position in Degrees
     * @return the corresponding NEO position in rotations
     */
    public double convertArmDegreesToNeoPosition(double armPosition){
        return armPosition / 360 * (361 / 3);
    }

    /**
     * Converts a NEO Position to a real world value (degrees of the arm)
     * @param neoPosition NEO motor position (in rotations)
     * @return the corresponding arm position to the given NEO Position
     */
    public double convertNeoPositionToArmDegrees(double neoPosition){
        return neoPosition / (361 / 3) * 360;
    }

    public void setArmPosition(double armPosition){
        this.armPidController.setReference(convertArmDegreesToNeoPosition(armPosition), ControlType.kPosition);
    }
    /**
     * Sets the current of the NEO arm motors for manual driving
     * @param  speed  a value from -1 to 1 (sets current of motor)
     */
    public void drive(double speed){
        this.armMotorA.set(speed);
    }

    /**
     * Gives the position of the arm in degrees
     * @returns the value in degrees of the arm    
     */
    public double getArmPosition(){
        return convertArmDegreesToNeoPosition(this.armRelativeEncoder.getPosition());
    }

    @Override
    public void periodic() {
         // This method will be called once per scheduler run
         SmartDashboard.putNumber("Arm Angle", getInstance().getArmPosition());
    }

    /**
     * Accesses the static instance of the ArmSubsystem singleton
     * @return ArmSubsystem Singleton Instance
     */
    public static ArmSubsystem getInstance() {
            if (INSTANCE == null) {
            INSTANCE = new ArmSubsystem();
        }
        return INSTANCE;
    }
}
