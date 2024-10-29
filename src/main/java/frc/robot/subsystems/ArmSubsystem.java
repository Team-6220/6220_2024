// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import edu.wpi.first.wpilibj.Timer;
// import frc.robot.Constants.*;
// import frc.robot.commands.ArmIdleCommand;
// import frc.lib.util.TunableNumber;

// // import frc.robot.Constants.ArmConstants;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkBase.IdleMode;
// import com.revrobotics.CANSparkLowLevel.MotorType;

// public class ArmSubsystem extends SubsystemBase{
//     private static ArmSubsystem INSTANCE = null; //Created so that only 1 instance of arm subsystem is 
//     // created at all time. Think of it as a "static" call to the subsystem where you can get static variables

//     private final TunableNumber armKp = new TunableNumber("Arm kP", ArmConstants.kP);
//     private final TunableNumber armKi = new TunableNumber("Arm kI", ArmConstants.kI);
//     private final TunableNumber armKd = new TunableNumber("Arm kD", ArmConstants.kD);
//     private final TunableNumber armKg = new TunableNumber("Arm kG", ArmConstants.kG);
//     private final TunableNumber armKv = new TunableNumber("Arm kV", ArmConstants.kV);
//     private final TunableNumber armKs = new TunableNumber("Arm kS", ArmConstants.kS);

//     private final TunableNumber armMaxVel = new TunableNumber("ArmMaxVel", ArmConstants.armMaxVel);
//     private final TunableNumber armMaxAccel = new TunableNumber("ArmMaxAccel", ArmConstants.armMaxAccel);

//     public final TunableNumber armTestAngle = new TunableNumber("Arm Degree Goal Set", 75);
//     //public final TunableNumber armAmpAngle = new TunableNumber("Amp Degree Set", ArmConstants.ampSetPoint);
    
//     private final CANSparkMax armMotorA, armMotorB;
//     private final DutyCycleEncoder armEncoder;
    
//     private final ProfiledPIDController m_Controller;
//     private ArmFeedforward m_FeedForward;
//     private TrapezoidProfile.Constraints m_Constraints;
//     private double lastTurnUpdate = 0;
//     private double feedForwardOutput, PIDOutput;
//     /**
//      * Initializes the ArmSubsystem
//      */
//     private ArmSubsystem() {
//         armMotorA = new CANSparkMax(ArmConstants.armMotorAID, MotorType.kBrushless);
//         armMotorB = new CANSparkMax(ArmConstants.armMotorBID, MotorType.kBrushless);

//         armMotorA.setInverted(ArmConstants.motorAInverted);
//         armMotorB.setInverted(ArmConstants.motorBInverted);

//         armMotorA.setIdleMode(IdleMode.kBrake);
//         armMotorB.setIdleMode(IdleMode.kBrake);

//         armMotorB.follow(armMotorA, true);

//         armEncoder = new DutyCycleEncoder(ArmConstants.k_ENC_PORT);

//         armMotorA.burnFlash();
//         armMotorB.burnFlash();

//         m_Constraints = new TrapezoidProfile.Constraints(armMaxVel.get(), armMaxAccel.get());

//         m_Controller = new ProfiledPIDController(
//             armKp.get(),
//             armKi.get(),
//             armKd.get(),
//             m_Constraints
//             );
//         m_FeedForward = new ArmFeedforward(armKs.get(), armKg.get(), armKv.get());
//         //Using I only withing 3 degrees of error
//         m_Controller.setIZone(3);

//         //Setting Tolerance
//         m_Controller.setTolerance(1.5);
        
//         // setDefaultCommand(new ArmIdleCommand());
//         // initDefaultCommand();
//     }

//     /**
//      * Sets the default command of the bot to arm idle command
//      */
//     public void initDefaultCommand() {
//         // Set the default command for a subsystem here.
//         setDefaultCommand(new ArmIdleCommand());
//     }

//     /**
//      * Converts a real world value (degrees of the arm) to a usable value (Encoder Value in rotations)
//      * @param armPosition Arm Position in Degrees
//      * @return the corresponding Encoder Value in rotations
//      */
//     public double convertArmDegreesToEncoderValue(double armPosition){
//         return armPosition / 360;
//     }

//     /**
//      * Converts a Encoder Value to a real world value (degrees of the arm)
//      * @param encoderValue NEO motor position (in rotations)
//      * @return the corresponding arm position to the given Encoder Value
//      */
//     public double convertEncoderValueToArmDegrees(double encoderValue){
//         return encoderValue * 360;
//     }

//     /**
//      * Sets the current of the NEO arm motors for manual driving
//      * @param  speed  a value from -1 to 1 (sets current of motor)
//      */
//     public void simpleDrive(double speed){
//         if(speed > 0.5){
//             speed = 0.5;
//         }
//         else if (speed < -0.5){
//             speed = -0.5;
//         }
//         armMotorA.set(speed);
//         // System.out.println(speed);
//     }

//     /**
//      * Sets the goal for the controller and drives motors
//      * @param  goal  a position in degrees for the arm
//      */
//     public void driveToGoal(double goal) {
//         //System.out.println("Driving To Goal");

//         if (Timer.getFPGATimestamp() - 0.2 > lastTurnUpdate) {
//             resetPid();
//         }
//         lastTurnUpdate = Timer.getFPGATimestamp();

//         m_Controller.setGoal(goal);
        
//         PIDOutput = m_Controller.calculate(getArmPosition());
//         feedForwardOutput = m_FeedForward.calculate((m_Controller.getSetpoint().position+90) * Math.PI/180, m_Controller.getSetpoint().velocity);
//         double calculatedSpeed = PIDOutput + feedForwardOutput;

        
//         SmartDashboard.putNumber("Arm Goal", goal);
//         armMotorA.setVoltage(calculatedSpeed);
//         //+90 because feed forward want the angle to be 0 at horizontal for gravity calculations
//     }

//     public void stop(){
//         armMotorA.set(0);
//         m_Controller.reset(getArmPosition());
//     }

//     public void resetPid() {
//         m_Controller.reset(getArmPosition());
//     }

//     /**
//      * Calculates the output of the arm PID for a given setpoint
//      * @param  setpoint desired arm position in degrees
//      * @return
//      */
//     public double calculate(double setpoint){
//         return -1 * m_Controller.calculate(getArmPosition(), setpoint);
//     }
//     /**
//      * Gives the position of the arm in degrees
//      * @returns the value in degrees of the arm    
//      */
//     public double getArmPosition(){
//         return convertEncoderValueToArmDegrees(armEncoder.get()) - ArmConstants.armOffset;
//     }

//     public boolean isAtGoal() {
//         return m_Controller.atGoal();
//     }

//     @Override
//     public void periodic() {
//         if(getArmPosition() > 92 || getArmPosition() < -10) {
//             stop();
//             System.out.println("Arm Over Extend");
//         }

//         // This method will be called once per scheduler run
//         if(armKp.hasChanged()
//         || armKi.hasChanged()
//         || armKd.hasChanged())
//         {
//             m_Controller.setPID(armKp.get(),armKi.get(),armKd.get());
//         }

//         if(armKs.hasChanged()
//         || armKg.hasChanged()
//         || armKv.hasChanged()) {
//             m_FeedForward = new ArmFeedforward(armKs.get(), armKg.get(), armKv.get());
//         }

//         if(armMaxVel.hasChanged()
//         || armMaxAccel.hasChanged()) {
//             m_Constraints = new TrapezoidProfile.Constraints(armMaxVel.get(), armMaxAccel.get());
//             m_Controller.setConstraints(m_Constraints);
//         }
//         SmartDashboard.putNumber("Arm Angle", getArmPosition());
//         //SmartDashboard.putNumber("Controller Setpoint", m_Controller.getSetpoint().position);
//         //SmartDashboard.putNumber("Controller Error", m_Controller.getPositionError());
//         //SmartDashboard.putNumber("PID Controller Output", PIDOutput);
//         //SmartDashboard.putNumber("Feed Forward Output", feedForwardOutput);
//         //System.out.println(getArmPosition());
//         SmartDashboard.putBoolean("arm at goal", isAtGoal());
//     }

//     /**
//      * Accesses the static instance of the ArmSubsystem singleton
//      * @return ArmSubsystem Singleton Instance
//      */
//     public static synchronized ArmSubsystem getInstance() {
//         if (INSTANCE == null) {
//             INSTANCE = new ArmSubsystem();
//         }
//         return INSTANCE;
//     }
// }
