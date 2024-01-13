// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// // import com.ctre.phoenix.sensors.PigeonIMU;
// import com.kauailabs.navx.frc.AHRS;
// // import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
// import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
// import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
// import com.swervedrivespecialties.swervelib.SwerveModule;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// // import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
// import edu.wpi.first.math.kinematics.SwerveModulePosition;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.networktables.GenericEntry;
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import edu.wpi.first.wpilibj.SPI;

// import static frc.robot.Constants.*;

// // import java.lang.reflect.GenericArrayType;

// public class DrivetrainSubsystem extends SubsystemBase {
//   /**
//    * The maximum voltage that will be delivered to the drive motors.
//    * <p>
//    * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
//    */
//   public static final double MAX_VOLTAGE = 12.0;
//   // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
//   //  The formula for calculating the theoretical maximum velocity is:
//   //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
//   //  By default this value is setup for a Mk4i standard module using Falcon500s to drive.
//   //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
//   //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
//   /**
//    * The maximum velocity of the robot in meters per second.
//    * <p>
//    * This is a measure of how fast the robot should be able to drive in a straight line.
//    */
//   public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 * SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;
//   /**
//    * The maximum angular velocity of the robot in radians per second.
//    * <p>
//    * This is a measure of how fast the robot can rotate in place.
//    */
//   // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
//   public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
//           Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);



//   // By default we use a Pigeon for our gyroscope. But if you use another gyroscope, like a NavX, you can change this.
//   // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
//   // cause the angle reading to increase until it wraps back over to zero.
//  private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

//   // These are our modules. We initialize them in the constructor.
//   private final SwerveModule m_frontLeftModule;
//   private final SwerveModule m_frontRightModule;
//   private final SwerveModule m_backLeftModule;
//   private final SwerveModule m_backRightModule;
//   private long prevTimeMilli;

//   private final SwerveDriveOdometry odometer;
//   private SwerveModulePosition[] positions = {
//         new SwerveModulePosition(),
//         new SwerveModulePosition(),
//         new SwerveModulePosition(),
//         new SwerveModulePosition()
//     };

//   private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
//   private SwerveModuleState[] m_states = {new SwerveModuleState(),new SwerveModuleState(),new SwerveModuleState(),new SwerveModuleState()};
// private boolean lock, auto;
// private final GenericEntry isLocked, pitch, roll, currPose, autoRunning, angle, flstate, frstate, blstate,brstate;

//   public DrivetrainSubsystem() {
//         new Thread(() -> {
//             try {
//                 Thread.sleep(1000);
//                 zeroHeading();
//             } 
//             catch (Exception e) {
//             }
//         }).start();
//         this.odometer = new SwerveDriveOdometry(Constants.m_kinematics, new Rotation2d(0), positions);
//     ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
//     lock = false;
//     auto = false;
//     this.isLocked = tab.add("locked wheels", lock).getEntry();
//     this.pitch = tab.add("pitch", 0).getEntry();
//     this.roll = tab.add("roll", 0).getEntry();
//     this.currPose = tab.add("currPose", getPose().toString()).getEntry();
//     this.autoRunning = tab.add("autoRunning", auto).getEntry();
//     this.angle = tab.add("gyro angle", 0).getEntry();
//     this.flstate = tab.add("flstate", "").getEntry();
//     this.frstate = tab.add("frstate", "").getEntry();
//     this.blstate = tab.add("blstate", "").getEntry();
//     this.brstate = tab.add("brstate", "").getEntry();
//     // There are 4 methods you can call to create your swerve modules.
//     // The method you use depends on what motors you are using.
//     //
//     // Mk4iSwerveModuleHelper.createFalcon500(...)
//     //   Your module has two Falcon 500s on it. One for steering and one for driving.
//     //
//     // Mk4iSwerveModuleHelper.createNeo(...)
//     //   Your module has two NEOs on it. One for steering and one for driving.
//     //
//     // Mk4iSwerveModuleHelper.createFalcon500Neo(...)
//     //   Your module has a Falcon 500 and a NEO on it. The Falcon 500 is for driving and the NEO is for steering.
//     //
//     // Mk4iSwerveModuleHelper.createNeoFalcon500(...)
//     //   Your module has a NEO and a Falcon 500 on it. The NEO is for driving and the Falcon 500 is for steering.
//     //
//     // Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper class.

//     // By default we will use Falcon 500s in standard configuration. But if you use a different configuration or motors
//     // you MUST change it. If you do not, your code will crash on startup.
//     // FIXME Setup motor configuration
//     m_frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500Neo(
//             // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
//             tab.getLayout("Front Left Module", BuiltInLayouts.kList)
//                     .withSize(2, 4)
//                     .withPosition(0, 0),
//             // This can either be STANDARD or FAST depending on your gear configuration
//             Mk4iSwerveModuleHelper.GearRatio.L2,
//             // This is the ID of the drive motor
//             FRONT_LEFT_MODULE_DRIVE_MOTOR,
//             // This is the ID of the steer motor
//             FRONT_LEFT_MODULE_STEER_MOTOR,
//             // This is the ID of the steer encoder
//             FRONT_LEFT_MODULE_STEER_ENCODER,
//             // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
//             FRONT_LEFT_MODULE_STEER_OFFSET
//     );

//     // We will do the same for the other modules
//     m_frontRightModule = Mk4iSwerveModuleHelper.createFalcon500Neo(
//             tab.getLayout("Front Right Module", BuiltInLayouts.kList)
//                     .withSize(2, 4)
//                     .withPosition(2, 0),
//             Mk4iSwerveModuleHelper.GearRatio.L2,
//             FRONT_RIGHT_MODULE_DRIVE_MOTOR,
//             FRONT_RIGHT_MODULE_STEER_MOTOR,
//             FRONT_RIGHT_MODULE_STEER_ENCODER,
//             FRONT_RIGHT_MODULE_STEER_OFFSET
//     );

//     m_backLeftModule = Mk4iSwerveModuleHelper.createFalcon500Neo(
//             tab.getLayout("Back Left Module", BuiltInLayouts.kList)
//                     .withSize(2, 4)
//                     .withPosition(4, 0),
//             Mk4iSwerveModuleHelper.GearRatio.L2,
//             BACK_LEFT_MODULE_DRIVE_MOTOR,
//             BACK_LEFT_MODULE_STEER_MOTOR,
//             BACK_LEFT_MODULE_STEER_ENCODER,
//             BACK_LEFT_MODULE_STEER_OFFSET
//     );

//     m_backRightModule = Mk4iSwerveModuleHelper.createFalcon500Neo(
//             tab.getLayout("Back Right Module", BuiltInLayouts.kList)
//                     .withSize(2, 4)
//                     .withPosition(6, 0),
//             Mk4iSwerveModuleHelper.GearRatio.L2,
//             BACK_RIGHT_MODULE_DRIVE_MOTOR,
//             BACK_RIGHT_MODULE_STEER_MOTOR,
//             BACK_RIGHT_MODULE_STEER_ENCODER,
//             BACK_RIGHT_MODULE_STEER_OFFSET
//     );
    
//     prevTimeMilli = System.currentTimeMillis();
//   }


//   /**
//    * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
//    * 'forwards' direction.
//    */
//   public void zeroHeading() {
//         m_navx.reset();
//         m_navx.zeroYaw();
//     }

//     public Rotation2d getPitch() {
//       return new Rotation2d(getGyroPitch() * -1 / 180 * Math.PI);
//     }

//     public Rotation2d getRoll() {
//       return new Rotation2d(getGyroRoll() * -1 / 180 * Math.PI);
//     }

//     public double getHeading() {
//         return 90-(m_navx.getYaw());
//     }

//   public Rotation2d getGyroscopeRotation() {
//         return m_navx.getRotation2d();
//   }
//   public Pose2d getPose(){
//         return odometer.getPoseMeters();
//   }
//   public void resetOdometry(Pose2d pose2d){
//         this.positions[0] = new SwerveModulePosition();
//         this.positions[1] = new SwerveModulePosition();
//         this.positions[2] = new SwerveModulePosition();
//         this.positions[3] = new SwerveModulePosition();
//         odometer.resetPosition(getGyroscopeRotation(), positions, pose2d);
//   }
//   public void drive(SwerveModuleState[] states) {
//     m_states = states;
//   }

//   public double getGyroPitch(){
//         return m_navx.getPitch();
//   }

//   public double getGyroRoll(){
//         return m_navx.getRoll();
//   }
//   public void updateAuto(boolean value){
//         this.auto = value;
//   }
//   public void updatePositions(SwerveModuleState[] states){
//         long time = System.currentTimeMillis();
//         double diff = (double)(time - prevTimeMilli)/1000d;
//         roll.setDouble(positions[0].distanceMeters + (m_frontLeftModule.getDriveVelocity() * diff));
//      this.positions[0] = new SwerveModulePosition(positions[0].distanceMeters + (m_frontLeftModule.getDriveVelocity() * diff), new Rotation2d(m_frontLeftModule.getSteerAngle()));
//      this.positions[1] = new SwerveModulePosition(positions[1].distanceMeters + (m_frontRightModule.getDriveVelocity() * diff), new Rotation2d(m_frontRightModule.getSteerAngle()));
//      this.positions[2] = new SwerveModulePosition(positions[2].distanceMeters + (m_backLeftModule.getDriveVelocity() * diff), new Rotation2d(m_backLeftModule.getSteerAngle()));
//      this.positions[3] = new SwerveModulePosition(positions[3].distanceMeters + (m_backRightModule.getDriveVelocity() * diff), new Rotation2d(m_backRightModule.getSteerAngle()));
//      prevTimeMilli = time;
//      odometer.update(Rotation2d.fromDegrees(getHeading()), positions);

//   }
//   public void toggleLock(){
//         this.lock = !this.lock;
//         isLocked.setBoolean(lock);
//   }

//   @Override
//   public void periodic() {
//     SwerveModuleState[] states = m_states;
//     SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
//     if(this.lock){
//         m_frontLeftModule.set(0, 45);
//         m_frontRightModule.set(0, -45);
//         m_backLeftModule.set(0, -45);
//         m_backRightModule.set(0, 45);
//     }else{
//         m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
//         m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
//         m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
//         m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
//     }
//     //roll.setDouble(getGyroRoll());
//     pitch.setDouble(getGyroPitch());
//     isLocked.setBoolean(lock);
//     currPose.setString(getPose().toString());
//     autoRunning.setBoolean(auto);
//     angle.setDouble(m_frontLeftModule.getDriveVelocity());
//     flstate.setString(states[0].toString());
//     frstate.setString(states[1].toString());
//     blstate.setString(states[2].toString());
//     brstate.setString(states[3].toString());
//     updatePositions(states);
//   }
// }
