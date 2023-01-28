// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.Nat;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.MathUtil;


import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;

import java.util.ArrayList;
import edu.wpi.first.math.Pair;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants.Drivetrain.*;
import frc.robot.Constants.CanId;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import java.io.File;


public class Drivetrain extends SubsystemBase {
  private final WPI_TalonSRX m_leftLead = new WPI_TalonSRX(CanId.leftTalonLead);
  private final WPI_TalonSRX m_leftFollow = new WPI_TalonSRX(CanId.leftTalonFollow);
  private final WPI_TalonSRX m_rightLead = new WPI_TalonSRX(CanId.rightTalonLead);
  private final WPI_TalonSRX m_rightFollow = new WPI_TalonSRX(CanId.rightTalonFollow);

  private final MotorControllerGroup m_left = new MotorControllerGroup(m_leftLead, m_leftFollow); 
  private final MotorControllerGroup m_right = new MotorControllerGroup(m_rightLead, m_rightFollow);

  private final Encoder m_leftEncoder = new Encoder(Encoders.leftAPort, Encoders.leftBPort);
  private final Encoder m_rightEncoder = new Encoder(Encoders.rightAPort, Encoders.rightBPort);
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
  private PhotonCamera m_aprilTagCamera = new PhotonCamera("AprilTagCam");

  private DifferentialDrivePoseEstimator m_poseEstimator;
  public SimpleMotorFeedforward m_lFeedforward = new SimpleMotorFeedforward(Feedforward.Left.kS, Feedforward.Left.kV, Feedforward.Left.kA);
  private SimpleMotorFeedforward m_rFeedforward = new SimpleMotorFeedforward(Feedforward.Right.kS, Feedforward.Right.kV, Feedforward.Right.kA);
  private PIDController m_leftPIDs = new PIDController(PIDs.Left.kP, PIDs.Left.kI, PIDs.Left.kD);
  private PIDController m_rightPIDs = new PIDController(PIDs.Right.kP, PIDs.Right.kI, PIDs.Right.kD);
  public final DifferentialDriveKinematics m_driveKinematics = new DifferentialDriveKinematics(Dimensions.trackWidthMeters);
  private RamseteController m_ramseteController = new RamseteController();

  //TODO Fix this bullshit up
  private AprilTagFieldLayout aprilTagFieldLayout;
  private List<Pair<PhotonCamera, Transform3d>> camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
  private RobotPoseEstimator robotPoseEstimator;

  private ShuffleboardTab m_SBTab = Shuffleboard.getTab("Pose Estimation");
  private ShuffleboardLayout m_SBSensors;
  private Field2d m_robotField2d = new Field2d();

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    m_left.setInverted(!Dimensions.kInvertDrive);
    m_right.setInverted(Dimensions.kInvertDrive);
    m_leftEncoder.setDistancePerPulse(Dimensions.wheelCircumferenceMeters/Encoders.PPR);
    m_rightEncoder.setDistancePerPulse(Dimensions.wheelCircumferenceMeters/Encoders.PPR);

    //TODO and this garb
    //TODO clean up this garbage
    try{
      aprilTagFieldLayout = new AprilTagFieldLayout(new File(Filesystem.getDeployDirectory(), "HallLayout.json").toPath());
    }
    catch(Exception e){
      System.out.println("Failed to load AprilTag Layout");
    }
    camList.add(new Pair<PhotonCamera, Transform3d>(m_aprilTagCamera, Dimensions.aprilTagCameraPositionTransform));
    robotPoseEstimator = new RobotPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camList);

    m_poseEstimator = new DifferentialDrivePoseEstimator(
      m_driveKinematics,
      new Rotation2d(getAngle()),
      getLeftDistance(), getRightDistance(),
      new Pose2d(2.1, 1, new Rotation2d(1.57)),
      new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), // Local measurement standard deviations. Left encoder, right encoder, gyro.
      new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.5)); // Global measurement standard deviations. X, Y, and theta.

    shuffleBoardInit();
  }

  //Enable or disable brake mode on the motors
  public void brakeMode(boolean mode){
    NeutralMode nMode = NeutralMode.Coast;
    if (mode) nMode = NeutralMode.Brake;

    m_leftLead.setNeutralMode(nMode);
    m_leftFollow.setNeutralMode(nMode);
    m_rightLead.setNeutralMode(nMode);
    m_leftFollow.setNeutralMode(nMode);
  }

  //Simple arcade drive that uses a percentage (-1.00 to 1.00) of the max forward and angular speeds to drive the chassis at
  public void arcadeDrive(double linearPercent, double angularPercent){
    linearPercent = MathUtil.clamp(linearPercent, -1, 1);
    angularPercent = MathUtil.clamp(angularPercent, -1, 1);

    double maxAngularSpeed = m_driveKinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(Rate.maxSpeed, Rate.maxSpeed)).omegaRadiansPerSecond;
    driveChassisSpeeds(new ChassisSpeeds(Rate.maxSpeed * linearPercent, 0, maxAngularSpeed * angularPercent));
  }

  //Simple tank drive that uses a percentage (-1.00 to 1.00) of the max left and right speeds to drive the wheels at
  public void tankDrive(double leftPercent, double rightPercent){
    leftPercent = MathUtil.clamp(leftPercent, -1, 1);
    rightPercent = MathUtil.clamp(rightPercent, -1 , 1);

    driveWheelSpeeds(new DifferentialDriveWheelSpeeds(Rate.maxSpeed * leftPercent, Rate.maxSpeed * rightPercent));
  }

  //Set the appropriate motor voltages for a desired set of wheel speeds
  public void driveWheelSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds){
    m_left.setVoltage(m_lFeedforward.calculate(wheelSpeeds.leftMetersPerSecond));
    m_right.setVoltage(m_rFeedforward.calculate(wheelSpeeds.rightMetersPerSecond));
  }

  //Set the appropriate motor voltages for a desired set of linear and angular chassis speeds
  public void driveChassisSpeeds(ChassisSpeeds chassisSpeeds){
    driveWheelSpeeds(m_driveKinematics.toWheelSpeeds(chassisSpeeds));
  }

  //Drive the motors at a given voltage
  public void driveVoltages(double leftVoltage, double rightVoltage){
    m_left.setVoltage(leftVoltage);
    m_right.setVoltage(rightVoltage);
  }

  //PID Control
  public void FeedforwardPIDControl(double leftVelocitySetpoint, double rightVelocitySetpoint){
    m_left.setVoltage(m_leftPIDs.calculate(m_leftEncoder.getRate(), leftVelocitySetpoint) + m_lFeedforward.calculate(leftVelocitySetpoint));
    m_right.setVoltage(m_rightPIDs.calculate(m_rightEncoder.getRate(),rightVelocitySetpoint) + m_rFeedforward.calculate(rightVelocitySetpoint));
  }

  //Utility function to map joystick input nonlinearly for driver "feel"
  public static double NonLinear(double input){ return Math.copySign(input * input, input);}

  public double getLeftDistance(){
    return m_leftEncoder.getDistance();
  }

  public double getRightDistance(){
    return m_rightEncoder.getDistance();
  }

  public double getLeftVelocity(){
    return m_rightEncoder.getRate();
  }

  public double getRightVelocity(){
    return m_leftEncoder.getRate();
  }

  public double getAngle(){
    return Units.degreesToRadians(-m_gyro.getYaw());
  }

  public Pose2d getPose(){
    return m_poseEstimator.getEstimatedPosition();
  }

  public Command followPath(Trajectory path){
    return new RamseteCommand(path, this::getPose, m_ramseteController, m_driveKinematics, this::FeedforwardPIDControl, this);
  }

  private void shuffleBoardInit(){
    m_SBSensors = m_SBTab.getLayout("Sensors", BuiltInLayouts.kList)
    .withSize(2,4)
    .withPosition(0, 0);
    m_SBSensors.add("NavX2", m_gyro).withWidget(BuiltInWidgets.kGyro);
    m_SBSensors.add("Left Encoder", m_leftEncoder).withWidget(BuiltInWidgets.kEncoder);
    m_SBSensors.add("Right Encoder", m_rightEncoder).withWidget(BuiltInWidgets.kEncoder);
    m_SBTab.add("Pose Estimate", m_robotField2d).withWidget(BuiltInWidgets.kField)
      .withSize(7, 4)
      .withPosition(2, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double time = Timer.getFPGATimestamp();
    m_poseEstimator.updateWithTime(
      time,
      new Rotation2d(getAngle()),
      getLeftDistance(), 
      getRightDistance());
    m_robotField2d.setRobotPose(m_poseEstimator.getEstimatedPosition());

    robotPoseEstimator.setReferencePose(m_poseEstimator.getEstimatedPosition());
    Optional<Pair<Pose3d, Double>> result = robotPoseEstimator.update();
    if (result.isPresent()){
      if(time - (Timer.getFPGATimestamp() - result.get().getSecond()/1000) > 0){
      //m_poseEstimator.addVisionMeasurement(
      //  result.get().getFirst().toPose2d(),
      //  Timer.getFPGATimestamp() - result.get().getSecond()/1000);
      }
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
