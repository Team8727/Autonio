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


import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTag;

import java.util.ArrayList;
import edu.wpi.first.math.Pair;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.wpilibj.SerialPort.Port;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Encoder;

import frc.robot.Constants.Drivetrain.Dimensions;
import frc.robot.Constants.Drivetrain.Encoders;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;


public class Drivetrain extends SubsystemBase {
  private DifferentialDrivePoseEstimator m_poseEstimator;
  private DifferentialDriveKinematics m_driveKinematics= new DifferentialDriveKinematics(Dimensions.trackWidthMeters);
  private Encoder m_leftEncoder = new Encoder(Encoders.leftAPort, Encoders.leftBPort);
  private Encoder m_rightEncoder = new Encoder(Encoders.rightAPort, Encoders.rightBPort);
  private AHRS m_gyro = new AHRS(Port.kMXP);
  private PhotonCamera m_aprilTagCamera = new PhotonCamera("photonvision");

  //TODO Fix this bullshit up
  private List<AprilTag> tags = new ArrayList<AprilTag>();
  private AprilTagFieldLayout aprilTagFieldLayout;
  private List<Pair<PhotonCamera, Transform3d>> camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
  private RobotPoseEstimator robotPoseEstimator;

  private ShuffleboardTab m_SBTab = Shuffleboard.getTab("Pose Estimation");
  private ShuffleboardLayout m_SBSensors;
  private Field2d m_robotField2d = new Field2d();

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    m_leftEncoder.setDistancePerPulse(Dimensions.wheelCircumferenceMeters/Encoders.PPR);
    m_rightEncoder.setDistancePerPulse(Dimensions.wheelCircumferenceMeters/Encoders.PPR);

    //TODO and this garb
    tags.add(new AprilTag(0, new Pose3d(new Pose2d(0,0, new Rotation2d()))));
    aprilTagFieldLayout = new AprilTagFieldLayout(tags, 10, 10);
    camList.add(new Pair<PhotonCamera, Transform3d>(m_aprilTagCamera, Dimensions.aprilTagCameraPositionTransform));
    robotPoseEstimator = new RobotPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camList);

    m_poseEstimator = new DifferentialDrivePoseEstimator(
      m_driveKinematics,
      new Rotation2d(getAngle()),
      getLeftDistance(), getRightDistance(),
      new Pose2d(3, 3, new Rotation2d(0)),
      new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), // Local measurement standard deviations. Left encoder, right encoder, gyro.
      new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01)); // Global measurement standard deviations. X, Y, and theta.

    shuffleBoardInit();
  }

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
    m_poseEstimator.updateWithTime(
      Timer.getFPGATimestamp(),
      new Rotation2d(getAngle()),
      getLeftDistance(), 
      getRightDistance());
    m_robotField2d.setRobotPose(m_poseEstimator.getEstimatedPosition());

    robotPoseEstimator.setReferencePose(m_poseEstimator.getEstimatedPosition());
    Optional<Pair<Pose3d, Double>> result = robotPoseEstimator.update();
    if (result.isPresent()) {
      m_poseEstimator.addVisionMeasurement(
        result.get().getFirst().toPose2d(),
        Timer.getFPGATimestamp() - result.get().getSecond());
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
