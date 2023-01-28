// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SPI;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  private final PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");

  DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(0.615);

  static AprilTagFieldLayout fieldLayout;


  static {
    try {
      fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
  }

  public final WPI_TalonFX leftLeader = new WPI_TalonFX(1);
	public final WPI_TalonFX leftFollower = new WPI_TalonFX(2);
	public final WPI_TalonFX rightLeader = new WPI_TalonFX(3);
	public final WPI_TalonFX rightFollower = new WPI_TalonFX(4);

  private final DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(KINEMATICS, new Rotation2d(), 0.0, 0.0, new Pose2d());
  private final PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(
			fieldLayout, PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY, camera,
			new Transform3d(new Translation3d(Units.inchesToMeters(16.5), 0, Units.inchesToMeters(25.5)), new Rotation3d())
	);

  private final NetworkTableInstance instance = NetworkTableInstance.getDefault();
  private final NetworkTable table = instance.getTable("Pose");
  private final DoublePublisher xPub = table.getDoubleTopic("X").publish();
  private final DoublePublisher yPub = table.getDoubleTopic("Y").publish();
  private final DoublePublisher anglePub = table.getDoubleTopic("angle").publish();

  @Override
  public void robotInit() {
    // poseEstimator.resetPosition(new Rotation2d(), 0, 0, new Pose2d(1.8, -0.4, new Rotation2d()));
    poseEstimator.update(new Rotation2d(), 0, 0);
    leftLeader.setNeutralMode(NeutralMode.Coast);
    leftFollower.setNeutralMode(NeutralMode.Coast);
    rightLeader.setNeutralMode(NeutralMode.Coast);
    rightFollower.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void robotPeriodic() {
    // poseEstimator.update(
		// 		new Rotation2d(Units.degreesToRadians(gyro.getAngle())),
		// 		leftLeader.getSelectedSensorPosition() * 1.941E-5,
		// 		rightLeader.getSelectedSensorPosition() * 1.941E-5
		// );

    var result = camera.getLatestResult();
    if (result.hasTargets()) {

      var target = result.getBestTarget();
      var trans = target.getBestCameraToTarget();
      var pose = (new Pose3d()).transformBy(trans.inverse()).toPose2d();

      System.out.println(pose.getX() + " " + pose.getY());

      poseEstimator.addVisionMeasurement(
        pose,
        result.getTimestampSeconds(),
        new Matrix<>(VecBuilder.fill(0, 0, 0))
      );
    }

    // var res = photonPoseEstimator.update();
    // res.ifPresent(pose -> {
    //   poseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(), camera.getLatestResult().getTimestampSeconds());
    // });
    

    xPub.set(poseEstimator.getEstimatedPosition().getX());
    yPub.set(8.01 - poseEstimator.getEstimatedPosition().getY());
    anglePub.set(poseEstimator.getEstimatedPosition().getRotation().getDegrees());
    System.out.println(poseEstimator.getEstimatedPosition().getX());
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
