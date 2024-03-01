// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.chassis;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.IOException;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

/** Add your docs here. */
public class PoseEstimator {
  private final Chassis chassis;

  private final SwerveDrivePoseEstimator poseEstimator;
  private Transform2d vel;
  private Pose2d lastPose;
  private PhotonCamera ATCam1;
  private Pigeon2 pigeon;
  private PhotonPoseEstimator photonEstimator;
  private AprilTagFieldLayout aprilTagFieldLayout;

  private double lastEstTimestamp = 0;

  private final Matrix<N3, N1> singleTagStdDevs;
  private final Matrix<N3, N1> multiTagStdDevs;
  private final Transform3d robotToCam;

  public PoseEstimator(
      Chassis chassis,
      PhotonCamera ATCam1,
      Transform3d cam1Offset,
      int pigeonId,
      Matrix<N3, N1> stateStdDevs,
      Matrix<N3, N1> visionStdDevs,
      Matrix<N3, N1> singleTagStdDevs,
      Matrix<N3, N1> multiTagStdDevs) {
    this.ATCam1 = ATCam1;
    this.chassis = chassis;
    this.robotToCam = cam1Offset;

    this.singleTagStdDevs = singleTagStdDevs;
    this.multiTagStdDevs = multiTagStdDevs;

    pigeon = new Pigeon2(pigeonId);

    poseEstimator =
        new SwerveDrivePoseEstimator(
            chassis.kinematics,
            Rotation2d.fromDegrees(getGyroRot()),
            chassis.getPositions(),
            new Pose2d(),
            stateStdDevs,
            visionStdDevs);

    vel = new Transform2d();
    lastPose = new Pose2d();

    try {
      aprilTagFieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (IOException e) {
      System.err.println("Failed to load AprilTagFieldLayout");
    }
    photonEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, ATCam1, robotToCam);
  }

  public void update() {
    Optional<EstimatedRobotPose> estimatedPose = getEstimatedVisionPose();

    estimatedPose.ifPresent(
        est -> {
          var estPose = est.estimatedPose.toPose2d();
          // Change our trust in the measurement based on the tags we can see
          var estStdDevs = getEstimationStdDevs(estPose);

          poseEstimator.addVisionMeasurement(
              est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);

          Logger.recordOutput("AT Estimate", estimatedPose.get().estimatedPose.toPose2d());
        });

    poseEstimator.update(Rotation2d.fromDegrees(getGyroRot()), chassis.getPositions());
    Pose2d currentPose = poseEstimator.getEstimatedPosition();
    vel = currentPose.minus(lastPose); // Why is this robot relative?
    vel =
        new Transform2d(
            vel.getTranslation().rotateBy(currentPose.getRotation()), vel.getRotation());

    lastPose = currentPose;

    SmartDashboard.putNumber("gyroYaw", getGyroRot());
    Logger.recordOutput("Estimated Pose", currentPose);
    Logger.recordOutput("Estimated Velocity", getEstimatedVel());
  }

  public Optional<EstimatedRobotPose> getEstimatedVisionPose() {
    var visionEst = photonEstimator.update();
    double latestTimestamp = ATCam1.getLatestResult().getTimestampSeconds();
    boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;

    if (newResult) lastEstTimestamp = latestTimestamp;
    return visionEst;
  }

  public Pose2d getFusedPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Transform2d getEstimatedVel() {
    return vel.div(0.02); // How consistent is this update rate?
  }

  public PhotonPipelineResult getLatestResult() {
    return ATCam1.getLatestResult();
  }

  public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
    var estStdDevs = singleTagStdDevs;
    var targets = getLatestResult().getTargets();
    int numTags = 0;
    double avgDist = 0;
    for (var tgt : targets) {
      var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty()) continue;
      numTags++;
      avgDist +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }
    if (numTags == 0) return estStdDevs;
    avgDist /= numTags;
    // Decrease std devs if multiple targets are visible
    if (numTags > 1) estStdDevs = multiTagStdDevs;
    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 4)
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

    return estStdDevs;
  }

  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(Rotation2d.fromDegrees(getGyroRot()), chassis.getPositions(), pose);
  }

  public double getGyroRot() {
    return pigeon.getYaw().getValueAsDouble();
  }

  public void setGyroRot(double rot) {
    pigeon.setYaw(rot);
  }
}
