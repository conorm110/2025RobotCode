// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public class Vision {
    private PhotonCamera m_Camera = new PhotonCamera("insertNameHere");
    private final Transform3d robotToCam = new Transform3d(new Translation3d(Distance.ofBaseUnits(0, Inches), Distance.ofBaseUnits(-6, Inches), Distance.ofBaseUnits(18, Inches)), new Rotation3d());
    private PhotonPoseEstimator m_PhotonPoseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);

    public Vision() {
        m_PhotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d previousEstimatedRobotPose) {
        m_PhotonPoseEstimator.setReferencePose(previousEstimatedRobotPose);

        List<PhotonPipelineResult> results = m_Camera.getAllUnreadResults();

        if( ! results.isEmpty()) {
            return m_PhotonPoseEstimator.update(results.get(results.size() - 1));
        } else {
            return null;
        }
    }
}
