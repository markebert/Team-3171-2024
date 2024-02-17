package frc.team3171.controllers;

// FRC Imports
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.HashMap;

// Photon Vision Imports
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

// Team 3171 Imports
import frc.robot.RobotProperties;
import frc.team3171.models.PhotonCameraConfig;

public class VisionController implements RobotProperties {

    // Photon Vision Objects
    private final HashMap<String, PhotonCamera> PHOTON_CAMERAS;

    public VisionController() {
        PHOTON_CAMERAS = new HashMap<>();

        // Photon Vision init
        PHOTON_CAMERAS_CONFIGS.forEach((photonCameraName, photonCameraConfig) -> {
            PHOTON_CAMERAS.put(photonCameraName, new PhotonCamera(photonCameraConfig.getCAMERA_NAME()));
        });

    }

    public PhotonTrackedTarget getCameraBestTarget(final String photonCameraName) {
        final PhotonCameraConfig photonCameraConfig = PHOTON_CAMERAS_CONFIGS.get(photonCameraName);
        final PhotonCamera photonCamera = PHOTON_CAMERAS.get(photonCameraName);

        // Check that the provided camera name returned an actual result
        if (photonCameraConfig == null || photonCamera == null) {
            System.err.printf("The provided camera name [%s] did not return a valid result!\nPlease verify that the camera was initialized correctly.\n",
                    photonCameraName);
            return null;
        }

        if (photonCamera.isConnected()) {
            PhotonPipelineResult result = photonCamera.getLatestResult();
            if (result.hasTargets()) {
                return result.getBestTarget();
            }
        }
        return null;

    }

    public void smartdashboard(final String photonCameraName) {
        final PhotonCameraConfig photonCameraConfig = PHOTON_CAMERAS_CONFIGS.get(photonCameraName);
        final PhotonCamera photonCamera = PHOTON_CAMERAS.get(photonCameraName);

        // Check that the provided camera name returned an actual result
        if (photonCameraConfig == null || photonCamera == null) {
            System.err.printf("The provided camera name [%s] did not return a valid result!\nPlease verify that the camera was initialized correctly.\n",
                    photonCameraName);
            return;
        }

        // Photon Vision Read Outs
        int targetID = 0;
        double range = -1, yaw = 0, pitch = 0;
        if (photonCamera.isConnected()) {
            PhotonPipelineResult result = photonCamera.getLatestResult();
            if (result.hasTargets()) {
                PhotonTrackedTarget bestTarget = result.getBestTarget();
                targetID = bestTarget.getFiducialId();
                final Double targetHeightMeters = APRILTAG_HEIGHTS_METERS.get(targetID);
                if (targetHeightMeters == null) {
                    System.err.printf(
                            "The AprilTag with ID [%s] did not return a valid target height!\nPlease verify that the target height was initialized correctly in 'APRILTAG_HEIGHTS_METERS' in RobotProperties.java.\n",
                            photonCameraName);
                } else {
                    range = PhotonUtils.calculateDistanceToTargetMeters(
                            photonCameraConfig.getCAMERA_HEIGHT_METERS(),
                            targetHeightMeters,
                            photonCameraConfig.getCAMERA_PITCH_RADIANS(),
                            Units.degreesToRadians(bestTarget.getPitch()));
                }
                yaw = bestTarget.getYaw();
            }
        }
        SmartDashboard.putBoolean(String.format("%s:", photonCameraName), photonCamera.isConnected());
        SmartDashboard.putString(String.format("%s Best Target:", photonCameraName),
                String.format("ID: %d | R: %.2f | Y: %.2f | P: %.2f", targetID, range, yaw, pitch));
    }
}