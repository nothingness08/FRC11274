package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Data structure for a single path point.
 * @param pose The (X, Y) and direction of travel (velocity vector).
 * @param heading The direction the chassis should face.
 * @param velocity The speed to maintain through this point (m/s).
 * @param tolerance Distance from target (meters) to trigger the next waypoint.
 */
public record HolonomicWaypoint(
    Pose2d pose, 
    Rotation2d heading, 
    double velocity, 
    Pose2d tolerance
) {}