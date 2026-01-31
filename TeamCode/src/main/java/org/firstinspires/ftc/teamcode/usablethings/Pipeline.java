package org.firstinspires.ftc.teamcode.usablethings;

/**
 * Enum for Limelight pipelines.
 */
public enum Pipeline {
    APRILTAGS_TARGET_BOTH(0),
    APRILTAG_TARGET_BLUE(2),
    APRILTAG_TARGET_RED(1),
    APRILTAG_MOTIF(3);

    public final int id;

    Pipeline(int id) {
        this.id = id;
    }
}