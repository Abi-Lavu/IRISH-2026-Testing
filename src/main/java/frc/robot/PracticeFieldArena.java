package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Simulation only: our practice field (from "Full Field Assembly" in Onshape) for the maple-sim
 * physics engine. One hub in the middle, the bump next to it (solid: our robot can't cross it), the
 * trench on the other side, and 35 balls in each end's depot.
 *
 * Coordinates are WPILib's: meters from the blue corner, X toward red, Y to the left of the blue
 * drivers. The field is 16.513 m x 8.035 m inside the walls.
 */
public class PracticeFieldArena extends SimulatedArena {
    public static final double FIELD_LENGTH = 16.513;
    public static final double FIELD_WIDTH = 8.0345;
    private static final Translation2d CENTER = new Translation2d(FIELD_LENGTH / 2, FIELD_WIDTH / 2);

    private static final double HUB_SIZE = 1.1938;
    // Trench: the robot drives under the arm, but the two bases at its ends are solid.
    private static final double TRENCH_BASE_X = 1.1938;
    private static final double TRENCH_BASE_Y = 0.3048;
    private static final double TRENCH_BASE_NEAR_Y = CENTER.getY() - 0.7493;
    private static final double TRENCH_BASE_FAR_Y = CENTER.getY() - 3.8681;

    // Bump: our robot beaches on its 6.5 in ridge, so it's solid here (we always use the trench).
    private static final double BUMP_X = 1.1276;
    private static final double BUMP_Y = 3.429;
    private static final double BUMP_CENTER_Y = CENTER.getY() + 2.3114;

    // Depots: 5 x 7 grid of balls at each end, 0.15 m apart (from the CAD).
    private static final double BALL_SPACING = 0.15;
    private static final double BLUE_DEPOT_X = 0.081;
    private static final double RED_DEPOT_X = 15.832;
    private static final double DEPOT_Y = 3.567;

    private final boolean ballsEnabled;

    /**
     * @param fieldCollisions true: the hub, bump and trench are solid. false: robot and balls pass
     *     through them. The outside walls are always solid.
     * @param ballsEnabled true: balls are placed on the field and roll when hit. false: no balls.
     */
    public PracticeFieldArena(boolean fieldCollisions, boolean ballsEnabled) {
        super(new PracticeFieldMap(fieldCollisions));
        this.ballsEnabled = ballsEnabled;
    }

    @Override
    public void placeGamePiecesOnField() {
        if (!ballsEnabled) {
            return;
        }
        for (double depotX : new double[] {BLUE_DEPOT_X, RED_DEPOT_X}) {
            for (int col = 0; col < 5; col++) {
                for (int row = 0; row < 7; row++) {
                    addGamePiece(new RebuiltFuelOnField(new Translation2d(
                        depotX + col * BALL_SPACING, DEPOT_Y + row * BALL_SPACING)));
                }
            }
        }
    }

    /** The solid parts of the field. */
    private static class PracticeFieldMap extends FieldMap {
        PracticeFieldMap(boolean fieldCollisions) {
            // Outside walls.
            addBorderLine(new Translation2d(0, 0), new Translation2d(FIELD_LENGTH, 0));
            addBorderLine(new Translation2d(0, FIELD_WIDTH), new Translation2d(FIELD_LENGTH, FIELD_WIDTH));
            addBorderLine(new Translation2d(0, 0), new Translation2d(0, FIELD_WIDTH));
            addBorderLine(new Translation2d(FIELD_LENGTH, 0), new Translation2d(FIELD_LENGTH, FIELD_WIDTH));

            if (!fieldCollisions) {
                return;
            }
            addRectangularObstacle(HUB_SIZE, HUB_SIZE, new Pose2d(CENTER, Rotation2d.kZero));
            addRectangularObstacle(TRENCH_BASE_X, TRENCH_BASE_Y,
                new Pose2d(CENTER.getX(), TRENCH_BASE_NEAR_Y, Rotation2d.kZero));
            addRectangularObstacle(TRENCH_BASE_X, TRENCH_BASE_Y,
                new Pose2d(CENTER.getX(), TRENCH_BASE_FAR_Y, Rotation2d.kZero));
            addRectangularObstacle(BUMP_X, BUMP_Y, new Pose2d(CENTER.getX(), BUMP_CENTER_Y, Rotation2d.kZero));
        }
    }
}
