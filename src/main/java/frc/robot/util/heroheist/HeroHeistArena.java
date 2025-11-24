package frc.robot.util.heroheist;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.FieldConstants;
import org.ironmaple.simulation.SimulatedArena;

public class HeroHeistArena extends SimulatedArena {
    public static final class HeroFieldObstaclesMap extends FieldMap {
        public HeroFieldObstaclesMap() {
            super();

            super.addBorderLine(new Translation2d(0, 0), new Translation2d(FieldConstants.FIELD_LENGTH, 0));
            super.addBorderLine(new Translation2d(0, 0), new Translation2d(0, FieldConstants.FIELD_WIDTH));
            super.addBorderLine(
                    new Translation2d(FieldConstants.FIELD_LENGTH, 0),
                    new Translation2d(FieldConstants.FIELD_LENGTH, FieldConstants.FIELD_WIDTH));
            super.addBorderLine(
                    new Translation2d(0, FieldConstants.FIELD_WIDTH),
                    new Translation2d(FieldConstants.FIELD_LENGTH, FieldConstants.FIELD_WIDTH));
        }
    }

    public HeroHeistArena() {
        super(new HeroFieldObstaclesMap());
    }

    private static final Translation2d[] NOTE_INITIAL_POSITIONS = new Translation2d[] {
        new Translation2d(2.9, 4.1),
        new Translation2d(2.9, 5.55),
        new Translation2d(2.9, 7),
        new Translation2d(8.27, 0.75),
        new Translation2d(8.27, 2.43),
        new Translation2d(8.27, 4.1),
        new Translation2d(8.27, 5.78),
        new Translation2d(8.27, 7.46),
        new Translation2d(13.64, 4.1),
        new Translation2d(13.64, 5.55),
        new Translation2d(13.64, 7),
    };

    @Override
    public void placeGamePiecesOnField() {
        for (Translation2d notePosition : NOTE_INITIAL_POSITIONS)
            super.addGamePiece(new SpeechBubbleOnField(notePosition));
    }
}
