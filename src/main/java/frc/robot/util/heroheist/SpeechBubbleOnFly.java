package frc.robot.util.heroheist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;

public class SpeechBubbleOnFly extends GamePieceProjectile {
    public SpeechBubbleOnFly(
            Translation2d robotPosition,
            Translation2d shooterPositionOnRobot,
            ChassisSpeeds chassisSpeeds,
            Rotation2d shooterFacing,
            Distance initialHeight,
            LinearVelocity launchingSpeed,
            Angle shooterAngle) {
        super(
                SpeechBubbleOnField.SPEECH_BUBBLE_INFO,
                robotPosition,
                shooterPositionOnRobot,
                chassisSpeeds,
                shooterFacing,
                initialHeight,
                launchingSpeed,
                shooterAngle);

        super.withTouchGroundHeight(0.8); // seems high?
        super.enableBecomesGamePieceOnFieldAfterTouchGround();
    }
}
