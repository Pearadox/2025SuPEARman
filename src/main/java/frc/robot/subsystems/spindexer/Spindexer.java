package frc.robot.subsystems.spindexer;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpindexerConstants;
import frc.robot.Constants.SpindexerConstants.SpindexerState;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Spindexer extends SubsystemBase {
    @AutoLogOutput
    @Getter
    @Setter
    private SpindexerState state = SpindexerState.SPAIN_WITHOUT_THE_IN;

    private SpindexerIO io;
    private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

    public Spindexer(SpindexerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Spindexer", inputs);

        io.runVolts(state.volts);
    }

    @AutoLogOutput
    public double getAngleRads() {
        return inputs.spindexerData.position() * SpindexerConstants.SPINDEXER_P_COEFFICIENT;
    }

    @AutoLogOutput
    public double getRelativeYawDegrees() {
        return Units.radiansToDegrees(getAngleRads()) % 360.0;
    }

    @AutoLogOutput
    public double getAngularVelocityRadsPerSec() {
        return inputs.spindexerData.velocity() * SpindexerConstants.SPINDEXER_P_COEFFICIENT;
    }
}
