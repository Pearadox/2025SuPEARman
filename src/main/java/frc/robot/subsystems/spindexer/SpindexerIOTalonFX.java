package frc.robot.subsystems.spindexer;

import frc.robot.Constants.SpindexerConstants;
import frc.robot.util.PearadoxTalonFX;

public abstract class SpindexerIOTalonFX implements SpindexerIO {
    protected final PearadoxTalonFX spindexerMotor;

    protected SpindexerIOTalonFX() {
        spindexerMotor = new PearadoxTalonFX(SpindexerConstants.SPINDEXER_ID, SpindexerConstants.getSpindexerConfig());
    }

    @Override
    public void updateInputs(SpindexerIOInputs inputs) {
        inputs.spindexerData = spindexerMotor.getData();
    }

    @Override
    public void runVolts(double volts) {
        spindexerMotor.setVoltage(volts);
    }
}
