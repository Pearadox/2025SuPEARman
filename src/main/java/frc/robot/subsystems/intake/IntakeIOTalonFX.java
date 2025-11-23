package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.PearadoxTalonFX;

public class IntakeIOTalonFX implements IntakeIO {
    protected final PearadoxTalonFX pivotMotor;
    protected final PearadoxTalonFX rollerMotor;

    protected final MotionMagicVoltage pivotMMRequest = new MotionMagicVoltage(0);

    protected IntakeIOTalonFX() {
        pivotMotor = new PearadoxTalonFX(IntakeConstants.PIVOT_ID, IntakeConstants.getPivotConfig());
        rollerMotor = new PearadoxTalonFX(IntakeConstants.ROLLER_ID, IntakeConstants.getRollerConfig());
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.pivotData = pivotMotor.getData();
        inputs.rollerData = rollerMotor.getData();
    }

    @Override
    public void runPivotPosition(double setpointRots) {
        pivotMotor.setControl(pivotMMRequest.withPosition(setpointRots));
    }

    @Override
    public void runRollerVolts(double volts) {
        rollerMotor.setVoltage(volts);
    }
}
