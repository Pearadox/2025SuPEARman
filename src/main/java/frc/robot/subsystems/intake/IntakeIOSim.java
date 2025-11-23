package frc.robot.subsystems.intake;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOSim extends IntakeIOTalonFX {
    private final SingleJointedArmSim pivotPhysicsSim = new SingleJointedArmSim(
            IntakeConstants.PIVOT_MOTOR,
            IntakeConstants.PIVOT_GEAR_RATIO,
            SingleJointedArmSim.estimateMOI(IntakeConstants.PIVOT_LENGTH, IntakeConstants.PIVOT_MASS),
            IntakeConstants.PIVOT_LENGTH,
            IntakeConstants.PIVOT_MIN_ANGLE,
            IntakeConstants.PIVOT_MAX_ANGLE,
            true,
            IntakeConstants.PIVOT_STARTING_ANGLE);

    private final SingleJointedArmSim rollerPhysicsSim = new SingleJointedArmSim(
            IntakeConstants.ROLLER_MOTOR,
            IntakeConstants.ROLLER_GEAR_RATIO,
            IntakeConstants.ROLLER_MOI,
            IntakeConstants.ROLLER_RADIUS,
            Double.NEGATIVE_INFINITY,
            Double.POSITIVE_INFINITY,
            false,
            0);

    private final TalonFXSimState pivotSimState;
    private final TalonFXSimState rollerSimState;

    public IntakeIOSim() {
        pivotSimState = pivotMotor.getSimState();
        rollerSimState = rollerMotor.getSimState();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        super.updateInputs(inputs);

        pivotSimState.setSupplyVoltage(Constants.NOMINAL_VOLTAGE);
        rollerSimState.setSupplyVoltage(Constants.NOMINAL_VOLTAGE);

        pivotPhysicsSim.setInputVoltage(pivotSimState.getMotorVoltage());
        rollerPhysicsSim.setInputVoltage(rollerSimState.getMotorVoltage());

        pivotPhysicsSim.update(Constants.LOOP_PERIOD);
        rollerPhysicsSim.update(Constants.LOOP_PERIOD);

        pivotSimState.setRawRotorPosition((pivotPhysicsSim.getAngleRads() - IntakeConstants.PIVOT_STARTING_ANGLE)
                / IntakeConstants.PIVOT_P_COEFFICIENT);
        rollerSimState.setRawRotorPosition(rollerPhysicsSim.getAngleRads() / IntakeConstants.ROLLER_P_COEFFICIENT);

        pivotSimState.setRotorVelocity(pivotPhysicsSim.getVelocityRadPerSec() / IntakeConstants.PIVOT_P_COEFFICIENT);
        rollerSimState.setRotorVelocity(rollerPhysicsSim.getVelocityRadPerSec() / IntakeConstants.ROLLER_P_COEFFICIENT);
    }
}
