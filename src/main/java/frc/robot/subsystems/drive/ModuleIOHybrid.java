package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import java.util.OptionalDouble;
import java.util.Queue;

/**
 * Module IO implementation for Talon FX drive motor controller, SparkMax turn motor controller, and
 * CANcoder
 */
public class ModuleIOHybrid implements ModuleIO {

	private final TalonFX driveTalon;
	private final CANSparkMax turnSparkMax;
	private final CANcoder cancoder;

	private final Queue<Double> timestampQueue;

	private final StatusSignal<Double> drivePosition;
	private final Queue<Double> drivePositionQueue;
	private final StatusSignal<Double> driveVelocity;
	private final StatusSignal<Double> driveAppliedVolts;
	private final StatusSignal<Double> driveCurrent;

	private final RelativeEncoder turnRelativeEncoder;
	private final Queue<Double> turnPositionQueue;

	private final boolean isTurnMotorInverted = true;
	private final Rotation2d absoluteEncoderOffset;

	public ModuleIOHybrid(int index) {
		switch (index) {
			case 0:
				driveTalon = new TalonFX(Constants.Drive.Module.Hybrid.DRIVE0_ID);
				turnSparkMax = new CANSparkMax(
					Constants.Drive.Module.Hybrid.TURN0_ID,
					MotorType.kBrushless
				);
				cancoder = new CANcoder(Constants.Drive.Module.Hybrid.CANCODER0_ID);
				absoluteEncoderOffset = new Rotation2d(Constants.Drive.Module.Hybrid.OFFSET0);
				break;
			case 1:
				driveTalon = new TalonFX(Constants.Drive.Module.Hybrid.DRIVE1_ID);
				turnSparkMax = new CANSparkMax(
					Constants.Drive.Module.Hybrid.TURN1_ID,
					MotorType.kBrushless
				);
				cancoder = new CANcoder(Constants.Drive.Module.Hybrid.CANCODER1_ID);
				absoluteEncoderOffset = new Rotation2d(Constants.Drive.Module.Hybrid.OFFSET1);
				break;
			case 2:
				driveTalon = new TalonFX(Constants.Drive.Module.Hybrid.DRIVE2_ID);
				turnSparkMax = new CANSparkMax(
					Constants.Drive.Module.Hybrid.TURN2_ID,
					MotorType.kBrushless
				);
				cancoder = new CANcoder(Constants.Drive.Module.Hybrid.CANCODER2_ID);
				absoluteEncoderOffset = new Rotation2d(Constants.Drive.Module.Hybrid.OFFSET2);
				break;
			case 3:
				driveTalon = new TalonFX(Constants.Drive.Module.Hybrid.DRIVE3_ID);
				turnSparkMax = new CANSparkMax(
					Constants.Drive.Module.Hybrid.TURN3_ID,
					MotorType.kBrushless
				);
				cancoder = new CANcoder(Constants.Drive.Module.Hybrid.CANCODER3_ID);
				absoluteEncoderOffset = new Rotation2d(Constants.Drive.Module.Hybrid.OFFSET3);
				break;
			default:
				throw new RuntimeException("Invalid module index");
		}

		// TalonFX configuration
		var driveConfig = new TalonFXConfiguration();
		driveConfig.CurrentLimits.SupplyCurrentLimit =
			Constants.Drive.Module.Hybrid.DRIVE_CURRENT_LIMIT;
		driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		driveTalon.getConfigurator().apply(driveConfig);
		setDriveBrakeMode(true);

		cancoder.getConfigurator().apply(new CANcoderConfiguration());

		timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

		drivePosition = driveTalon.getPosition();
		drivePositionQueue = PhoenixOdometryThread.getInstance()
			.registerSignal(driveTalon, driveTalon.getPosition());
		driveVelocity = driveTalon.getVelocity();
		driveAppliedVolts = driveTalon.getMotorVoltage();
		driveCurrent = driveTalon.getSupplyCurrent();

		BaseStatusSignal.setUpdateFrequencyForAll(
			Constants.Drive.Module.ODOMETRY_FREQUENCY,
			drivePosition
		);
		BaseStatusSignal.setUpdateFrequencyForAll(
			Constants.Drive.Module.Hybrid.TALON_UPDATE_FREQUENCY_HZ,
			driveVelocity,
			driveAppliedVolts,
			driveCurrent
		);
		driveTalon.optimizeBusUtilization();

		// SparkMax configuration
		turnSparkMax.restoreFactoryDefaults();
		turnSparkMax.setCANTimeout(Constants.Drive.Module.Hybrid.SPARK_TIMEOUT_MS);
		turnRelativeEncoder = turnSparkMax.getEncoder();

		turnSparkMax.setInverted(isTurnMotorInverted);
		turnSparkMax.setSmartCurrentLimit(Constants.Drive.Module.Hybrid.TURN_CURRENT_LIMIT);
		turnSparkMax.enableVoltageCompensation(Constants.MAX_VOLTS);

		turnRelativeEncoder.setPosition(0.0);
		turnRelativeEncoder.setMeasurementPeriod(
			Constants.Drive.Module.Hybrid.SPARK_MEASURMENT_PERIOD_MS
		);
		turnRelativeEncoder.setAverageDepth(Constants.Drive.Module.Hybrid.SPARK_AVG_DEPTH);

		turnSparkMax.setCANTimeout(0);
		turnSparkMax.setPeriodicFramePeriod(
			PeriodicFrame.kStatus2,
			(int) (Constants.Drive.Module.Hybrid.SPARK_FRAME_PERIOD)
		);
		turnPositionQueue = SparkMaxOdometryThread.getInstance()
			.registerSignal(() -> {
				double value = turnRelativeEncoder.getPosition();
				if (turnSparkMax.getLastError() == REVLibError.kOk) {
					return OptionalDouble.of(value);
				} else {
					return OptionalDouble.of(0);
				}
			});

		turnSparkMax.burnFlash();
	}

	@Override
	public void updateInputs(ModuleIOInputs inputs) {
		BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);

		// Turn Stuff
		inputs.turnAbsolutePosition = Rotation2d.fromRotations(
			cancoder.getAbsolutePosition().getValueAsDouble()
		).minus(absoluteEncoderOffset);
		inputs.turnPosition = Rotation2d.fromRotations(
			turnRelativeEncoder.getPosition() / Constants.Drive.Module.Hybrid.TURN_GEAR_RATIO
		);
		inputs.turnVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
			turnRelativeEncoder.getVelocity()
		) /
		Constants.Drive.Module.Hybrid.TURN_GEAR_RATIO;
		inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
		inputs.turnCurrentAmps = new double[] { turnSparkMax.getOutputCurrent() };

		// Other stuff
		inputs.odometryTimestamps = timestampQueue
			.stream()
			.mapToDouble((Double value) -> value)
			.toArray();
		inputs.odometryDrivePositionsRad = drivePositionQueue
			.stream()
			.mapToDouble(
				(Double value) ->
					Units.rotationsToRadians(value) / Constants.Drive.Module.Hybrid.DRIVE_GEAR_RATIO
			)
			.toArray();
		inputs.odometryTurnPositions = turnPositionQueue
			.stream()
			.map((Double value) ->
				Rotation2d.fromRotations(value / Constants.Drive.Module.Hybrid.TURN_GEAR_RATIO)
			)
			.toArray(Rotation2d[]::new);
		timestampQueue.clear();
		drivePositionQueue.clear();
		turnPositionQueue.clear();
	}

	@Override
	public void setDriveVoltage(double volts) {
		driveTalon.setControl(new VoltageOut(volts));
	}

	@Override
	public void setTurnVoltage(double volts) {
		turnSparkMax.setVoltage(volts);
	}

	@Override
	public void setDriveBrakeMode(boolean enable) {
		var config = new MotorOutputConfigs();
		config.Inverted = InvertedValue.CounterClockwise_Positive;
		config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
		driveTalon.getConfigurator().apply(config);
	}

	public void setTurnBrakeMode(boolean enable) {
		turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
	}
}
