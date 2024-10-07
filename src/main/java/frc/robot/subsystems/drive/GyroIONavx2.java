package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort;
import java.util.OptionalDouble;
import java.util.Queue;

/** IO implementation for NavX-MXP */
public class GyroIONavx2 implements GyroIO {

	private final AHRS navx;
	private final Queue<Double> yawPositionQueue;
	private final Queue<Double> yawTimestampQueue;

	private Rotation3d offset;

	public GyroIONavx2() {
		navx = new AHRS(SerialPort.Port.kUSB1);
		navx.reset();
		offset = new Rotation3d();

		yawTimestampQueue = HybridOdometryThread.getInstance().makeTimestampQueue();
		yawPositionQueue = HybridOdometryThread.getInstance()
			.registerSignal(() -> {
				if (navx.isConnected()) {
					return OptionalDouble.of(navx.getRotation3d().minus(offset).getAngle());
				} else {
					return OptionalDouble.empty();
				}
			});
	}

	@Override
	public void zero() {
		this.offset = navx.getRotation3d();
	}

	@Override
	public void updateInputs(GyroIOInputs inputs) {
		inputs.connected = navx.isConnected();
		inputs.yawPosition = Rotation2d.fromRadians(navx.getRotation3d().minus(offset).getAngle());
		inputs.yawVelocityRadPerSec = Units.degreesToRadians(navx.getRate());
		inputs.yawPosDeg = Units.radiansToDegrees(navx.getRotation3d().minus(offset).getAngle());

		if (yawTimestampQueue != null && yawPositionQueue != null) {
			inputs.odometryYawTimestamps = yawTimestampQueue
				.stream()
				.mapToDouble((Double value) -> value)
				.toArray();
			inputs.odometryYawPositions = yawPositionQueue
				.stream()
				.map((Double value) -> Rotation2d.fromDegrees(value))
				.toArray(Rotation2d[]::new);

			yawTimestampQueue.clear();
			yawPositionQueue.clear();
		}
	}
}
