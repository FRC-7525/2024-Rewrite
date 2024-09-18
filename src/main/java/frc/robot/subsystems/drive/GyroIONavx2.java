package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import java.util.OptionalDouble;
import java.util.Queue;

/** IO implementation for NavX-MXP */
public class GyroIONavx2 implements GyroIO {

	private final AHRS navx;
	private final Queue<Double> yawPositionQueue;
	private final Queue<Double> yawTimestampQueue;

	public GyroIONavx2() {
		navx = new AHRS(SerialPort.Port.kUSB1);
		navx.reset();
		yawTimestampQueue = HybridOdometryThread.getInstance().makeTimestampQueue();
		yawPositionQueue = HybridOdometryThread.getInstance()
			.registerSignal(() -> {
				if (navx.isConnected()) {
					return OptionalDouble.of(navx.getYaw());
				} else {
					return OptionalDouble.empty();
				}
			});
	}

	@Override
	public void updateInputs(GyroIOInputs inputs) {
		inputs.connected = navx.isConnected();
		inputs.yawPosition = Rotation2d.fromDegrees(navx.getYaw());
		inputs.yawVelocityRadPerSec = Units.degreesToRadians(navx.getRate());

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
