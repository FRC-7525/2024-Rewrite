package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;
import java.util.LinkedList;
import java.util.Queue;

public class GyroIONavx2 implements GyroIO {

	// I forgot what we plugged the NavX into :(
	private final AHRS navx;
	private final Queue<Double> yawPositionQueue = new LinkedList<>();
	private final Queue<Double> yawTimestampQueue = new LinkedList<>();

	public GyroIONavx2(SPI.Port port) {
		navx = new AHRS(port);
		navx.reset();
	}

	public void updateInputs(GyroIOInputs inputs) {
		inputs.connected = navx.isConnected();
		inputs.yawPosition = Rotation2d.fromDegrees(navx.getYaw());
		inputs.yawVelocityRadPerSec = Units.degreesToRadians(navx.getRate());

		yawTimestampQueue.add(
			(double) System.currentTimeMillis() / Constants.Drive.Navx2.UPDATE_FREQUENCY
		); // Time in seconds
		yawPositionQueue.add((double) navx.getYaw());

		inputs.odometryYawTimestamps = yawTimestampQueue
			.stream()
			.mapToDouble(Double::doubleValue)
			.toArray();
		inputs.odometryYawPositions = yawPositionQueue
			.stream()
			.map(Rotation2d::fromDegrees)
			.toArray(Rotation2d[]::new);
		yawTimestampQueue.clear();
		yawPositionQueue.clear();
	}
}
