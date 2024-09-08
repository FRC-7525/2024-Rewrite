// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import java.util.OptionalDouble;
import java.util.Queue;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {

  private final Pigeon2 pigeon = new Pigeon2(Constants.Drive.Pidgeon2.DEVICE_ID);
  private final StatusSignal<Double> yaw = pigeon.getYaw();
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<Double> yawVelocity = pigeon.getAngularVelocityZWorld();

  public GyroIOPigeon2(boolean phoenixDrive) {
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(Constants.Drive.Module.ODOMETRY_FREQUENCY);
    yawVelocity.setUpdateFrequency(Constants.Drive.Pidgeon2.UPDATE_FREQUENCY);
    pigeon.optimizeBusUtilization();
    if (phoenixDrive) {
      yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
      yawPositionQueue =
          PhoenixOdometryThread.getInstance().registerSignal(pigeon, pigeon.getYaw());
    } else {
      yawTimestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
      yawPositionQueue =
          SparkMaxOdometryThread.getInstance()
              .registerSignal(
                  () -> {
                    boolean valid = yaw.refresh().getStatus().isOK();
                    if (valid) {
                      return OptionalDouble.of(yaw.getValueAsDouble());
                    } else {
                      return OptionalDouble.empty();
                    }
                  });
    }
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    // for (int x = 0; x < 4; x++) {
    //   System.out.println(inputs.odometryYawPositions[x]);
    //   System.out.println(x);
    // }
    // System.out.println("heyy");
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
