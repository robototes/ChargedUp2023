package frc.team2412.robot.util.auto;

import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.function.DoubleSupplier;

public class AutonomousField {
	private static Pose2d getHolonomicPose(Trajectory.State state) {
		if (state instanceof PathPlannerState) {
			return getHolonomicPose((PathPlannerState) state);
		}
		return state.poseMeters;
	}

	private static Pose2d getHolonomicPose(PathPlannerState state) {
		return new Pose2d(state.poseMeters.getTranslation(), state.holonomicRotation);
	}

	private static class Data {
		public Object key;
		public double speed;

		public Data(Object key, double speed) {
			this.key = key;
			this.speed = speed;
		}

		@Override
		public boolean equals(Object obj) {
			if (obj instanceof Data) {
				Data data = (Data) obj;
				return Objects.equals(key, data.key) && speed == data.speed;
			}
			return false;
		}

		@Override
		public int hashCode() {
			return Objects.hash(key, speed);
		}
	}

	private final Field2d field = new Field2d();
	private final DoubleSupplier speedMultiplier;
	/** Timer storing the time elapsed on the current trajectory. */
	private final Timer timer = new Timer();
	/**
	 * Key to use to determine if the trajectory changed. Empty for the first update, otherwise
	 * contains the key to use.
	 */
	private Optional<Data> lastData = Optional.empty();

	private int trajectoryIndex = 0;

	public AutonomousField() {
		this(() -> 1);
	}

	public AutonomousField(double speedMultiplier) {
		this(() -> speedMultiplier);
	}

	public AutonomousField(DoubleSupplier speedMultiplier) {
		this.speedMultiplier = speedMultiplier;
	}

	/**
	 * Updates the {@link Field2d} robot pose.
	 *
	 * @param key The key associated with the trajectories. If the key passed does not compare equal
	 *     to the last key passed (according to {@link Objects#equals}), the robot pose will go to the
	 *     start of the new path.
	 * @param trajectories The list of trajectories representing the auto path.
	 */
	public void update(Object key, List<? extends Trajectory> trajectories) {
		double speed = speedMultiplier.getAsDouble();
		Data newData = new Data(key, speed);
		if (lastData.isEmpty() || !lastData.get().equals(newData)) {
			lastData = Optional.of(newData);
			trajectoryIndex = 0;
			timer.restart();
		}
		while (timer.advanceIfElapsed(
				trajectories.get(trajectoryIndex).getTotalTimeSeconds() / speed)) {
			trajectoryIndex++;
			if (trajectoryIndex >= trajectories.size()) {
				trajectoryIndex = 0;
			}
		}
		field.setRobotPose(
				getHolonomicPose(trajectories.get(trajectoryIndex).sample(timer.get() * speed)));
	}

	public Field2d getField() {
		return field;
	}
}
