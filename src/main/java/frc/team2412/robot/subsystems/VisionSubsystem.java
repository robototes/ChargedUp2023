package frc.team2412.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Log;
import java.util.EnumSet;
import java.util.function.BiConsumer;

public class VisionSubsystem extends SubsystemBase {
	private BiConsumer<Pose2d, Double> poseConsumer;
	/** Null if no known robot pose, otherwise the last calculated robot pose from vision data. */
	private Pose3d robotPose = null;

	private DoubleArraySubscriber targetPose;

	public VisionSubsystem(BiConsumer<Pose2d, Double> poseConsumer) {
		this.poseConsumer = poseConsumer;
		var networkTables = NetworkTableInstance.getDefault();

		// // Connect to photonvision server
		// // Only in sim because normally photonvision connects to robot
		// if (RobotBase.isSimulation()) {
		// 	networkTables.stopServer();
		// 	networkTables.startClient4("localhost");
		// }

		this.targetPose =
				networkTables
						.getTable("limelight")
						.getDoubleArrayTopic("botpose_wpired") // TODO Get color from driverstation
						.subscribe(new double[] {});
		networkTables.addListener(
				targetPose, EnumSet.of(NetworkTableEvent.Kind.kValueAll), this::processEvent);
	}

	public void processEvent(NetworkTableEvent event) {
		// TODO Account for latency
		var time = Timer.getFPGATimestamp();
		double[] pose = targetPose.get();

		var rotation =
				new Rotation3d(Math.toRadians(pose[3]), Math.toRadians(pose[4]), Math.toRadians(pose[5]));
		robotPose = new Pose3d(pose[0], pose[1], pose[2], rotation);
		// TODO 3D odometry
		poseConsumer.accept(robotPose.toPose2d(), time);
	}

	@Log
	public boolean hasTargets() {
		return true;
	}

	/**
	 * Calculates the robot pose using the best target. Returns null if there is no known robot pose.
	 *
	 * @return The calculated robot pose in meters.
	 */
	public Pose3d getRobotPose() {
		return robotPose;
	}
}
