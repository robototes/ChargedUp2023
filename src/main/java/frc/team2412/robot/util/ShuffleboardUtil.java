package frc.team2412.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.function.ToDoubleFunction;

public class ShuffleboardUtil {
	public static <T> DoubleSupplier mapNullableSupplierToDouble(
			Supplier<T> supplier, ToDoubleFunction<T> mapper, double nullValue) {
		return () -> {
			T obj = supplier.get();
			return obj == null ? nullValue : mapper.applyAsDouble(obj);
		};
	}

	public static ShuffleboardLayout addPose3dLayout(
			ShuffleboardContainer container,
			String title,
			Supplier<Pose3d> poseSupplier,
			int columnIndex,
			int rowIndex) {
		ShuffleboardLayout layout =
				container
						.getLayout(title, BuiltInLayouts.kGrid)
						.withPosition(columnIndex, rowIndex)
						.withSize(2, 3);
		layout
				.addDouble("X", mapNullableSupplierToDouble(poseSupplier, pose -> pose.getX(), -1))
				.withPosition(0, 0);
		layout
				.addDouble("Y", mapNullableSupplierToDouble(poseSupplier, pose -> pose.getY(), -1))
				.withPosition(0, 1);
		layout
				.addDouble("Z", mapNullableSupplierToDouble(poseSupplier, pose -> pose.getZ(), -1))
				.withPosition(0, 2);
		layout
				.addDouble(
						"Rot X",
						mapNullableSupplierToDouble(poseSupplier, pose -> pose.getRotation().getX(), -1))
				.withPosition(1, 0);
		layout
				.addDouble(
						"Rot Y",
						mapNullableSupplierToDouble(poseSupplier, pose -> pose.getRotation().getY(), -1))
				.withPosition(1, 1);
		layout
				.addDouble(
						"Rot Z",
						mapNullableSupplierToDouble(poseSupplier, pose -> pose.getRotation().getZ(), -1))
				.withPosition(1, 2);
		return layout;
	}
}
