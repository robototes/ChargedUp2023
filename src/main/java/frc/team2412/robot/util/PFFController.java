package frc.team2412.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

/**
 * Class for PFF controllers that provide P controllers but with the advantage of FeedForward
 *
 * @author Alex Stedman
 * @param <T> type of input/output
 */
@SuppressWarnings("unused")
public class PFFController<T> {
	private final double p, f;
	private final Timer timer;
	private final PFFOperator<T> op;
	private double targetPositionTolerance = 0.1;

	/**
	 * functional interface for core operator used by the controller
	 *
	 * @author Alex Stedman
	 * @param <T> type of input/output
	 */
	public interface PFFOperator<T> {

		T add(T first, T second);

		default T subtract(T first, T second) {
			return add(first, scale(second, -1));
		}

		T scale(T first, double scale);

		boolean equals(T first, T second, double tolerance);

		T zero();

		default T operate(T first, T second, double scale, double tolerance) {
			if (equals(first, second, tolerance)) return zero();
			return scale(subtract(first, second), scale);
		}
	}

	protected PFFController(double pValue, double fValue, PFFOperator<T> operator) {
		p = pValue;
		f = fValue;
		op = operator;
		timer = new Timer();
	}

	private T targetPosition;

	/**
	 * Set the target position of the controller
	 *
	 * @param target new target position
	 * @return this
	 */
	public PFFController<T> setTargetPosition(T target) {
		targetPosition = target;
		timer.reset();
		return this;
	}

	public PFFController<T> setTargetPositionTolerance(double tolerance) {
		targetPositionTolerance = tolerance;
		return this;
	}

	private T pastValue;

	/**
	 * output the adjustment offered by the controller
	 *
	 * @param measuredValue measured value at current time
	 * @return new value to set
	 */
	public T update(T measuredValue) {
		if (pastValue == null) pastValue = measuredValue;
		if (targetPosition == null) return null;
		T velocity = op.operate(measuredValue, pastValue, f / timer.get(), targetPositionTolerance);
		timer.reset();
		pastValue = measuredValue;
		T pControl = op.operate(targetPosition, measuredValue, p, targetPositionTolerance);
		return op.add(pControl, velocity);
	}

	/**
	 * create PFF controller of doubles
	 *
	 * @param pValue P value for proportional control
	 * @param fValue F value for feedforward control
	 * @return new PFFController
	 */
	public static PFFController<Double> ofDouble(double pValue, double fValue) {
		return new PFFController<>(
				pValue,
				fValue,
				new PFFOperator<>() {
					@Override
					public Double add(Double first, Double second) {
						return first + second;
					}

					@Override
					public Double scale(Double first, double scale) {
						return first * scale;
					}

					@Override
					public boolean equals(Double first, Double second, double tolerance) {
						return Math.abs(first - second) < tolerance;
					}

					@Override
					public Double zero() {
						return 0.0;
					}
				});
	}

	/**
	 * create PFF controller of {@link Translation2d}
	 *
	 * @param pValue P value for proportional control
	 * @param fValue F value for feedforward control
	 * @return new PFFController
	 */
	public static PFFController<Translation2d> ofTranslation2d(double pValue, double fValue) {
		return new PFFController<>(
				pValue,
				fValue,
				new PFFOperator<>() {
					@Override
					public Translation2d add(Translation2d first, Translation2d second) {
						return first.plus(second);
					}

					@Override
					public Translation2d scale(Translation2d first, double scale) {
						return first.times(scale);
					}

					@Override
					public boolean equals(Translation2d first, Translation2d second, double tolerance) {
						return first.equals(second);
					}

					@Override
					public Translation2d zero() {
						return new Translation2d(0, 0);
					}
				});
	}
}
