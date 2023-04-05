package frc.team2412.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.time.Duration;

public class ArmLEDSubsystem extends SubsystemBase {

	// CONSTANTS

	// time out durations (seconds)
	private static final long CONNECT_TIMEOUT_DURATION = 15;
	private static final long REQUEST_TIMEOUT_DURATION = 15;

	private static final String IP = "10.24.12.12"; // TODO: get IP

	private static final int LED_ALPHA = 205;
	private static final int EFFECT_SPEED = 50;
	private static final int EFFECT_INTENSITY = 10;

	// enum selector

	public static enum ColorSelector {
		RED(255, 0, 0),
		ORANGE(255, 165, 0),
		YELLOW(255, 255, 0),
		GREEN(0, 255, 0),
		BLUE(0, 0, 255),
		PURPLE(255, 0, 255),
		WHITE(205, 205, 205),
		BLACK(0, 0, 0);

		public final int r;
		public final int g;
		public final int b;

		ColorSelector(int r, int g, int b) {
			this.r = r;
			this.g = g;
			this.b = b;
		}
	}

	public static enum LEDStateSelector {
		OFF,
		ALLIANCE_COLOR,
		CUSTOM_COLOR;
	}

	// VARIABLES

	// led
	private ColorSelector color1;
	private ColorSelector color2;
	private ColorSelector color3;
	private int enabled;
	private int effectIndex; // https://kno.wled.ge/features/effects/
	private LEDStateSelector state;

	// http get request
	private HttpClient client;
	private HttpRequest request;

	// logging
	private SendableChooser<ColorSelector> color1Chooser = new SendableChooser<>();
	private SendableChooser<ColorSelector> color2Chooser = new SendableChooser<>();
	private SendableChooser<ColorSelector> color3Chooser = new SendableChooser<>();
	private SendableChooser<InstantCommand> stateChooser = new SendableChooser<>();

	private ShuffleboardTab armLEDTab;

	// CONSTRUCTOR

	public ArmLEDSubsystem() {

		// led values
		enabled = 1;
		effectIndex = 0;
		color1 = ColorSelector.RED;
		color2 = ColorSelector.BLACK;
		color3 = ColorSelector.RED;
		state = LEDStateSelector.ALLIANCE_COLOR;

		// http request
		try {
			client =
					HttpClient.newBuilder()
							.connectTimeout(Duration.ofSeconds(CONNECT_TIMEOUT_DURATION))
							.build();
		} catch (Exception e) {
			System.out.println("Failed to initialize http client");
			System.out.println(e.getMessage());
		}
		try {
			request = getRequest();
		} catch (Exception e) {
			System.out.println("Failed to initialize http request");
			System.out.println(e.getMessage());
		}

		// logging
		color1Chooser.setDefaultOption("RED", ColorSelector.RED);
		color1Chooser.addOption("ORANGE", ColorSelector.ORANGE);
		color1Chooser.addOption("YELLOW", ColorSelector.YELLOW);
		color1Chooser.addOption("GREEN", ColorSelector.GREEN);
		color1Chooser.addOption("BLUE", ColorSelector.BLUE);
		color1Chooser.addOption("PURPLE", ColorSelector.PURPLE);
		color1Chooser.addOption("WHITE", ColorSelector.WHITE);
		color1Chooser.addOption("BLACK", ColorSelector.BLACK);

		color2Chooser.addOption("RED", ColorSelector.RED);
		color2Chooser.addOption("ORANGE", ColorSelector.ORANGE);
		color2Chooser.addOption("YELLOW", ColorSelector.YELLOW);
		color2Chooser.addOption("GREEN", ColorSelector.GREEN);
		color2Chooser.addOption("BLUE", ColorSelector.BLUE);
		color2Chooser.addOption("PURPLE", ColorSelector.PURPLE);
		color2Chooser.addOption("WHITE", ColorSelector.WHITE);
		color2Chooser.setDefaultOption("BLACK", ColorSelector.BLACK);

		color3Chooser.setDefaultOption("RED", ColorSelector.RED);
		color3Chooser.addOption("ORANGE", ColorSelector.ORANGE);
		color3Chooser.addOption("YELLOW", ColorSelector.YELLOW);
		color3Chooser.addOption("GREEN", ColorSelector.GREEN);
		color3Chooser.addOption("BLUE", ColorSelector.BLUE);
		color3Chooser.addOption("PURPLE", ColorSelector.PURPLE);
		color3Chooser.addOption("WHITE", ColorSelector.WHITE);
		color3Chooser.addOption("BLACK", ColorSelector.BLACK);

		stateChooser.addOption(
				"Disable Arm LEDs",
				new InstantCommand(
						() -> {
							enableLED(false);
						}));
		stateChooser.addOption(
				"Use Alliance Color (RED/BLUE)",
				new InstantCommand(
						() -> {
							enableLED(true);
							setLEDAlliance();
						}));
		stateChooser.addOption(
				"Use Custom Colors",
				new InstantCommand(
						() -> {
							enableLED(true);
							setLEDCustom();
						}));
		stateChooser.setDefaultOption(
				"Use Alliance Color (RED/BLUE)",
				new InstantCommand(
						() -> {
							enableLED(true);
							setLEDAlliance();
						}));

		armLEDTab = Shuffleboard.getTab("Arm LED");

		armLEDTab.add("Color 1", color1Chooser).withPosition(0, 0).withSize(2, 1);
		armLEDTab.add("Color 2", color2Chooser).withPosition(1, 0).withSize(2, 1);
		armLEDTab.add("Color 3", color3Chooser).withPosition(2, 0).withSize(2, 1);

		color1 = color1Chooser.getSelected();
		color2 = color2Chooser.getSelected();
		color3 = color3Chooser.getSelected();

		setLEDAlliance();
	}

	// METHODS

	/** Called during autonomous to be green (color might be changed later?) */
	public void setLEDAutonomous() {
		color1 = ColorSelector.GREEN;
		effectIndex = 0;

		updateLED();
	}

	/** Called during Teleop to set LED to red or blue based off alliance. */
	public void setLEDAlliance() {

		if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
			// red
			color1 = ColorSelector.RED;

		} else {
			// blue
			color1 = ColorSelector.BLUE;
		}
		effectIndex = 0;

		updateLED();
	}

	/** Potentially called during Teleop after alliance selection to represent overall team colors */
	public void setLEDCustom() {
		effectIndex = 2;

		color1 = color1Chooser.getSelected();
		color2 = color2Chooser.getSelected();

		updateLED();
	}

	/*
	 * Updates the LED by sending an HTTP request
	 */
	public void updateLED() {
		try {
			request = getRequest();
		} catch (Exception e) {
			System.out.println("Failed to create Http Request");
			System.out.println(e.getMessage());
		}

		try {
			client.sendAsync(request, HttpResponse.BodyHandlers.ofString());
		} catch (Exception e) {
			System.out.println("Failed to send http request");
			System.out.println(e.getMessage());
		}
	}

	/*
	 * Enables/Disables the LED.
	 * @param enable Enables if true, disables if false.
	 */
	public void enableLED(boolean enable) {
		if (enable) {
			enabled = 1;
			return;
		}
		enabled = 0;

		updateLED();
	}

	public HttpRequest getRequest() {
		String uri = getURI();
		return HttpRequest.newBuilder()
				.timeout(Duration.ofSeconds(REQUEST_TIMEOUT_DURATION))
				.uri(URI.create(getURI()))
				.build();
	}

	// TODO: finish URI getter
	public String getURI() {
		return "http://"
				+ IP
				+ "/win?&T="
				+ enabled
				+ "&A="
				+ LED_ALPHA
				+ "&R="
				+ color1.r
				+ "&G="
				+ color1.g
				+ "&B="
				+ color1.b
				+ "&R2="
				+ color2.r
				+ "&G2="
				+ color2.g
				+ "&B2="
				+ color2.b
				+ "&FX="
				+ effectIndex;
	}
}
