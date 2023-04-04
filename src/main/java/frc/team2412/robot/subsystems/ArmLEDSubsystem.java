package frc.team2412.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.time.Duration;

public class ArmLEDSubsystem {

	// CONSTANTS

	// time out durations (seconds)
	private static final long CONNECT_TIMEOUT_DURATION = 15;
	private static final long REQUEST_TIMEOUT_DURATION = 15;

	private static final String IP = "wled-00e2b4"; // TODO: get IP

	// color URIs
	// private static final String RED_URI = "https://" + IP + "/win&T=1&A=255&R=255&G=0&B=0&FX=0";
	// private static final String ORANGE_URI = "https://" + IP + "/win&T=1&A=255&R=255&G=165&B=0&FX=0";
	// private static final String YELLOW_URI = "https://" + IP + "/win&T=1&A=255&R=255&G=255&B=0&FX=0";
	// private static final String GREEN_URI = "https://" + IP + "/win&T=1&A=255&R=0&G=255&B=0&FX=0";
	// private static final String BLUE_URI = "https://" + IP + "/win&T=1&A=255&R=0&G=0&B=255&FX=0";
	// private static final String PURPLE_URI = "https://" + IP + "/win&T=1&A=255&R=255&G=0&B=255&FX=0";
	// private static final String WHITE_URI = "https://" + IP + "/win&T=1&A=205&R=205&G=205&B=205&FX=0";
	// private static final String BLACK_URI = "https://" + IP + "/win&T=1&A=255&R=0&G=0&B=0&FX=0";

	// enum selector

	public static enum ColorSelector {
		RED("/win&T=1&A=255&R=255&G=0&B=0&FX=0"),
		ORANGE("/win&T=1&A=255&R=255&G=165&B=0&FX=0"),
		YELLOW("/win&T=1&A=255&R=255&G=255&B=0&FX=0"),
		GREEN("/win&T=1&A=255&R=0&G=255&B=0&FX=0"),
		BLUE("/win&T=1&A=255&R=0&G=0&B=255&FX=0"),
		PURPLE("/win&T=1&A=255&R=255&G=0&B=255&FX=0"),
		WHITE("/win&T=1&A=205&R=205&G=205&B=205&FX=0"),
		BLACK("/win&T=1&A=255&R=0&G=0&B=0&FX=0");

		public final String url;

		ColorSelector(String url) {
			this.url = url;
		}
	}

	// VARIABLES

	private HttpClient client;
	private HttpRequest request;

	// alliance colors
	private ColorSelector color1;
	private ColorSelector color2;
	private ColorSelector color3;

	// logging
	private SendableChooser<ColorSelector> color1Chooser = new SendableChooser<>();
	private SendableChooser<ColorSelector> color2Chooser = new SendableChooser<>();
	private SendableChooser<ColorSelector> color3Chooser = new SendableChooser<>();

	private ShuffleboardTab armLEDTab;

	// CONSTRUCTOR

	public ArmLEDSubsystem() {

		client =
				HttpClient.newBuilder()
						.connectTimeout(Duration.ofSeconds(CONNECT_TIMEOUT_DURATION))
						.build();
		request =
				HttpRequest.newBuilder().uri(URI.create(GREEN_URI)).build(); // TODO: set timeout duration

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

		armLEDTab = Shuffleboard.getTab("Arm LED");

		armLEDTab.add("Color 1", color1Chooser).withPosition(0, 0).withSize(2, 1);
		armLEDTab.add("Color 2", color2Chooser).withPosition(0, 0).withSize(2, 1);
		armLEDTab.add("Color 3", color3Chooser).withPosition(0, 0).withSize(2, 1);
	}

	// METHODS

	public void setLEDAutonomous() {
		request = HttpRequest.newBuilder().uri(URI.create(getURI(ColorSelector.GREEN))).build();
		request();
	}

	public void setLEDTeleOp() {

		if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
			// red
			request = HttpRequest.newBuilder().uri(URI.create(getURI(ColorSelector.RED))).build();

		} else {
			// blue
			request = HttpRequest.newBuilder().uri(URI.create(getURI(ColorSelector.BLUE))).build();
		}
		request();
	}

	public void setLEDAlliance() {}

	public void request() {
		try {
			client.sendAsync(request, HttpResponse.BodyHandlers.ofString());
		} catch (Exception e) {
			System.out.println("Failed to http request");
		}
	}

    public String getURI(ColorSelector color) {
		return "https://" + IP + color.url;
	}
}
