package frc.team2412.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
	private static final String RED_URI = "https://" + IP + "/win&T=1&A=255&R=255&G=0&B=0&FX=0";
	private static final String ORANGE_URI = "https://" + IP + "/win&T=1&A=255&R=255&G=165&B=0&FX=0";
	private static final String YELLOW_URI = "https://" + IP + "/win&T=1&A=255&R=255&G=255&B=0&FX=0";
	private static final String GREEN_URI = "https://" + IP + "/win&T=1&A=255&R=0&G=255&B=0&FX=0";
	private static final String BLUE_URI = "https://" + IP + "/win&T=1&A=255&R=0&G=0&B=255&FX=0";
	private static final String PURPLE_URI = "https://" + IP + "/win&T=1&A=255&R=255&G=0&B=255&FX=0";
	private static final String WHITE_URI = "https://" + IP + "/win&T=1&A=205&R=205&G=205&B=205&FX=0";
	private static final String BLACK_URI = "https://" + IP + "/win&T=1&A=255&R=0&G=0&B=0&FX=0";

	// enum selector

	public static enum colorSelector {
		RED,
		ORANGE,
		YELLOW,
		GREEN,
		BLUE,
		PURPLE,
		WHITE,
		BLACK;
	}

	// VARIABLES

	HttpClient client;
	HttpRequest request;

	// alliance colors

	// CONSTRUCTOR

	public ArmLEDSubsystem() {

		client =
				HttpClient.newBuilder()
						.connectTimeout(Duration.ofSeconds(CONNECT_TIMEOUT_DURATION))
						.build();
		request =
				HttpRequest.newBuilder().uri(URI.create(GREEN_URI)).build(); // TODO: set timeout duration
	}

	// METHODS

	public void setLED() {}

	public void setLEDAutonomous() {
		request = HttpRequest.newBuilder().uri(URI.create(GREEN_URI)).build();
	}

	public void setLEDTeleOp() {

		if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
			// red
			request = HttpRequest.newBuilder().uri(URI.create(RED_URI)).build();

		} else {
			// blue
			request = HttpRequest.newBuilder().uri(URI.create(BLUE_URI)).build();
		}
	}

	public void setLEDAlliance() {}

	public void request() {
		try {
			client.sendAsync(request, HttpResponse.BodyHandlers.ofString());
		} catch (Exception e) {
			System.out.println("Failed to http request");
		}
	}
}
