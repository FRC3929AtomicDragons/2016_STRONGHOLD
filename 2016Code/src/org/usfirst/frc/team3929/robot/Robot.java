package org.usfirst.frc.team3929.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx_mxp.AHRS;

//have to import vision from outside directory, not loading onto roborio
//import org.usfirst.frc.team3929.robot.TowerTracker;


/*HAVE A LIST OF THE CONTROLS HERE:
 * 
 * 
 * */
 
// Vision code credit to team 3019 thanks!
public class Robot extends IterativeRobot {

	public enum IntakeState {
		MANINTAKE, AUTOINTAKE
	}

	IntakeState currentIntakeState = IntakeState.MANINTAKE;

	public enum hoodState {
		MANHOOD, AUTOHOOD
	}

	hoodState currentHoodState = hoodState.MANHOOD;

	public enum AutoHoodState {
		LOWERED, HIGHGOAL
	}

	AutoHoodState currentAutoHoodState = AutoHoodState.LOWERED;

	public enum AutoIntakeState {
		REST, VERTICAL, STOPONE, STOPTWO,
	}

	AutoIntakeState currentAutoIntake = AutoIntakeState.VERTICAL;

	Joystick drive;
	Joystick operator;
	
	//Drive speed controllers
	Victor bL;
	Victor bR;
	Victor fL;
	Victor fR;
	RobotDrive tank;
	
	//Mechanism speed controllers
	Victor intake;
	Victor shooter;
	Victor hood;

	Encoder rightEncoder;
	Encoder leftEncoder;
	Encoder intakeEncoder;
	Encoder hoodEncoder;

	//Gyro and Digital
	SerialPort serial_port;
	AHRS imu;
	DigitalInput intakeLim;
	
	Timer timer;
	
	NetworkTable table;
	
	PIDTool pidIntake;
	PIDTool pidHood;
	PIDTool pidGyro;
	PIDTool pidDist;
	
	//PID parameters, need to find actual values
	final double kPgyro = 0.04;
	final double kIgyro = 0.0;
	final double kDgyro = 0.0;
	final double MAX_ROTATION_INPUT = 0.3;

	final double kPhood = 0.04;
	final double kIhood = 0.0;
	final double kDhood = 0.0;
	final double MAX_HOOD_INPUT = 0.4;
	
	final double kPdist = 0.04;
	final double kIdist = 0.0;
	final double kDdist = 0.0;
	final double MAX_DIST_INPUT = 0.4;
	
	final double hoodSpeed = 0.6;
	
	//Encoder limits for intake positions
	final double verticalStop = 1000; 
	final double stopOne = 1000;
	final double stopTwo = 1000;;

	//Encoder limits for hood positions
	final double loweredAngle = 0;
	double highGoalAngle;
	
	//Encoder values for Intake and hood count
	int rightCount, leftCount, intakeCount, hoodCount;
	
	double driveDistance;
	
	//Encoder DPP values
	final double driveEncoderDPP = 10;
	final double hoodEncoderDPP = 10; 
	final double intakeEncoderDPP = 10;

	double intakeSpeed;
	
	//Camera Distance measurement, defaultDistance printed when no NetworkTable value
	final double defaultDistance = 10;
	double distance;
	
	//To check for hood loop finish
	boolean finished = false;

	public void robotInit() {

		drive = new Joystick(0);
		operator = new Joystick(1);

		bL = new Victor(0); 
		bR = new Victor(1); 
		fL = new Victor(2); 
		fR = new Victor(3);
		tank = new RobotDrive(fL, bL, fR, bR);

		intake = new Victor(4);
		shooter = new Victor(5);
		hood = new Victor(6);

		// add more PID initialization
		pidIntake = new PIDTool(kPgyro, kIgyro, kDgyro, 0, -MAX_ROTATION_INPUT, MAX_ROTATION_INPUT);
		pidHood = new PIDTool(kPhood, kIhood, kDhood, 0, -MAX_HOOD_INPUT, MAX_HOOD_INPUT);
		pidDist = new PIDTool(kPdist, kIdist, kDdist, 0, -MAX_DIST_INPUT, MAX_DIST_INPUT);
		pidGyro = new PIDTool(kPgyro, kIgyro, kDgyro, 0, -MAX_ROTATION_INPUT, MAX_ROTATION_INPUT);

		rightEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
		leftEncoder = new Encoder(2, 3, true, Encoder.EncodingType.k4X);
		
		intakeEncoder = new Encoder(4, 5, false, Encoder.EncodingType.k4X);
		hoodEncoder = new Encoder(6, 7, false, Encoder.EncodingType.k4X);

		intakeLim = new DigitalInput(8);
		
		//thatOnePortThat'sOnTheLeftButActuallyOntheRightGam3r

		table = NetworkTable.getTable("DistanceBoard");

		try {

			// Use SerialPort.Port.kOnboard if connecting nav6 to Roborio Rs-232
			// port
			// Use SerialPort.Port.kMXP if connecting navX MXP to the RoboRio
			// MXP port
			// Use SerialPort.Port.kUSB if connecting nav6 or navX MXP to the
			// RoboRio USB port

			serial_port = new SerialPort(57600, SerialPort.Port.kMXP);

			// You can add a second parameter to modify the
			// update rate (in hz) from. The minimum is 4.
			// The maximum (and the default) is 100 on a nav6, 60 on a navX MXP.
			// If you need to minimize CPU load, you can set it to a
			// lower value, as shown here, depending upon your needs.
			// The recommended maximum update rate is 50Hz

			// You can also use the IMUAdvanced class for advanced
			// features on a nav6 or a navX MXP.

			// You can also use the AHRS class for advanced features on
			// a navX MXP. This offers superior performance to the
			// IMU Advanced class, and also access to 9-axis headings
			// and magnetic disturbance detection. This class also offers
			// access to altitude/barometric pressure data from a
			// navX MXP Aero.

			byte update_rate_hz = 50;
			// imu = new IMU(serial_port,update_rate_hz);
			// imu = new IMUAdvanced(serial_port,update_rate_hz);
			imu = new AHRS(serial_port, update_rate_hz);
		} catch (Exception ex) {

		}

	}

	public void autonomousInit() {
		imu.zeroYaw();
		resetDriveEncoders();
		resetOpEncoders();
	}

	public void autonomousPeriodic() {
		double angle = imu.getYaw();
		double xInput, yInput;

		readDriveEncoders();
		readOpEncoders();

		distance = table.getNumber("Distance", defaultDistance);
		SmartDashboard.putString("DB/String 1", "Camera Estimate " + Double.toString(distance));
		System.out.println(distance);

		SmartDashboard.putString("DB/String 2", Double.toString(driveDistance));
		SmartDashboard.putString("DB/String 3", Double.toString(imu.getYaw()));
		SmartDashboard.putString("DB/String 0", currentHoodState.name());
		SmartDashboard.putString("DB/String 4", Boolean.toString(intakeLim.get()));

		xInput = 0.0;
		yInput = 0.0;

		if (driveDistance >= -130) {
			yInput = 0.5;
		} else {
			yInput = 0.0;
			outtake();
		}
		
		tank.arcadeDrive(xInput, yInput);
	}

	public void telopInit() {
		imu.zeroYaw();
		resetDriveEncoders();
		resetOpEncoders();
		timer.reset();
	}

	public void teleopPeriodic() {
		
		double drivePower;
		
		if(drive.getRawButton(1)) { 
			drivePower = 1.00; 
		} else{ 
			drivePower = 0.5;
		}

		readDriveEncoders();
		readOpEncoders();

		tank.tankDrive(drive.getRawAxis(2) * drivePower, drive.getRawAxis(5) * drivePower);
		
		intakePeriodic();
		hoodPeriodic();
		
		//intake();

		distance = table.getNumber("Distance", defaultDistance);
		SmartDashboard.putString("DB/String 1", Double.toString(distance));
		System.out.println(distance);

		// rough mock-up of teleopCode

	}

	public void intake() {
		intake.set(intakeSpeed);
	}

	public void outtake() {
		intake.set(-intakeSpeed);
	}

	public void intakePeriodic() {
		if (intakeLim.get()) {
			resetOpEncoders();
			intakeSpeed = 0.0;
		}
		switch (currentIntakeState) {

		case MANINTAKE:
			if (operator.getRawButton(5)) {
				currentIntakeState = IntakeState.AUTOINTAKE;
			}
			manIntake();
			break;
		case AUTOINTAKE:
			autoIntake();
			break;

		}

	}

	public void manIntake() {
		if (operator.getRawButton(1)) {
			intakeSpeed = 1.0;
		} else if (operator.getRawButton(2)) {
			intakeSpeed = -1.0;
		} else {
			intakeSpeed = 0.0;
		}
		if (operator.getRawButton(4)){
			shooter.set(1.0);
		}

	}

	public void autoIntake() {
		switch (currentAutoIntake) {

		case REST:
			currentAutoIntake = AutoIntakeState.VERTICAL;
			break;
		case VERTICAL:
			if (!intakeLim.get()) {
				intakeSpeed = 0.2;
			} else if (intakeLim.get() && intakeCount == 0.0 ) {
				intakeSpeed = 0.0;
				currentAutoIntake = AutoIntakeState.STOPONE;
			}
			break;

		case STOPONE:
			
			//Check this..
			if ((intakeCount < stopOne)) {
				if (operator.getRawButton(5)) {
					while (intakeCount < stopOne) {
						intakeSpeed = -0.2;
					}
				}
			}
			else {
				intakeSpeed = 0.0;
				resetOpEncoders();
			}
			break;
		}
	}

	public void hoodPeriodic() {
		switch (currentHoodState) {

		case MANHOOD:
			if(operator.getRawButton(3)){
				currentHoodState = hoodState.AUTOHOOD;
			} 
			manHood();
			break;

		case AUTOHOOD:
			autoHood();
			break;
		}
	}

	public void manHood() {
		if (operator.getRawButton(6)) {
			hood.set(0.5);
		} else if (operator.getRawButton(7)) {
			hood.set(-0.5);
		} else {
			hood.set(0.0);
		}
	}

	public void autoHood() {		
		switch (currentAutoHoodState) {

		case LOWERED:
			timer.start();
			if (timer.get() <= 3.0) {
				hood.set(-0.5);
			} else {
				timer.reset();
				hood.set(0.0);
				resetOpEncoders();
			}
			if(finished == true){
				currentHoodState = hoodState.MANHOOD;
				finished = false;
			} else {
				resetOpEncoders();
				currentAutoHoodState = AutoHoodState.HIGHGOAL;
			}
			break;
		case HIGHGOAL:

			// Actually figure out the math here please-----------
			highGoalAngle = (int) Math.round(calculateHighAngle()) / 2;
			if (hoodCount <= highGoalAngle) {
				hood.set(0.5);
			} else {
				timer.start();
				hood.set(0.0);
				shooter.set(1.0);
				if (timer.get() >= 2.0 && timer.get() < 4.0) {
					if (intakeCount < stopTwo) {
							intakeSpeed = -0.2;
						}
					} else if (intakeCount >= stopTwo) {
						intakeSpeed = 0.0;
						resetOpEncoders();
						currentAutoIntake = AutoIntakeState.REST;
						currentIntakeState = IntakeState.MANINTAKE;
					}
					else {
						shooter.set(0.0);
					} 
				}
			finished = true;
			currentAutoHoodState = AutoHoodState.LOWERED;
			break;
			}
		}
	

	public void readDriveEncoders() {
		rightCount = rightEncoder.get();
		leftCount = leftEncoder.get();

		// placeholding multipliers

		driveDistance = driveEncoderDPP * ((rightCount + leftCount)/2);

	}

	public void readOpEncoders() {
		intakeCount = intakeEncoder.get();
		hoodCount = hoodEncoder.get();
	}

	public void resetDriveEncoders() {
		rightEncoder.reset();
		leftEncoder.reset();
		driveDistance = 0.0;
	}

	public void resetOpEncoders() {
		intakeEncoder.reset();
		hoodEncoder.reset();
		hoodCount = 0;
		intakeCount = 0;
	}

	public double calculateHighAngle() {

		double[] newtZeros = new double[21];
		double[] newtZerosInt = new double[21];
		double[] newtZerosDeriv = new double[21];
		double Radians;

		newtZeros[0] = 0.5; // initial value, can be tuned for greater accuracy,
							// but this value should be close enough
		newtZerosDeriv[0] = 1;
		newtZerosInt[0] = 1;
		double zeroX = 0;

		// ft/s
		double speed = 32.8;
		double g = 16;
		double speedSquare = speed * speed;

		double goal = 8.0833333333333; // This value is the height of the ball,
									// going into the middle of the goal.
		double release = 1.5; // height of ball upon release.

		for (int b = 1; b < newtZeros.length; b++) {

			// solving for reaching goal on rise, substituting in for theta.
			// Equation in terms of t Andrew's version :D
			newtZerosInt[b] = (-g * newtZeros[b - 1] * newtZeros[b - 1])
					+ (speed * (Math.sin(Math.acos(distance / (speed * newtZeros[b - 1])))) * newtZeros[b - 1]) + (release - goal);
			newtZerosDeriv[b] = (-2 * g * newtZeros[b - 1]) + (speed)
					/ (Math.sqrt(1 - (((distance * distance) / (speedSquare)) / (newtZeros[b - 1] * newtZeros[b - 1]))));
			newtZeros[b] = newtZeros[b - 1] - (newtZerosInt[b]) / (newtZerosDeriv[b]);

		}
		Radians = Math.acos(distance / (speed * newtZeros[20]));
		System.out.println("Distance: " + distance + "    Radians: " + Radians + "           Degrees: " + (Radians * 180) / Math.PI);

		return Radians;
	}

}
