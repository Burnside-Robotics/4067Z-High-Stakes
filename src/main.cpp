#include "main.h"


int interpolate(float last, float current, float strength) {
	return last + (current - last) * strength;
}

enum DriveMode {
	DRIVE_MODE_TANK,
	DRIVE_MODE_ARCADE
};
enum Status {
	STATUS_RECORDING,
	STATUS_RECORD_COUNTDOWN,
	STATUS_REPLAYING,
	STATUS_DRIVING,
	STATUS_REPLAY_COUTNDOWN
};
int replaySaveSlot = 0;

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(0, "0 (Init)");
	pros::lcd::set_text(1, "1");
	pros::lcd::set_text(2, "2");
	pros::lcd::set_text(3, "3");
	pros::lcd::set_text(4, "4");
	pros::lcd::set_text(5, "5");
	pros::lcd::set_text(6, "6");
	pros::lcd::set_text(7, "7");
	
	pros::lcd::register_btn1_cb(on_center_button);

	pros::ADIDigitalOut goalClamp('A');
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	pros::lcd::set_text(0, "Disabled");
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
	pros::lcd::set_text(0, "Comp init");
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	pros::lcd::set_text(0, "Autonomous");

	
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::MotorGroup left_mg({-20, -19});
	pros::MotorGroup right_mg({18, 17});
	pros::Motor intake(1);
	pros::Motor ramp(-10);
	pros::adi::DigitalOut goalClamp('A');

	int driveDeadzone = 20;
	DriveMode driveMode = DRIVE_MODE_ARCADE;
	float interpolateStrength = 0.2;
	int lastTurn = 0;
	bool lastGoalClamp = false;
	bool switchButtonStatus = 0;	// 0 -> not pressed, 1 -> held, 2 -> just pressed

	Status runStatus = STATUS_DRIVING;
	// --- Arrays to save ---
	int drivetrainControl[1500];	// 0 -> 749 is left, 750 -> 1499 is right
	int intakeControl[750];			// 0 -> 749 is intake
	int goalClampControlArray[50];	// Stores the time when the goal clamp is toggled
	// ----------------------
	int nextGoalClampIndex = 0;

	int time = 0;

	replaySaveSlot = 0;

	while (true) {
		if (master.get_digital(DIGITAL_Y) && master.get_digital(DIGITAL_B)  && switchButtonStatus == 0) {
			switchButtonStatus = 2;
			if (driveMode == DRIVE_MODE_ARCADE) {	// Switches drive mode
				driveMode = DRIVE_MODE_TANK;
				master.set_text(3, 0, "Tank drive");
			} else if (driveMode == DRIVE_MODE_TANK) {
				driveMode = DRIVE_MODE_ARCADE;
				master.set_text(3, 0, "Arcade drive");
			}
		} else if (master.get_digital(DIGITAL_Y) && master.get_digital(DIGITAL_B) && switchButtonStatus == 2) {
			switchButtonStatus = 1;
		} else if (!master.get_digital(DIGITAL_Y) && !master.get_digital(DIGITAL_B) && switchButtonStatus == 1) {
			switchButtonStatus = 0;
		}
		int left = 0;
		int right = 0;
		int intakeDirection = 0;
		bool goalClampControl = false;
		if (master.get_digital(DIGITAL_R1)) {
			goalClampControl = true;
		}
		if (master.get_digital(DIGITAL_L1)) {
			intakeDirection = 1;
		} else if (master.get_digital(DIGITAL_L2)) {
			intakeDirection = -1;
		}
		switch (driveMode) {		// Sets the left and right motor values based on the drive mode
			case DRIVE_MODE_TANK:
				left = master.get_analog(ANALOG_LEFT_Y);
				right = master.get_analog(ANALOG_RIGHT_Y);
				break;
			case DRIVE_MODE_ARCADE:
				int direction = master.get_analog(ANALOG_LEFT_Y);
				int turn = interpolate(lastTurn, master.get_analog(ANALOG_RIGHT_X) * -2, interpolateStrength);
				lastTurn = turn;
				left = std::clamp(direction + turn, -127, 127);
				right = std::clamp(direction - turn, -127, 127);
				break;
		}

		// Change recording / replay / driving mode
		if (master.get_digital(DIGITAL_X) && runStatus == STATUS_DRIVING) {
			// Start countdown
			runStatus = STATUS_RECORD_COUNTDOWN;
			time = 0;
		} else if (runStatus == STATUS_RECORD_COUNTDOWN && time < 150) {
			time += 1;
			pros::lcd::set_text(0, "Recording in " + std::to_string(3.0 - float(time * 20) / 1000.0) + " seconds");
		} else if (runStatus == STATUS_RECORD_COUNTDOWN && time >= 150) {
			// Start recording
			pros::lcd::set_text(0, "Recording");
			runStatus = STATUS_RECORDING;
			time = 0;
		} else if (runStatus == STATUS_RECORDING && time < 750) {
			// Recording ------------
			drivetrainControl[time] = left;
			drivetrainControl[time + 750] = right;
			intakeControl[time] = intakeDirection;
			if (goalClampControl != lastGoalClamp) {
				goalClampControlArray[nextGoalClampIndex] = time;
				nextGoalClampIndex++;
			}

			time++;
		} else if (runStatus == STATUS_RECORDING && time >= 750) {
			// End recording
			runStatus = STATUS_DRIVING;
			pros::lcd::set_text(0, "Driving");
			// Saves the file to disk
			std::string filePath = "/usd/replay" + std::to_string(replaySaveSlot) + ".bin";
			const char * fileName = filePath.c_str();
			FILE* usd_file_write = fopen(fileName, "wb");
			//FILE* usd_file_write = fopen("/usd/replay.bin", "wb");
			if (usd_file_write == nullptr) {
				pros::lcd::set_text(0, "Failed to open file");
				return;
			}
			// Concatenates the arrays into one array to save
			int concatArray[2300];	// 0 -> 1499 is drivetrain, 1500 -> 2249 is intake, 2250 -> 2299 is goalClamp
			for (int i = 0; i < 1500; i++) {
				concatArray[i] = drivetrainControl[i];
			}
			for (int i = 0; i < 750; i++) {
				concatArray[i + 1500] = intakeControl[i];
			}
			for (int i = 0; i < 50; i++) {
				concatArray[i + 2250] = goalClampControlArray[i];
			}
			size_t elementsWritten = std::fwrite(concatArray, sizeof(int), 2300, usd_file_write);
			if (elementsWritten != 2300) {
				pros::lcd::set_text(2, "Error writing data to file!");
				return;
			} else {
				pros::lcd::set_text(2, "Array written to file successfully!");
			}
			std::fclose(usd_file_write);
		}

		if (master.get_digital(DIGITAL_A) && runStatus == STATUS_DRIVING) {
			// Start replay
			runStatus = STATUS_REPLAY_COUTNDOWN;
			time = 0;
		} else if (runStatus == STATUS_REPLAY_COUTNDOWN && time < 150) {
			// Replay countdown
			time++;
			pros::lcd::set_text(0, "Replaying in " + std::to_string(3.0 - float(time * 20) / 1000.0) + " seconds");
		} else if (runStatus == STATUS_REPLAY_COUTNDOWN && time >= 150) {
			// Starts replay
			time = 0;
			nextGoalClampIndex = 0;
			runStatus = STATUS_REPLAYING;
			pros::lcd::set_text(0, "Replaying");
			// Loads file from disk
			std::string filePath = "/usd/replay" + std::to_string(replaySaveSlot) + ".bin";
			const char * fileName = filePath.c_str();
			FILE* usd_file_read = fopen(fileName, "r");
			//FILE* usd_file_read = fopen("/usd/replay.bin", "r");
			if (usd_file_read == nullptr) {
				pros::lcd::set_text(2, "Failed to open read file");
				return;
			}
			int buf[2300];
			size_t elementsRead = std::fread(buf, sizeof(int), 2300, usd_file_read); 
			if (elementsRead != 2300) {
				pros::lcd::set_text(2, "Error reading data from file!");
				return;
			}
			std::fclose(usd_file_read); 
			// Sets individual arrays to the values in the file
			for (int i = 0; i < 750; i++) {
				drivetrainControl[i] = buf[i];
				drivetrainControl[i + 750] = buf[i + 750];
			}
			for (int i = 0; i < 750; i++) {
				intakeControl[i] = buf[i + 1500];
			}
			for (int i = 0; i < 50; i++) {
				goalClampControlArray[i] = buf[i + 2250];
			}

		} else if (runStatus == STATUS_REPLAYING && time < 750) {
			// Replaying ------------
			left = drivetrainControl[time];
			right = drivetrainControl[time + 750];
			intakeDirection = intakeControl[time];
			if (time == goalClampControlArray[nextGoalClampIndex]) {
				goalClampControl = !goalClampControl;
				nextGoalClampIndex++;
			}

			time++;
		} else if (runStatus == STATUS_REPLAYING && time >= 750) {
			// End replay
			runStatus = STATUS_DRIVING;
			pros::lcd::set_text(0, "Driving");
		}

		// This is when the robot is not countdowning (don't know if thats even a word)
		if (runStatus != STATUS_RECORD_COUNTDOWN && runStatus != STATUS_REPLAY_COUTNDOWN) {
			if (left < -driveDeadzone || left > driveDeadzone) {		// Moves the motor groups, brake if inside deadzone
				left_mg.move(left);	
			} else {
				left_mg.brake();
				left = 0;
			}
			if (right < -driveDeadzone || right > driveDeadzone) {
				right_mg.move(right);
			} else {
				right_mg.brake();
				right = 0;
			}
			intake.move(intakeDirection * 127);		// Moves the intake and roller
			ramp.move(intakeDirection * 127);
			goalClamp.set_value(goalClampControl);	// Moves the goal clamp
		}

		pros::lcd::set_text(1, "Time " + std::to_string(time));
		lastGoalClamp = goalClampControl;
		pros::delay(20);
	}
}


// void opcontrol() {
// 	return;
// 	pros::Controller master(pros::E_CONTROLLER_MASTER);
// 	pros::MotorGroup left_mg({-20, -19});
// 	pros::MotorGroup right_mg({18, 17});
// 	pros::Motor roller(1);
// 	pros::Motor intake(-10);
// 	pros::adi::DigitalOut goalClamp('A');
// 	int drive_deadzone = 20;

// 	int drivetrainControl[1500];
// 	int subsystemControl[760];	// 0 -> 750 is intake, 751 -> 760 is goalClamp
// 	int goalClampControli = 0;
// 	bool goalClampReplaySetting = false;
// 	int replaySaveSlot = 0;

// 	DriveMode drive_mode = DRIVE_MODE_ARCADE;
// 	float interpolate_strength = 0.2;
// 	int last_turn = 0;
// 	bool lastGoalClamp = false;
// 	int i = 0;
// 	bool replay = false;
// 	bool recording = false;

// 	while (true) {
// 		int left;
// 		int right;
// 		switch (drive_mode) {		// Sets the left and right motor values based on the drive mode
// 			case DRIVE_MODE_TANK:
// 				left = master.get_analog(ANALOG_LEFT_Y);
// 				right = master.get_analog(ANALOG_RIGHT_Y);
// 				break;
// 			case DRIVE_MODE_ARCADE:
// 				int direction = master.get_analog(ANALOG_LEFT_Y);
// 				int turn = interpolate(last_turn, master.get_analog(ANALOG_RIGHT_X) * -2, interpolate_strength);
// 				last_turn = turn;
// 				left = std::clamp(direction + turn, -127, 127);
// 				right = std::clamp(direction - turn, -127, 127);
// 				break;
// 		}

// 		if (!recording && !replay && master.get_digital(DIGITAL_A)) {
// 			// Start recording  (calls when recording is started)
// 			i = 0;
// 			replay = false;
// 			recording = true;
// 		} else if (recording && i > 750) {
// 			// Stops recording (called when recording is stopped)
// 			recording = false;

// 			// Writes the file to disk
// 			FILE* usd_file_write = fopen("/usd/replay_drivetrain.bin", "wb");
// 			FILE* usd_file_write_subsystem = fopen("/usd/replay_subsystem.bin", "wb");
// 			if (usd_file_write == nullptr || usd_file_write_subsystem == nullptr) {
// 				pros::lcd::set_text(2, "Failed to open file");
// 				return;
// 			}

// 			size_t elementsWrittenSubsystem = std::fwrite(subsystemControl, sizeof(int), 760, usd_file_write);
// 			size_t elementsWritten = std::fwrite(drivetrainControl, sizeof(int), 1500, usd_file_write);
// 			if (elementsWritten != 1500 || elementsWrittenSubsystem != 760) {
// 				pros::lcd::set_text(2, "Error writing data to file!");
// 				return;
// 			} else {
// 				pros::lcd::set_text(2, "Array written to file successfully!");
// 			}
// 			std::fclose(usd_file_write);
// 			std::fclose(usd_file_write_subsystem);
// 		}
// 		if (!recording && !replay && master.get_digital(DIGITAL_B)) {
// 			// Start replay (called when replay is started)
// 			goalClampReplaySetting = false;
// 			goalClampControli = 0;

// 			// Loads the files from storage
// 			FILE* usd_file_read = fopen("/usd/replay_drivetrain.bin", "r");
// 			FILE* usd_file_read_subsystem = fopen("/usd/replay_subsystem.bin", "r");
// 			if (usd_file_read == nullptr || usd_file_read_subsystem == nullptr) {
// 				pros::lcd::set_text(2, "Failed to open read file");
// 				return;
// 			}
// 			std::fread(subsystemControl, sizeof(int), 760, usd_file_read_subsystem);
// 			std::fread(drivetrainControl, sizeof(int), 1500, usd_file_read); 
// 			std::fclose(usd_file_read); 
// 			std::fclose(usd_file_read_subsystem);
// 			i = 0;
// 			replay = true;
// 		} else if (replay && i > 750) {
// 			// Stops replay (called when replay is stopped)
// 			replay = false;
// 		}

// 		int intakeDirection;
// 		bool goalClampSetting = master.get_digital(DIGITAL_R1);
// 		if (!replay) {
// 			// This is when the robot is just driving or recording
// 			if (left < -drive_deadzone || left > drive_deadzone) {		// Moves the motor groups, brake if inside deadzone
// 				left_mg.move(left);	
// 			} else {
// 				left_mg.brake();
// 				left = 0;
// 			}
// 			if (right < -drive_deadzone || right > drive_deadzone) {
// 				right_mg.move(right);
// 			} else {
// 				right_mg.brake();
// 				right = 0;
// 			}
// 			if (master.get_digital(DIGITAL_L1)) {
// 				intakeDirection = 1;
// 			} else if (master.get_digital(DIGITAL_L2)) {
// 				intakeDirection = -1;
// 			} else {
// 				intakeDirection = 0;
// 			}
// 			intake.move(intakeDirection * 127);
// 			roller.move(intakeDirection * 127);
// 			goalClamp.set_value(goalClampSetting);
// 		}

// 		if (recording && i <= 750) {
// 			// This is when the robot is recording
// 			drivetrainControl[i] = left;
// 			drivetrainControl[i + 750] = right;
// 			subsystemControl[i] = intakeDirection;
// 			if (goalClampSetting != lastGoalClamp) {
// 				subsystemControl[goalClampControli + 750] = i;
// 				goalClampControli++;
// 			}
// 			i++;
// 		} else if (replay && i <= 750) {
// 			// This is when the robot is replaying
// 			left_mg.move(drivetrainControl[i]);	
// 			right_mg.move(drivetrainControl[i + 750]);
// 			intake.move(subsystemControl[i] * 127);
// 			roller.move(subsystemControl[i] * 127);
// 			if (i == subsystemControl[goalClampControli + 750]) {
// 				goalClampReplaySetting = !goalClampReplaySetting;
// 				goalClampControli++;
// 			}
// 			goalClamp.set_value(goalClampReplaySetting);
// 			i++;
// 		}
// 		pros::lcd::set_text(2, std::to_string(i) + " " + std::to_string(left) + " " + std::to_string(right));
// 		if (recording) {
// 			pros::lcd::set_text(3, "Recording");
// 		} else if (replay) {
// 			pros::lcd::set_text(3, "Replaying");
// 		} else {
// 			pros::lcd::set_text(3, "Driving");
// 		}
		
// 		lastGoalClamp = goalClampSetting;
// 		pros::delay(20);
// 	}
// }
