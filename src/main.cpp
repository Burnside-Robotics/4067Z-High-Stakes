#include "main.h"


int interpolate(float last, float current, float strength) {
	return last + (current - last) * strength;
}

enum DriveMode {
	DRIVE_MODE_TANK,
	DRIVE_MODE_ARCADE
};

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
	pros::lcd::set_text(1, "Hello PROS User!");
	
	pros::lcd::register_btn1_cb(on_center_button);

	pros::ADIDigitalOut goalClamp('A');
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

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
	FILE* read_file = fopen("/usd/data.txt", "r");
	int drivetrainControl[2][750];
	int buf[1500];
	fread(buf, sizeof(int), 1500, read_file);
	pros::MotorGroup left_mg({-20, -19});
	pros::MotorGroup right_mg({18, 17});

	for (int i = 0; i < 750; i++) {
		drivetrainControl[0][i] = buf[i];
		drivetrainControl[1][i] = buf[i + 750];
	}

	int i = 0;
	while (true) {
		left_mg.move(drivetrainControl[0][i]);	
		right_mg.move(drivetrainControl[1][i]);
		pros::lcd::set_text(2, std::to_string(i) + " " + std::to_string(drivetrainControl[0][i]) + " " + std::to_string(drivetrainControl[1][i]));

		i++;
		pros::delay(20);
	}
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
	pros::Motor roller(1);
	pros::Motor intake(-10);
	pros::adi::DigitalOut goalClamp('A');
	int drive_deadzone = 20;
	int drivetrainControl[2][750];
	DriveMode drive_mode = DRIVE_MODE_TANK;
	float interpolate_strength = 0.2;
	int last_turn = 0;
	int i = 0;
	bool replay = false;
	bool recording = false;

	while (true) {
		int direction = master.get_analog(ANALOG_LEFT_Y);

		int left;
		int right;
		switch (drive_mode) {		// Sets the left and right motor values based on the drive mode
			case DRIVE_MODE_TANK:
				left = master.get_analog(ANALOG_LEFT_Y);
				right = master.get_analog(ANALOG_RIGHT_Y);
				break;
			case DRIVE_MODE_ARCADE:
				int turn = interpolate(last_turn, master.get_analog(ANALOG_RIGHT_X) * -2, interpolate_strength);
				last_turn = turn;
				int left = std::clamp(direction + turn, -127, 127);
				int right = std::clamp(direction - turn, -127, 127);
				break;
		}
		
		int intakeDirection;
		if (master.get_digital(DIGITAL_L1)) {
			intakeDirection = 1;
		} else if (master.get_digital(DIGITAL_L2)) {
			intakeDirection = -1;
		} else {
			intakeDirection = 0;
		}
		intake.move(intakeDirection * 127);
		roller.move(intakeDirection * 127);
		goalClamp.set_value(master.get_digital(DIGITAL_R1));

		if (!recording && !replay && master.get_digital(DIGITAL_A)) {
			// Start recording  (calls when recording is started)
			i = 0;
			replay = false;
			recording = true;
		} else if (recording && i > 750) {
			// Stops recording (called when recording is stopped)
			recording = false;

			FILE* write_file = fopen("/usd/data.txt", "w");
			fwrite(drivetrainControl, sizeof(drivetrainControl), 1, write_file);
			fclose(write_file);
		}
		if (!recording && !replay && master.get_digital(DIGITAL_B)) {
			// Start replay (called when replay is started)
			i = 0;
			replay = true;
		} else if (replay && i > 750) {
			// Stops replay (called when replay is stopped)
			replay = false;
		}

		if (!replay) {
			if (left < -drive_deadzone || left > drive_deadzone) {		// Moves the motor groups, brake if inside deadzone
				left_mg.move(left);	
			} else {
				left_mg.brake();
				left = 0;
			}
			if (right < -drive_deadzone || right > drive_deadzone) {
				right_mg.move(right);
			} else {
				right_mg.brake();
				right = 0;
			}
		}

		if (recording && i <= 750) {
			drivetrainControl[0][i] = left;
			drivetrainControl[1][i] = right;
			i++;
		} else if (replay && i <= 750) {
			left_mg.move(drivetrainControl[0][i]);	
			right_mg.move(drivetrainControl[1][i]);
			i++;
		}
		pros::lcd::set_text(2, std::to_string(i) + " " + std::to_string(left) + " " + std::to_string(right));
		if (recording) {
			pros::lcd::set_text(3, "Recording");
		} else if (replay) {
			pros::lcd::set_text(3, "Replaying");
		} else {
			pros::lcd::set_text(3, "Driving");
		}
		
		pros::delay(20);
	}
}
