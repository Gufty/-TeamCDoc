#include "main.h"
#include "systems/DriveTrain.hpp"
#include "systems/Indexer.hpp"
#include "systems/Extender.hpp"
#include "autonomous/Odometry.hpp"


using namespace pros;
using namespace Display;

Controller master(E_CONTROLLER_MASTER);

DriveTrain dt = DriveTrain();
Indexer index = Indexer();
Extender xtend = Extender();

LV_IMG_DECLARE(normal);
lv_obj_t* backgroundImage = lv_img_disp(&normal);

lv_obj_t * odometryInfo = createLabel(lv_scr_act(), 150, 90, 150, 40, "Odom Info");
Odometry odom = Odometry(&dt, &odometryInfo);

bool arcade;

inline lv_res_t toggleMode(lv_obj_t * btn)//press the button on the screen to change to the different drives.
{
    if (arcade) {
	dt.teleMove = [=]{dt.tankDrive(master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y),master.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y));};
    } else {
	dt.teleMove = [=]{dt.arcadeDrive(master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y),master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X));};
    }
	arcade = !arcade;

	btnSetToggled(btn, arcade);

    return LV_RES_OK;
}

bool rightSide;

inline lv_res_t toggleSide(lv_obj_t * btn) {//changes to wheter the robot is on the left or the right
	rightSide = !rightSide;

	btnSetToggled(btn, rightSide);

	return LV_RES_OK;
}

void initialize() {

	lv_obj_t* modeButton = createBtn(lv_scr_act(),  200,  10, 150,  20, "Toggle Mode");
	lv_style_t* modeBtnSty = createBtnStyle(&lv_style_plain, LV_COLOR_MAKE(0, 100, 0), LV_COLOR_MAKE(0, 125, 0), LV_COLOR_MAKE(0, 150, 150), LV_COLOR_MAKE(0, 150, 175), LV_COLOR_MAKE(255, 255, 255));
	setBtnStyle(modeBtnSty, modeButton);
	lv_btn_set_action(modeButton, LV_BTN_ACTION_CLICK, toggleMode);
	//area that the button is used
	
	lv_obj_t* teamButton = createBtn(lv_scr_act(), 200, 50,  150,  20, "Toggle Side");
	lv_style_t* teamBtnSty = createBtnStyle(&lv_style_plain, LV_COLOR_MAKE(0, 0, 255), LV_COLOR_MAKE(0, 0, 125), LV_COLOR_MAKE(255, 0, 0), LV_COLOR_MAKE(125, 0, 0), LV_COLOR_MAKE(255, 255, 255));
	setBtnStyle(teamBtnSty, teamButton);
	lv_btn_set_action(teamButton, LV_BTN_ACTION_CLICK, toggleSide);
	//actually side button, which changes the where the robot knows where it is at which allows it to follow the correct path based on where it is at.
	
	//lv_style_t* textSty = createLabelSty(&lv_style_plain, LV_COLOR_MAKE(0,0,0), LV_COLOR_MAKE(255,255,255), LV_OPA_50);
	//lv_label_set_style(odometryInfo, &textSty[0]);

	dt.teleMove = [=]{dt.tankDrive(master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y),master.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y));};
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
void autonomous() {//is hard coded, change depending on allies
	//odom.loadPath("/usd/points.dat", "/usd/distances.dat");
	//odom.followPath();
	if (rightSide) {
		odom.followPath();
		troll.move(127);
		delay(1000);
		troll.move(0);
	} else {
		dt.tankDrive(-69, -69);
		troll.move(127);
		delay(1000);
		dt.tankDrive(0, 0);
		troll.move(0);
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
	unsigned int startTime = millis();//used mainly for endgame to make sure people do not fat finger endgame
	unsigned int xChangeTime;
	bool xChange = false;
	bool cPass = false;
	while (true) {
		dt.teleMove();//a function that links back to toggle mode, where telemove has been change to arcade and tank drive.

		if (master.get_digital(E_CONTROLLER_DIGITAL_R1)) {troll.move(127);}
		else if (master.get_digital(E_CONTROLLER_DIGITAL_R2)) {troll.move(-127);}
		else {troll.move(0);}

		if(millis()-startTime >= 90000){//checks to see if the 90 seconds has happened if it is true, the driver can launch end game
			if (master.get_digital(E_CONTROLLER_DIGITAL_A)) {xtend.set(true); xChange=true; xChangeTime=millis();}//tells that program that the end game has been launched
			//since it keeps on going trough the while loop, it keeps on checking how much time has passed since the end game has been launch
			if (xChange && millis()-xChangeTime >= 500) {xtend.set(false); xChange=false;}//once 1 and a half seconds have passed, set endgame back to normal and change state back to false
		}

		cPass = !master.get_digital(E_CONTROLLER_DIGITAL_B);

		if (master.get_digital(E_CONTROLLER_DIGITAL_L1)) {cata.move(127,cPass);}
		else if (master.get_digital(E_CONTROLLER_DIGITAL_L2)) {cata.move(-127,cPass);}
		else {cata.move(0, true);}
		
        delay(20);
    }
}
