#ifndef DISPLAY_H
#define DISPLAY_H

#include "display/lvgl.h"
#include "main.h"

void displayInit(void);
void display(void);
void runSelectedAuto(void);
void setCatTemp(int temp);
void setIntakeTemp(int temp);
void setDriveTemp(int temp);

extern std::string teamColor;

extern int displayMode;
extern int selectedProgram;
extern int autoType;
extern bool driverSkillsMode;

extern lv_obj_t * mainMenuScreen;
extern lv_obj_t * selectAutoScreen;
extern lv_obj_t * visionMenu;
extern lv_obj_t * loadScreen;


typedef enum autonomous_type{
    AUTONOMOUS_RED = 0,
    AUTONOMOUS_BLUE = 1,
    AUTONOMOUS_SKILLS = 2
}   autonomous_type_e_t;

#endif