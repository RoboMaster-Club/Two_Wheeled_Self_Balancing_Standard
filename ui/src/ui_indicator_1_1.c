//
// Created by RM UI Designer
//

#include "ui_indicator_1_1.h"

#define FRAME_ID 1
#define GROUP_ID 1
#define START_ID 7
#define OBJ_NUM 1
#define FRAME_OBJ_NUM 1

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_indicator_1_1;
ui_interface_line_t *ui_indicator_1_Orientation_Line = (ui_interface_line_t *)&(ui_indicator_1_1.data[0]);

void _ui_init_indicator_1_1() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_indicator_1_1.data[i].figure_name[0] = FRAME_ID;
        ui_indicator_1_1.data[i].figure_name[1] = GROUP_ID;
        ui_indicator_1_1.data[i].figure_name[2] = i + START_ID;
        ui_indicator_1_1.data[i].operate_tpyel = 1;
    }
    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
        ui_indicator_1_1.data[i].operate_tpyel = 0;
    }

    ui_indicator_1_Orientation_Line->figure_tpye = 0;
    ui_indicator_1_Orientation_Line->layer = 1;
    ui_indicator_1_Orientation_Line->start_x = 1700;
    ui_indicator_1_Orientation_Line->start_y = 340;
    ui_indicator_1_Orientation_Line->end_x = 1700;
    ui_indicator_1_Orientation_Line->end_y = 620;
    ui_indicator_1_Orientation_Line->color = 6;
    ui_indicator_1_Orientation_Line->width = 10;


    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_indicator_1_1);
    SEND_MESSAGE((uint8_t *) &ui_indicator_1_1, sizeof(ui_indicator_1_1));
}

void _ui_update_indicator_1_1() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_indicator_1_1.data[i].operate_tpyel = 2;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_indicator_1_1);
    SEND_MESSAGE((uint8_t *) &ui_indicator_1_1, sizeof(ui_indicator_1_1));
}

void _ui_remove_indicator_1_1() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_indicator_1_1.data[i].operate_tpyel = 3;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_indicator_1_1);
    SEND_MESSAGE((uint8_t *) &ui_indicator_1_1, sizeof(ui_indicator_1_1));
}
