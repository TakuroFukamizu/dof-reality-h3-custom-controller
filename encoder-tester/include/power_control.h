#pragma once

// 主電源リレーへの給電指令。GPIO のレベルであり、接点が閉じたことの確認ではない。

void power_force_off();
void power_begin();
void power_apply(bool on_cmd);
int power_gpio_level();
