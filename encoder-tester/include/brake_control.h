#pragma once

// ブレーキ解除リレーへの給電指令。brake_release=true は解除側。brake_on とは呼ばない。

void brake_force_off();
void brake_begin();
void brake_apply(bool release_cmd);
int brake_gpio_level();
