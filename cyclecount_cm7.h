#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void dwt_access_enable(bool enable);

void reset_cnt();

void start_cnt();

void stop_cnt();

unsigned int getCycles();

#ifdef __cplusplus
}
#endif