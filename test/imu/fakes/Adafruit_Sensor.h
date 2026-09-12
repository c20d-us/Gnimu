#pragma once
struct sensors_vec_t { float x, y, z; };
struct sensors_event_t { sensors_vec_t acceleration; sensors_vec_t gyro; float temperature; };
