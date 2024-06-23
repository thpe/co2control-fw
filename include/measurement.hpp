#pragma once
#include <cstdio>

typedef struct
{
  /** CO2 in ppm */
  float co2;
  /** Temperatur in deg C */
  float temp;
  /** Relative humidity in % */
  float hum;
  /** 5 V system voltage measured */
  float v_sys;
  /** measured cell voltage */
  float v_cell;
} measurement_t;

void print_meas(const measurement_t& meas)
{
  printf("DATA,%f,V,%f,V,%f,ppm,%f,C,%f,%%",meas.v_cell, meas.v_sys, meas.co2, meas.temp, meas.hum);
}
