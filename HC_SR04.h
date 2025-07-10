#ifndef HC_SR04_H
#define HC_SR04_H

#include "config.h"

void HC_SR04_Init(void);
uint16_t HC_SR04_GetDistance(void);

#endif /* HC_SR04_H */