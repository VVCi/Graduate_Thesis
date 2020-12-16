#include "distance.h"

/* Distance */
uint16_t d;

/*
 * Show distance from object to camera (cm)
 * - w: Lenght of object in real size (cm)
 * - p1: Max pixel object (range X)
 * - p2: Min pixel object (range X)
 * - f : Aperture of camera
 * Ex with iPhone object: w = 12, f = 800
*/
uint32_t distance(uint32_t w, uint32_t f, uint32_t p1, uint32_t p2){
	if((p1 > 0) && (p2 > 0) && (p2 > p1)){
		d = (w * f)/(p2 - p1);
		qDebug("Distance: %d\n", d);
		return d;
	}
	else{
		return 0;
	}
}
