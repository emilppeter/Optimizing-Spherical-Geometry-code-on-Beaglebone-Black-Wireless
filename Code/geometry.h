#ifndef GEOMETRY_H
#define GEOMETRY_H
// geometry 

#include "config.h"

/*void Find_Nearest_Waypoint(float cur_pos_lat, float cur_pos_lon, float * distance, float * bearing, 
			   char  * * name);*/

typedef struct {
	float Lat;
	float SinLat;
	float CosLat;
	float Lon;
	char Name[30];
} PT_T;
typedef struct{
	float dist;
	float bearing;
	char * * name;
}final_points;
typedef union{
	float compare_f;
	int compare_i;
}typecast;
void Find_Nearest_Waypoint(PT_T *ref, float * distance, float * bearing, 
			   char  * * name);
void Find_Nearest_Waypoint_Pass(PT_T *ref,float * distance, float * bearing, 
         char  * * name);
//int Find_Nearest_Waypoint_Pass_1(PT_T *ref,float *c, float *max_c);
void Find_Nearest_Waypoint_validation(PT_T *ref, float * distance, float * bearing, 
			   char  * * name);
void test_case(void);
void test_case_validation(void);
void test_case_Pass(void);
//void test_case_Pass_2(float *c, float *max_c, int *closest_i);
#endif
