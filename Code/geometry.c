#include "geometry.h"
#include <math.h>
#include <string.h>
#include <time.h>
#include <stdio.h>
#include "proto.h"

#define PI 3.14159265f
#define PI_OVER_180 (0.017453293f) // (3.1415927/180.0)

extern const PT_T waypoints[];
typecast compare;

#define Epsilon 0.075557

/*
11: radians in table, precalc'd sin, cos
12: Calc_Closeness
13: Don't do bearing
*/

// Table holds precalculated sin/cos for p2. Table Lat/Lon values are in radians

float Calc_Bearing( PT_T * p1,  const PT_T * p2){
  // calculates bearing in degrees between locations (represented in radians)
  float term1, term2;
  float angle;

  term1 = sinf(p1->Lon - p2->Lon)*p2->CosLat;
  term2 = p1->CosLat*p2->SinLat -
    p1->SinLat*p2->CosLat*cosf(p1->Lon - p2->Lon);
  angle = atan2f(term1, term2) * (180/PI);
  if (angle < 0.0)
    angle += 360;
  return angle;
}
float Calc_Bearing_validation( PT_T * p1,  const PT_T * p2){
  // calculates bearing in degrees between locations (represented in radians)
  float term1, term2;
  float angle;

  term1 = sin(p1->Lon - p2->Lon)*p2->CosLat;
  term2 = p1->CosLat*p2->SinLat -
    p1->SinLat*p2->CosLat*cos(p1->Lon - p2->Lon);
  angle = atan2(term1, term2) * (180/PI);
  if (angle < 0.0)
    angle += 360;
  return angle;
}
float Calc_Closeness_accurate( PT_T * p1,  const PT_T * p2) { 
  // calculates closeness (decreases as distance increases) of locations

  return p1->SinLat * p2->SinLat +
    p1->CosLat * p2->CosLat*
    cos_73(p2->Lon - p1->Lon);
}
float Calc_Closeness_fastest( PT_T * p1,  const PT_T * p2) { 
  // calculates closeness (decreases as distance increases) of locations

  return p1->SinLat * p2->SinLat +
    p1->CosLat * p2->CosLat*
    cos_12(p2->Lon - p1->Lon);
}
float Calc_Closeness_validation( PT_T * p1,  const PT_T * p2) { 
  // calculates closeness (decreases as distance increases) of locations

  return p1->SinLat * p2->SinLat +
    p1->CosLat * p2->CosLat*
    cos(p2->Lon - p1->Lon);
}
void Find_Nearest_Waypoint(PT_T *ref, float * distance, float * bearing, 
         char  * * name){
  // cur_pos_lat and cur_pos_lon are in degrees
  // distance is in kilometers
  // bearing is in degrees

  int i=0, closest_i=0;
  //PT_T ref;
  float d, b, c, max_c=0, closest_d=1E10;
  struct timespec prestart, start, end1, end2;
  unsigned to;
  *distance = 0.0f;
  *bearing = 0;
  *name = '\0';


while (strcmp(waypoints[i].Name, "END")) {
//    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);
    c = Calc_Closeness_accurate(ref, &(waypoints[i]) );
 //   clock_gettime(CLOCK_THREAD_CPUTIME_ID, &end1);
    if (c>max_c) {
      max_c = c;
      closest_i = i;
    }
 //   clock_gettime(CLOCK_THREAD_CPUTIME_ID, &end2);
    i++;
  }

/*
  printf("Start to End 1: %d\t", end1.tv_nsec - start.tv_nsec - to);
  printf("Start to End 2: %d\n", end2.tv_nsec - start.tv_nsec - to);
*/
  // Finish calcuations for the closest point
  d = acosf(max_c)*6371; // finish distance calcuation
  b = Calc_Bearing(ref, &(waypoints[closest_i]) );
  // return information to calling function about closest waypoint 
  *distance = d;
  *bearing = b;
  *name = (char * ) (waypoints[closest_i].Name);
}
int Find_Nearest_Waypoint_Pass_1(PT_T *ref,float *c, float *max_c){
  // cur_pos_lat and cur_pos_lon are in degrees
  // distance is in kilometers
  // bearing is in degrees

  int i=0;
  struct timespec prestart, start, end1, end2;
  unsigned to;
  *max_c=0;
while (strcmp(waypoints[i].Name, "END")) {

    c[i] = Calc_Closeness_fastest(ref, &(waypoints[i]) );
    if (c[i]>*max_c) {
      *max_c = c[i];
    }
    i++;
  }
  return i;
}
void Find_Nearest_Waypoint_Pass(PT_T *ref,float * distance, float * bearing, 
         char  * * name){
	int i=0,m=0,closest_i2=0,count=0;
  	float d, b,max_c=0,max_c2=0,upper_bound,lower_bound;
  	*distance = 0.0f;
  	*bearing = 0;
  	*name = '\0';
  	float c[200];
  	float c1;
  	float c2;
  	while (strcmp(waypoints[i].Name, "END")) {
	    c[i] = Calc_Closeness_fastest(ref, &(waypoints[i]) );
	    if (c[i]>max_c) {
	      max_c = c[i];
	      closest_i2=i;
   		 }
	  	i++;
	}
	lower_bound=(max_c)*((100-Epsilon)/(100+Epsilon));
	max_c2=max_c;
	for(int j=0;j<i;j++)
	{
		if(c[j]>lower_bound)
		{
			c2 = Calc_Closeness_accurate(ref, &(waypoints[j]));
			if(c2>max_c2)
			{	
				max_c2=c2;
				closest_i2=j;
			}
			//count++;
		}
	}
	//printf("For %s:number of passes is %d\n",ref->Name,count);
  	// Finish calcuations for the closest point
  	d = acosf(max_c2)*6371; // finish distance calcuation
  	b = Calc_Bearing(ref, &(waypoints[closest_i2]) );
   	*distance = d;
  	*bearing = b;
  	*name = (char * ) (waypoints[closest_i2].Name);
}
void Find_Nearest_Waypoint_validation(PT_T *ref, float * distance, float * bearing, 
         char  * * name){
  // cur_pos_lat and cur_pos_lon are in degrees
  // distance is in kilometers
  // bearing is in degrees

  int i=0, closest_i=0;
  //PT_T ref;
  float d, b, c, max_c=0, closest_d=1E10;
  struct timespec prestart, start, end1, end2;
  unsigned to;

  *distance = 0.0f;
  *bearing = 0;
  *name = '\0';

  while (strcmp(waypoints[i].Name, "END")) {
//    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);
    c = Calc_Closeness_validation(ref, &(waypoints[i]) );
 //   clock_gettime(CLOCK_THREAD_CPUTIME_ID, &end1);
    if (c>max_c) {
      max_c = c;
      closest_i = i;
    }
 //   clock_gettime(CLOCK_THREAD_CPUTIME_ID, &end2);
   i++;
  }

/*
  printf("Start to End 1: %d\t", end1.tv_nsec - start.tv_nsec - to);
  printf("Start to End 2: %d\n", end2.tv_nsec - start.tv_nsec - to);
*/
  // Finish calcuations for the closest point

d = acos(max_c)*6371; // finish distance calcuation
b = Calc_Bearing_validation(ref, &(waypoints[closest_i]) );
 // return information to calling function about closest waypoint 
  *distance = d;
  *bearing = b;
  *name = (char * ) (waypoints[closest_i].Name);
}
