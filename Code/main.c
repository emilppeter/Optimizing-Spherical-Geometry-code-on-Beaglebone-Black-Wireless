/*----------------------------------------------------------------------------
 *----------------------------------------------------------------------------*/
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include "geometry.h"
#include "proto.h"

#define TEST1_LAT (45.0)
#define TEST1_LON (79.0)

#define N_TESTS (10000)
#define VALIDATION 0
#define VER 2
extern const PT_T capitals[];
final_points output[100];
final_points exp_output[100];
int length=0;
/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int main (void) {
	float dist, bearing, cur_pos_lat, cur_pos_lon;
	char * name;
	struct timespec start, end;
	uint64_t diff, total=0, min=1234567890;
	int n=0;
	//int closest_i=0;
	cur_pos_lat = TEST1_LAT;
	cur_pos_lon = TEST1_LON;

	/*	printf("Current location is %f deg N, %f deg W\n", cur_pos_lat,
	       cur_pos_lon);
	*/
	for (n=0; n<N_TESTS; n++) 
	{
	  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);
	  /*Find_Nearest_Waypoint(cur_pos_lat, cur_pos_lon,
				&dist, &bearing, &name);*/
	  #if VER==1
	  test_case();
	  #else
	  test_case_Pass();
	  //test_case_Pass_2(c,&max_c,&closest_i);
	  #endif
	  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &end);
  
	  diff = 1000000000 * (end.tv_sec - start.tv_sec) +
	    end.tv_nsec - start.tv_nsec;
	  //	  printf("%2d: %8lu ns\n", n, diff);
	  total += diff;
	  if (diff < min)
	    min = diff;
	}
	#if  VALIDATION
	test_case_validation();
	float sum=0;
	int count=0;	
	printf("Vaildation:\n");
	for (int i=0;i<length;i++)
	{
		if ((exp_output[i].dist-output[i].dist)>=(0.0001*exp_output[i].dist) || 
				(exp_output[i].dist-output[i].dist)<=(-0.0001*exp_output[i].dist))
		{
			printf("Error in distance!Expected:%f Actual:%f Error:%f for %s\n",exp_output[i].dist,output[i].dist,
					(((exp_output[i].dist-output[i].dist)*100)/(exp_output[i].dist)),capitals[i].Name);
			//sum+=(exp_output[i].dist-output[i].dist);
			//count++;
		} 
		if ((exp_output[i].bearing-output[i].bearing)>=0.1 || 
				(exp_output[i].bearing-output[i].bearing)<=-0.1)
		{
			printf("Error in bearing!Expected:%f Actual:%f Error:%f for %s\n",exp_output[i].bearing,output[i].bearing,
					(exp_output[i].bearing-output[i].bearing),capitals[i].Name);
		}  
		if (strcmp(exp_output[i].name,output[i].name))
		{
			printf("Error in name!Expected:%s Actual:%s for %s\n",exp_output[i].name,output[i].name,
					capitals[i].Name);
		}
	}
	//printf("Average error=%f\n",(sum/count));
	#endif
	printf("Output:\n");
	for (int i=0;i<length;i++)
	{
		//printf("%d.Closest waypoint for %s\n",i,capitals[i].Name);
		printf("%d.Closest waypoint for %s is %s. %f km away at bearing %f degrees\n",
	       i,capitals[i].Name,output[i].name, output[i].dist, output[i].bearing);
	}
	printf("Average %10.3f us\n", total/(1000.0*N_TESTS*length));
	printf("Minimum %10.3f us\n",  min/(1000.0*length));
	exit(0);
}
void test_case()
{
	float dist, bearing;
	char * name;
	int i=0;
	while(strcmp(capitals[i].Name, "END"))
	{
		Find_Nearest_Waypoint(&(capitals[i]),&dist, &bearing, &name);
		output[i].dist=dist;
		output[i].bearing=bearing;
		output[i].name=name;
		//printf("Closest waypoint is %s for %s. %f km away at bearing %f degrees\n",
	     //  name,capitals[i].Name,dist, bearing);
		//printf("Average %10.3f us\n", total/(1000.0*N_TESTS));
		//printf("Minimum %10.3f us\n",  min/1000.0);
		i++;
		length=i;
	}
}
void test_case_Pass()
{
	int i=0;
	float dist, bearing;
	char * name;
	int number_of_entries=0;
	while(strcmp(capitals[i].Name, "END"))
	{
		//Find_Nearest_Waypoint(&(capitals[i]),&dist, &bearing, &name);
		//number_of_entries=Find_Nearest_Waypoint_Pass_1(&(capitals[i]),c,max_c);
		Find_Nearest_Waypoint_Pass(&(capitals[i]),&dist,&bearing,&name);
		output[i].dist=dist;
		output[i].bearing=bearing;
		output[i].name=name;
		//printf("Closest waypoint is %s for %s. %f km away at bearing %f degrees\n",
	     //  name,capitals[i].Name,dist, bearing);
		//printf("Average %10.3f us\n", total/(1000.0*N_TESTS));
		//printf("Minimum %10.3f us\n",  min/1000.0);
		i++;
		length=i;
	}
}
void test_case_validation()
{
	float dist, bearing;
	char * name;
	int i=0;
	while(strcmp(capitals[i].Name, "END"))
	{
		Find_Nearest_Waypoint_validation(&(capitals[i]),&dist, &bearing, &name);
		exp_output[i].dist=dist;
		exp_output[i].bearing=bearing;
		exp_output[i].name=name;
		//printf("Closest waypoint is %s for %s. %f km away at bearing %f degrees\n",
	     //  name,capitals[i].Name,dist, bearing);
		//printf("Average %10.3f us\n", total/(1000.0*N_TESTS));
		//printf("Minimum %10.3f us\n",  min/1000.0);
		i++;
	}
}

