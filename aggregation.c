#include "contiki.h"
#include <stdio.h> /* For printf() */
#include <math.h>

#include "dev/sht11-sensor.h"
#include "dev/light-sensor.h"

#define MAX_VALUES  10 // Number of values used in aggregation process (K)
#define READINGS_PER_SECOND 2
#define BUFFER_SIZE 12

/*----------------------- sensor transfer func -----------------------------------*/

float getTemperature(void)
{
  int tempData;

  // NOTE: You only need to use one of the following
  // If you run the code in Cooja Simulator, please remove the second one
  //tempData = sht11_sensor.value(SHT11_SENSOR_TEMP); // For XM1000 mote
  //float temp = 0.01*tempData + (-39.6) ; // mote transfer function

  tempData = sht11_sensor.value(SHT11_SENSOR_TEMP_SKYSIM); // For Cooja Sim
  float temp = 0.04*tempData-39.6; // cooja transfer function
  return temp;
}

float getLight(void)
{
  float   lightData = 1.5*light_sensor.value(LIGHT_SENSOR_PHOTOSYNTHETIC)/4096;
	float I = lightData/100000;
  float light = 0.625*1e6*I*1000; // you need to implement the transfer function here
  return light;
}

/*----------------------- printf type cast helper func --------------------------*/

int d1(float f) // Integer part
{
  return((int)f);
}
unsigned int d2(float f) // Fractional part
{
  if (f>0)
    return(1000*(f-d1(f)));
  else
    return(1000*(d1(f)-f));
}

/*----------------------- calculation func -----------------------------------*/

float find_root(float n){
	float difference = 0.0;
	float error = 0.001;  // error tolerance
	float x = 10.0;       // initial guess
	int   i;
	for (i=0; i<50; i++) // we can affort looping 50 times
	{
	  x = 0.5 * (x + n/x);
	  difference = x*x - n;
	  if (difference<0) difference = -difference;
	  if (difference<error) break; // the difference is deemed small enough
	}
	return x;
}

float calculate_mean(float arr[], int n) {
    double sum = 0;
    int i;
    for (i = 0; i < n; i++) {
        sum += arr[i];
    }
    return sum / n;
}

float calculate_stddev(float arr[]) {
    int n = BUFFER_SIZE;
    float mean = calculate_mean(arr,n);
    float sum_squared_diff = 0;
    int i;
    for (i = 0; i < n; i++) {
        float diff = arr[i] - mean;
        sum_squared_diff += diff * diff;
    }

    float mean_squared_diff = sum_squared_diff / n;
    float stddev = find_root(mean_squared_diff);

    return stddev;
}

/*----------------------- array func -----------------------------------*/

void normArray(float arr[], float stdDev, float mean){ //z normalisation
    int i;
    for (i=0; i<BUFFER_SIZE; i++){
    	arr[i]= (arr[i]-mean) / stdDev;
    }
}

void unnormaliseArray(float *arr, int size, float mean, float stddev) {
    int i;
    for (i = 0; i < size; i++) {
        arr[i] = (arr[i] * stddev) + mean;
    }
}

void clearArray(float arr[]) {
    //int n = sizeof(arr);
    int i;
    if (MAX_VALUES < BUFFER_SIZE){
	    // move elements
	    for (i = 0; i < MAX_VALUES; i++) {
		arr[i] = arr[i+MAX_VALUES]; 
	    }
	    // clear the rest
	    for (i = BUFFER_SIZE - MAX_VALUES; i < BUFFER_SIZE-1; i++) {
        	arr[i] = 0.0;
    	    }
    }
    else{
	for (i = 0; i < BUFFER_SIZE-1; i++) {
		arr[i] = 0.0; 
	}
    }
    arr[BUFFER_SIZE-1] = -1.0;
}

void printArray(float arr[], int n) {
    int i;
    printf("[");
    for (i = 0; i < n; i++) {
        printf("%d.%03u", d1(arr[i]),d2(arr[i]));
        if (i < n - 1) {
            printf(", ");
        }
    }
    printf("]\n");
}

/*----------------------- aggregation func -----------------------------------*/

void array_slice(float src[], float dst[], int start, int end) {
    int i, j = 0;

    // Copy elements from start to end-1 to the destination array
    for (i = start; i < end; i++) {
        dst[j++] = src[i];
    }

}

void performPAA(float src[], float dst[], int segments) {
    int windowSize = BUFFER_SIZE / segments;
    int i;
    for (i = 0; i < segments; i++) {
        float windowSum = 0.0;
        int j;
        for ( j= i * windowSize; j < (i + 1) * windowSize; j++) {
            windowSum += src[j];
        }
        dst[i] = windowSum / windowSize;
    }
    printArray(dst, segments);
}

/*----------------------- end of func def -----------------------------------*/

/*------------------------------program starts-------------------------------*/
static process_event_t event_data_ready; // Application specific event value

/*---------------------------------------------------------------------------*/
PROCESS(read_process, "Reading process");

/* We require the processes to be started automatically */
AUTOSTART_PROCESSES(&read_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(read_process, ev, data)
{
    /* Variables are declared static to ensure their values are kept */
    /* between kernel calls. */
    static struct etimer timer;

    static int count = 0;
    static float stdDev = 0;
    static float mean = 0;

    static float buffer[BUFFER_SIZE];
    buffer[BUFFER_SIZE-1] = -1.0; // indicate array is not filled
    //printf("check buffer[11]: %d.%03u\n", d1(buffer[11]),d2(buffer[11]));

    static float high_thres = 100.0; // stdDev value thres
    static float mid_thres = 50.0;
    int segments;
    int i;

    // Any process must start with this.
    PROCESS_BEGIN();

    // Allocate the required event
    event_data_ready = process_alloc_event();

    // Initialise the temperature sensor
    SENSORS_ACTIVATE(light_sensor);  // need this for sky-mote emulation
    SENSORS_ACTIVATE(sht11_sensor);

    // Initialise variables
    count = 0;

    // Set the etimer module to generate a periodic event
    etimer_set(&timer, CLOCK_CONF_SECOND/READINGS_PER_SECOND);

    while (1)
    {
        // Wait here for the timer to expire
        PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);

        // this is reading process process
       	count++;
  	float lightVal = getLight();
        printf("count: %d\n", count-1);
        printf("[reading process] reading = %d.%03u\n",d1(lightVal),d2(lightVal));
	buffer[count-1] = lightVal;
        //printf("size of buf: %d\n", sizeof(buffer));


        // Check if enough samples are collected
       	if (count==MAX_VALUES && buffer[11] > 0)
       	{
	  printf("\nB = ");
	  printArray(buffer, BUFFER_SIZE);

	  // calculate StdDev
	  mean = calculate_mean(buffer, BUFFER_SIZE);
	  stdDev = calculate_stddev(buffer);
	  printf("StdDev = %d.%03u\n", d1(stdDev),d2(stdDev));

	  // z normalise array
	  // normArray(buffer, stdDev, mean);
	  
	  // choose aggregation method
	  if (stdDev > high_thres){ 	// high activity
 	     printf("Aggregation = Nil\n");
	     //unnormaliseArray(myData, 12, mean, stdDev);
	     printf("X = ");
 	     printArray(buffer, BUFFER_SIZE);
	     printf("\n");
          }
	  else if (stdDev > mid_thres){ // some activity
	     printf("Aggregation = 4-into-1\n");
             segments = 3;
	     float aggData[segments];
             performPAA(buffer, aggData, segments);
	     //unnormaliseArray(myData, 3, mean, stdDev);
	     printf("X = ");
 	     printArray(aggData, 3);
             printf("\n");
	  }
	  else {  		// low activity
  	     printf("Aggregation = 12-into-1\n");
             segments = 1;
	     float aggData[segments];
	     performPAA(buffer, aggData, segments);
	     //unnormaliseArray(myData, 3, mean, stdDev);
	     printf("X = ");
 	     printArray(aggData, 1);
             printf("\n");
	  }
	
	  // Reset variables
       	  count = 0;
	  clearArray(buffer);
	  //free(aggData);

        } // end if

        // Reset the timer so it will generate another event
        etimer_reset(&timer);
     }// end while

     // Any process must end with this, even if it is never reached.
     PROCESS_END();
} // end thread


