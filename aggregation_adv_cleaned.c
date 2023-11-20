#include "contiki.h"
#include <stdio.h> /* For printf() */
#include <math.h>

#include "dev/sht11-sensor.h"
#include "dev/light-sensor.h"

#define MAX_VALUES 12 // Number of values used in aggregation process (K)
#define READINGS_PER_SECOND 2
#define BUFFER_SIZE 12
#define HIGH_THRES 150.0 // stdDev value thres
#define MID_THRES 50.0
#define OUR_PI 3.1415926535

/*----------------------- sensor transfer func -----------------------------------*/

float getLight(void)
{
    float lightData = 1.5 * light_sensor.value(LIGHT_SENSOR_PHOTOSYNTHETIC) / 4096;
    float I = lightData / 100000;
    float light = 0.625 * 1e6 * I * 1000; // you need to implement the transfer function here
    return light;
}

/*----------------------- printf type cast helper func --------------------------*/

int d1(float f) // Integer part
{
    return ((int)f);
}
unsigned int d2(float f) // Fractional part
{
    if (f > 0)
        return (1000 * (f - d1(f)));
    else
        return (1000 * (d1(f) - f));
}

/*----------------------- calculation func -----------------------------------*/

float find_root(float n)
{
    float difference = 0.0;
    float error = 0.001; // error tolerance
    float x = 10.0;      // initial guess
    int i;
    for (i = 0; i < 50; i++) // we can affort looping 50 times
    {
        x = 0.5 * (x + n / x);
        difference = x * x - n;
        if (difference < 0)
            difference = -difference;
        if (difference < error)
            break; // the difference is deemed small enough
    }
    return x;
}

float calculate_mean(float arr[], int n)
{
    float sum = 0;
    int i;
    for (i = 0; i < n; i++)
    {
        sum += arr[i];
    }
    return sum / n;
}

float calculate_stddev(float arr[])
{
    int n = BUFFER_SIZE;
    float mean = calculate_mean(arr, n);
    float sum_squared_diff = 0;
    int i;
    for (i = 0; i < n; i++)
    {
        float diff = arr[i] - mean;
        sum_squared_diff += diff * diff;
    }

    float mean_squared_diff = sum_squared_diff / n;
    float stddev = find_root(mean_squared_diff);

    return stddev;
}

/*----------------------- array func -----------------------------------*/

void deqArray(float arr[])
{
    int i;
    for (i = 0; i < BUFFER_SIZE - 1; i++)
    {
        arr[i] = arr[i + 1];
    }
}

void printArray(float arr[], int n)
{
    int i;
    printf("[");
    for (i = 0; i < n; i++)
    {
        printf("%d.%03u", d1(arr[i]), d2(arr[i]));
        if (i < n - 1)
        {
            printf(", ");
        }
    }
    printf("]\n");
}

/*----------------------- aggregation func -----------------------------------*/

void performPAA(float src[], float dst[], int segments)
{
    int windowSize = BUFFER_SIZE / segments;
    int i;
    for (i = 0; i < segments; i++)
    {
        float windowSum = 0.0;
        int j;
        for (j = i * windowSize; j < (i + 1) * windowSize; j++)
        {
            windowSum += src[j];
        }
        dst[i] = windowSum / windowSize;
    }
    // printArray(dst, segments);
}

/*----------------------- advance feat -----------------------------------*/

float calculateAutocorrelation(float X[], int lag, float mean, float stdDev)
{
    float sum = 0.0;
    float denominator = (BUFFER_SIZE - lag) * stdDev * stdDev;

    int t;
    for (t = 1; t <= BUFFER_SIZE - lag; t++)
    {
        sum += (X[t] - mean) * (X[t + lag] - mean);
    }

    return sum / denominator;
}

float estimateCos(float x, int terms)
{ // taylor series
    float result = 1.0;
    float term = 1.0;
    int i;

    for (i = 1; i < terms; i++)
    {
        term *= -x * x / (2 * i * (2 * i - 1));
        result += term;
    }

    return result;
}

float calculateDCT(float input[], int l)
{
    int k;
    float x;
    float sum = 0.0;

    for (k = 0; k < BUFFER_SIZE; k++)
    {
        x = (OUR_PI / BUFFER_SIZE) * (k + 0.5) * (l + 0.5);
        sum += input[k] * estimateCos(x, 3);
    }

    return sum / BUFFER_SIZE;
}
/*----------------------- end of func def -----------------------------------*/

/*------------------------------program starts-------------------------------*/

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
    static float autocorr[BUFFER_SIZE];
    static float dct[BUFFER_SIZE];
    int segments, k, l;

    // Any process must start with this.
    PROCESS_BEGIN();

    // Initialise the temperature sensor
    SENSORS_ACTIVATE(light_sensor); // need this for sky-mote emulation
    SENSORS_ACTIVATE(sht11_sensor);

    // Initialise variables
    count = 0;

    // Set the etimer module to generate a periodic event
    etimer_set(&timer, CLOCK_CONF_SECOND / READINGS_PER_SECOND);

    while (1)
    {
        // Wait here for the timer to expire
        PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);

        // this is reading process process
        count++;
        float lightVal = getLight();
        // printf("count: %d\n", count-1);
        printf("reading = %d.%03u\n", d1(lightVal), d2(lightVal));
        if (mean > 0)
        {
            deqArray(buffer);
            buffer[11] = lightVal;
        }
        else
        {
            buffer[count - 1] = lightVal;
        }
        // printf("check buffer[11]: %d.%03u\n", d1(buffer[11]),d2(buffer[11]));
        // printArray(buffer, BUFFER_SIZE);
        // printf("size of buf: %d\n", sizeof(buffer));

        // Check if enough samples are collected
        if (count >= MAX_VALUES && buffer[11] > 0)
        {
            printf("\nB = ");
            printArray(buffer, BUFFER_SIZE);

            // calculate StdDev
            mean = calculate_mean(buffer, BUFFER_SIZE);
            stdDev = calculate_stddev(buffer);
            printf("StdDev = %d.%03u\n", d1(stdDev), d2(stdDev));

            // z normalise array
            // normArray(buffer, stdDev, mean);

            // choose aggregation method
            if (stdDev > HIGH_THRES)
            { // high activity
                printf("Aggregation = Nil\n");
                // unnormaliseArray(myData, 12, mean, stdDev);
                printf("X = ");
                printArray(buffer, BUFFER_SIZE);
                printf("\n");
            }
            else if (stdDev > MID_THRES)
            { // some activity
                printf("Aggregation = 4-into-1\n");
                segments = 3;
                float aggData[segments];
                performPAA(buffer, aggData, segments);
                // unnormaliseArray(myData, 3, mean, stdDev);
                printf("X = ");
                printArray(aggData, segments);
                printf("\n");
            }
            else
            { // low activity
                printf("Aggregation = 12-into-1\n");
                segments = 1;
                float aggData[segments];
                performPAA(buffer, aggData, segments);
                // unnormaliseArray(myData, 3, mean, stdDev);
                printf("X = ");
                printArray(aggData, segments);
                printf("\n");
            }

            // calculate autocorrelation for all k = {0,1,2....n-1}
            for (k = 0; k < BUFFER_SIZE; k++)
            {
                autocorr[k] = calculateAutocorrelation(buffer, k, mean, stdDev);
            }
            printf("autocorrelation: ");
            printArray(autocorr, BUFFER_SIZE);

            // discrete consine transform
            for (l = 0; l < BUFFER_SIZE; l++)
            {
                dct[l] = calculateDCT(autocorr, l);
            }
            printf("DCT: ");
            printArray(dct, BUFFER_SIZE);
            printf("\n");

            // Reset variables
            count = 0;

        } // end if

        // Reset the timer so it will generate another event
        etimer_reset(&timer);
    } // end while

    // Any process must end with this, even if it is never reached.
    PROCESS_END();
} // end thread
