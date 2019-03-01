#include <Arduino_FreeRTOS.h>
#include <semphr.h> 

struct State{
  byte LightsOn;
  int Time;
};

// Define tasks for TaskSouthLight and TaskWestLight
void TaskSouthLight( void *pvParameters );
void TaskWestLight( void *pvParameters );
SemaphoreHandle_t xSerialSemaphore; //Declare semaphore handle that we will use to manage a serial port.

int buttonPin = 31; // Assign 31 to the variable buttonPin

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  
  pinMode(buttonPin, INPUT_PULLUP); //Configure our buttonPin to input pullup
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }

  DDRA = B11111111;  // Set PIN 22 - 29 to Output

  if ( xSerialSemaphore == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  {
    xSerialSemaphore = xSemaphoreCreateMutex();  // Create a semaphore we will use to manage the Serial Port
    if ( ( xSerialSemaphore ) != NULL )
      xSemaphoreGive( ( xSerialSemaphore ) );  // Make the Serial Port available for use, by "Giving" the Semaphore.
  }
  
  // Now set up two tasks to run independently.
  xTaskCreate(
    TaskWestLight
    ,  (const portCHAR *)"WestLight"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

  xTaskCreate(
    TaskSouthLight
    ,  (const portCHAR *) "SouthLight"
    ,  128  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL );

  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/
void TaskSouthLight(void *pvParameters)  // This is a task.
{
  
  (void) pvParameters;
    
  typedef const struct State States;
  
  States SouthLights[] = {
    {B10100100, 5000}, //Set all red lights to high
    {B10100110, 2000}, //Set West Red, West Pedestrian Red , South Red and South Yellow to high
    {B10100001, 5000}, //Set West Red, West Pedestrian Red and South Green to high
    {B10100010, 2000}, //Set West Red, West Pedestrian Red and South Yellow to high 
    };
  for (;;) // A Task shall never return or exit.
  {
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE ) //Check every 5 ticks if the Semaphore is available to take if it's not true.
    {
      Serial.println("Task South Light started...");
      for(int i = 0; i <= 3; i++) // Iterate through the states
      {
        PORTA = (SouthLights[i].LightsOn); //Send bits to PORTA on the Arduino Board. PORT A is Digital Port 22-29
        vTaskDelay(SouthLights[i].Time / portTICK_PERIOD_MS ); // Wait for amount of seconds declared in time of the state
      }
      xSemaphoreGive(xSerialSemaphore); //Give the Semaphore to other tasks.
    }
    vTaskDelay(1);
  }
}

void TaskWestLight(void *pvParameters)  // This is a task.
{  
  (void) pvParameters;
  static bool ButtonPushed = false;
  typedef const struct State States;
  
  States WestLights[] = {
    {B10100100, 5000}, //Set all red lights to high
    {B10110100, 2000}, //Set South Red, West Pedestrian Red, West Red and North Yellow to high
    {B10001100, 5000}, //Set South Red, West Pedestrian Red and West Green to high
    {B10010100, 2000}, //Set South Red, West Pedestrian Red and West Yellow to high
    {B01001100, 5000} //Set South Red and West Green to high + pedestrian crossing to green 
    };

  for (;;) // A Task shall never return or exit.
  {
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE ) //Check every 5 ticks if the Semaphore is available to take if it's not true.
    {
      Serial.println("Task West Light started...");
      for(int i = 0; i <= 3; i++) // Iterate through the states
      {
        if(i == 2 && ButtonPushed == true) //If the light is green for cars
        {
          PORTA = (WestLights[4].LightsOn); //Send bits to PORTA on the Arduino Board. PORT A is Digital Port 22-29
          vTaskDelay(WestLights[4].Time / portTICK_PERIOD_MS ); // Wait for amount of seconds declared in time of the state
          ButtonPushed = false;
        }
        else{
          PORTA = (WestLights[i].LightsOn); //Send bits to PORTA on the Arduino Board. PORT A is Digital Port 22-29
          vTaskDelay(WestLights[i].Time / portTICK_PERIOD_MS ); // Wait for amount of seconds declared in time of the state
        } 
      }
      xSemaphoreGive(xSerialSemaphore); //Give the Semaphore to other tasks.
    }
    else //If the task doesn't run
    {
      int buttonState = 0;         // Variable for reading the button status
      buttonState = digitalRead(buttonPin);
      if (buttonState == LOW)
      {
        ButtonPushed = true;
        Serial.println("Button was pushed");
      }
    }
    vTaskDelay(1);
  }
}
