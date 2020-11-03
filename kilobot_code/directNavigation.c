/* Automatic positioning - Kilobot control software
 * ARK - Demo B
 * author: Andreagiovanni Reina (University of Sheffield) a.reina@sheffield.ac.uk
 */

#include <kilolib.h>
#include <stdlib.h>
//#define DEBUG
//#include <debug.h>
#include <math.h>

/* Enum for different motion types */
typedef enum {
   FORWARD = 0,
   TURN_LEFT = 1,
   TURN_RIGHT = 2,
   STOP = 3,
} motion_t;

/* Enum for boolean flags */
typedef enum {
   false = 0,
   true = 1,
} bool;

/* Debug variables */
//int msgRec[30];
//int msgTime[30];
//int msgCount = 0;

/* current motion type */
motion_t current_motion_type = STOP;

/* counters for motion, turning and broadcasting */
unsigned int turning_ticks = 0;
const uint8_t max_turning_ticks = 160; /* constant to allow a maximum rotation of 180 degrees with \omega=\pi/5 */
const uint16_t max_straight_ticks = 320; /* set the \tau_m period to 2.5 s: n_m = \tau_m/\delta_t = 2.5/(1/32) */
uint32_t last_motion_ticks = 0;

uint32_t last_turn_ticks = 0;
uint32_t turn_ticks = 60;
uint32_t led_ticks_start = 0;
uint32_t led_ticks_length = 1000;
uint32_t led_ticks_length_random = 500;
int led_colour = 0;

/* Variables for Smart Arena messages */
int sa_type = 3;
int sa_payload = 0;
bool new_sa_msg = false;

/*-------------------------------------------------------------------*/
/* Callback function for message reception                           */
/*-------------------------------------------------------------------*/
void rx_message(message_t *msg, distance_measurement_t *d) {
    if (msg->type == 0) {
        // unpack message
        int id1 = msg->data[0] << 2 | (msg->data[1] >> 6);
        int id2 = msg->data[3] << 2 | (msg->data[4] >> 6);
        int id3 = msg->data[6] << 2 | (msg->data[7] >> 6);
        if (id1 == kilo_uid) {
            // unpack type
        	sa_type = msg->data[1] >> 2 & 0x0F;
            // unpack payload 
        	sa_payload = ((msg->data[1]&0b11) << 8) | (msg->data[2]);
        	new_sa_msg = true;
        }
        if (id2 == kilo_uid) {
            // unpack type
        	sa_type = msg->data[4] >> 2 & 0x0F;
            // unpack payload 
        	sa_payload = ((msg->data[4]&0b11)  << 8) | (msg->data[5]);
        	new_sa_msg = true;
        }
        if (id3 == kilo_uid) {
            // unpack type
        	sa_type = msg->data[7] >> 2 & 0x0F;
            // unpack payload 
        	sa_payload = ((msg->data[7]&0b11)  << 8) | (msg->data[8]);
        	new_sa_msg = true;
        	
//        	if (msgCount >= sizeof(msgRec)/sizeof(msgRec[0])){
//        		msgCount = 0;
//        	}
//        	int prevMsg = msgCount-1;
//        	if (prevMsg < 0) prevMsg = sizeof(msgRec)/sizeof(msgRec[0]) -1;
//        	msgTime[msgCount] = kilo_ticks;
//        	msgRec[msgCount] = sa_payload;
//        	msgCount++;
        }
    }
    else if (msg->type == 120) {
        int id = (msg->data[0] << 8) | msg->data[1];
        if (id == kilo_uid) {
            set_color(RGB(0,0,3));
        } else {
            set_color(RGB(3,0,0));
        }
    }
}

/*-------------------------------------------------------------------*/
/* Function for setting the motor speed                              */
/*-------------------------------------------------------------------*/
void set_motion( motion_t new_motion_type ) {
	bool calibrated = true;
	if ( current_motion_type != new_motion_type ){
		switch( new_motion_type ) {
		case FORWARD:
			spinup_motors();
			if (calibrated)
				set_motors(kilo_straight_left,kilo_straight_right);
			else
				set_motors(70,70);
			break;
		case TURN_LEFT:
			spinup_motors();
			if (calibrated)
				set_motors(kilo_turn_left,0);
			else
				set_motors(70,0);
			break;
		case TURN_RIGHT:
			spinup_motors();
			if (calibrated)
				set_motors(0,kilo_turn_right);
			else
				set_motors(0,70);
			break;
		case STOP:
		default:
			set_motors(0,0);
		}
		current_motion_type = new_motion_type;
	}
}

/*-------------------------------------------------------------------*/
/* Function implementing the uncorrelated random walk                */
/*-------------------------------------------------------------------*/
void random_walk(){
   switch( current_motion_type ) {
   case TURN_LEFT:
   case TURN_RIGHT:
      if( kilo_ticks > last_motion_ticks + turning_ticks ) {
         /* start moving forward */
         last_motion_ticks = kilo_ticks;  // fixed time FORWARD
         //	last_motion_ticks = rand() % max_straight_ticks + 1;  // random time FORWARD
         set_motion(FORWARD);
      }
      break;
   case FORWARD:
      if( kilo_ticks > last_motion_ticks + max_straight_ticks ) {
         /* perform a radnom turn */
         last_motion_ticks = kilo_ticks;
         if( rand()%2 ) {
            set_motion(TURN_LEFT);
         }
         else {
            set_motion(TURN_RIGHT);
         }
         turning_ticks = rand()%max_turning_ticks + 1;
      }
      break;
   case STOP:
   default:
      set_motion(STOP);
   }
}

/*-------------------------------------------------------------------*/
/* Function to update colour as a function of the received msg       */
/*-------------------------------------------------------------------*/
void update_colour() {
//	if (sa_type == 1){
//		set_color(RGB(2,0,0));
//	} else
//	if (sa_type == 2){
//		set_color(RGB(0,2,0));
//	} else
//	if (sa_type == 3){
//		set_color(RGB(0,0,2));
//	}
	
	// Read from VS and change LED to reflect value:
	// Sets light according to payload (-1000)
	if (led_colour == 0) {
		//printf("msg payload is 0");
		set_color(RGB(3,3,3));
	} else if ( led_colour == 1 ) {
		set_color(RGB(3,0,0));
	} else if ( led_colour == 2 ) {
		set_color(RGB(0,3,0));
	} else if ( led_colour == 3 ) {
		set_color(RGB(0,0,3));
	} else if ( led_colour == 4 ) {
		set_color(RGB(3,3,3));
	}
}

/*-------------------------------------------------------------------*/
/* Init function                                                     */
/*-------------------------------------------------------------------*/
void setup()
{
	/* Initialise LED and motors */
	set_color(RGB(0,0,0));
	set_motors(0,0);

	/* seed the c RNG well */
	int temp = rand_hard();
	srand(((temp<<8)|rand_hard()));

	/* Initialise motion variables */
	set_motion( STOP );
//	last_motion_ticks = rand() % max_straight_ticks + 1;  // random time FORWARD
	last_motion_ticks = kilo_ticks;  // fixed time FORWARD
	
	new_sa_msg = false;
	
	// clear debug msg array
//	int i;
//	for (i = 0; i < (sizeof(msgRec)/sizeof(msgRec[0])); ++i){
//		msgRec[i] = 0;
//		msgTime[i] = 0;
//	}
}

/*-------------------------------------------------------------------*/
/* Main loop                                                         */
/*-------------------------------------------------------------------*/
void loop() {
	if (new_sa_msg){
		new_sa_msg = false;
		set_motion(sa_type);
		if (sa_type == 0){
			set_color(RGB(3,0,0));
		} else if (sa_type == 1){
			set_color(RGB(0,3,0));	
//			last_turn_ticks = kilo_ticks;
		} else if (sa_type == 2){
			set_color(RGB(0,0,3));
//			last_turn_ticks = kilo_ticks;
		} else if (sa_type == 3){
			set_color(RGB(3,3,3));				
		} else if (sa_type == 4){	
			led_colour = (int)floor(sa_payload/256.0f);
			led_ticks_start = kilo_ticks + ((sa_payload - (256*led_colour) )*3);
			update_colour();				
		} else {
			set_color(RGB(0,0,0));				
		}
		

//		if (sa_type < 0 || sa_type > 3){
//			set_color(RGB(3,0,0));
//		} else {
//			set_motion(sa_type);
//			if (sa_type == 3){
//				set_color(RGB(0,3,0));
//			} else {
//				set_color(RGB(0,0,3));				
//			}
//		}
	} else {
//		if (current_motion_type != STOP && kilo_ticks > last_turn_ticks + turn_ticks) {
//			set_motion( FORWARD );
//			set_color(RGB(3,0,0));
//		}
		if (led_ticks_start > 0 && kilo_ticks > led_ticks_start + led_ticks_length) {
			set_color(RGB(0,0,0));
			//led_ticks_start = 0;
		}
		if (led_ticks_start > 0 && kilo_ticks > led_ticks_start + led_ticks_length + led_ticks_length_random) {
			set_motion( FORWARD );
			set_color(RGB(0,0,3));
			led_ticks_start = 0;
		}
	}
	
//	int i;
//	for (i = 0; i < (sizeof(msgRec)/sizeof(msgRec[0])); ++i){
//		printf("[%d] message n.%d at tick %d: str:%d \n", kilo_uid, i, msgTime[i], msgRec[i] );
//	}
}

int main()
{
  kilo_init();
  //debug_init();
  kilo_message_rx = rx_message;
  kilo_start(setup, loop);
    
  return 0;
}
