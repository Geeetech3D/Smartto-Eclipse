

#include "planner.h"
#include "step_motor.h"
#include "adc.h"
#include "setting.h"
#include "delay.h"
#include "command.h"
#include "endstop.h"
#include "fat.h"
#include "rxfile.h"
#include "wifi.h"
#include "sd_print.h"
#include "variable.h"

s32 position[4];   //rescaled from extern when axis_steps_per_unit are changed by gcode
static float previous_vector_factor[3];
static float previous_speed[4]; // Speed of previous path line segment
static float previous_nominal_speed; // Nominal speed of previous path line segment

block_t block_buffer[BLOCK_BUFFER_SIZE];
__IO u8 block_buffer_head; 
__IO u8 block_buffer_tail;          


volatile long count_position[NUM_AXIS] = { 0, 0, 0, 0};

#ifdef DELTA
  extern float delta[3];
#endif

extern char sd_command_buffer[CMDBUF_SIZE];
extern char sd_command_buffer_befor[CMDBUF_SIZE];
extern u8 normal_end_signal[3], touch_end_signal[3], endstop_status[3];                                                                                                              
autohome_st Autohome;
extern void command_process(char* str);


static u8 next_block_index(u8 block_index) 
{
  block_index++;
  if (block_index == BLOCK_BUFFER_SIZE) 
 { 
    block_index = 0; 
  }
  return(block_index);
}

static u8 prev_block_index(u8 block_index) 
{
  if (block_index == 0) 
  { 
    block_index = BLOCK_BUFFER_SIZE; 
  }
  block_index--;
  return(block_index);
}


float estimate_acceleration_distance(float vi, float vt, float a)
{
  if (a!=0)
  {
    return((vt*vt-vi*vi)/(2.0*a));
  }
  else 
  {
    return 0.0; 
  }
}


float intersection_distance(float vi, float vt, float a, float d) 
{
  if (a!=0) 
  {
    return((2.0*a*d-vi*vi+vt*vt)/ (4.0*a) );
  }
  else 
  {
    return 0.0; 
  }
}


void calculate_trapezoid_for_block(block_t *block, float entry_speed, float exit_speed) 
{
	u32 initial_rate = (u32)ceil(entry_speed * block->steps_per_mm); // (step/min)
	u32 final_rate = (u32)ceil(exit_speed* block->steps_per_mm); // (step/min)

	block->entry_speed = entry_speed;
	block->exit_speed = exit_speed;


	s32 acceleration = block->acceleration_st;
	s32 accelerate_steps = (s32)ceil(estimate_acceleration_distance(initial_rate, block->nominal_rate, acceleration));
	s32 decelerate_steps =(s32)floor(estimate_acceleration_distance(block->nominal_rate, final_rate, -acceleration));
	s32 plateau_steps = (s32)block->step_event_count-accelerate_steps-decelerate_steps;

  u16 nominal_timer = (u16)floor(F_TIM1 / (float)block->nominal_rate);
  u16 first_timer = (u16)floor(F_TIM1 / (float)initial_rate);

	if (plateau_steps < 0) 
	{
		accelerate_steps = (s32)ceil(intersection_distance(block->initial_rate, block->final_rate, acceleration, block->step_event_count));
		accelerate_steps = MAX(accelerate_steps,0); 
		accelerate_steps = MIN((u32)accelerate_steps,block->step_event_count);
		plateau_steps = 0;
	}
	if(block->busy == false) 
	{
		block->accelerate_until = accelerate_steps;
		block->decelerate_after = accelerate_steps+plateau_steps;
		block->initial_rate = initial_rate;
		block->final_rate = final_rate;
    block->nominal_timer = nominal_timer;
    block->first_timer = first_timer;
	}
}                    

float max_allowable_speed(const float a, const float v, const float d) 
{
  return  sqrt(v*v-2*a*d);
}



void planner_reverse_pass_kernel(block_t *previous, block_t *current, block_t *next) 
{
  if(!current)
  { 
    return; 
  }

  if (next) 
  {
    if (current->entry_speed != current->max_entry_speed) 
    {
      if ((!current->nominal_length_flag) && (current->max_entry_speed > next->entry_speed)) 
      {
        current->entry_speed = MIN( current->max_entry_speed,max_allowable_speed(-current->acceleration,next->entry_speed,current->millimeters));
      } 
      else 
	   {
        current->entry_speed = current->max_entry_speed;
      }
      current->recalculate_flag = true;
    }
  }
}

// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This 
// implements the reverse pass.
void planner_reverse_pass() 
{
  u8 tail = block_buffer_tail;
  u8 block_index = block_buffer_head;
  block_t *block[3] = {NULL, NULL, NULL};
  while(block_index != tail) 
  { 
    block_index = prev_block_index(block_index); 
    block[2] = block[1];
    block[1] = block[0];
    block[0] = &block_buffer[block_index];
    planner_reverse_pass_kernel(block[0], block[1], block[2]);
  }
}

void planner_forward_pass_kernel(block_t *previous, block_t *current, block_t *next) 
{
  if(!previous) 
  { 
    return; 
  }

  if (!previous->nominal_length_flag) 
  {
    if (previous->entry_speed < current->entry_speed) 
	  {
      float entry_speed = MIN( current->entry_speed,max_allowable_speed(-previous->acceleration,previous->entry_speed,previous->millimeters) );
      if (current->entry_speed != entry_speed) 
	    {
        current->entry_speed = entry_speed;
        current->recalculate_flag = true;
      }
    }
  }
}

// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This 
// implements the forward pass.
void planner_forward_pass() 
{
  u8 block_index = block_buffer_tail;
  block_t *block[3] = { NULL, NULL, NULL };

  while(block_index != block_buffer_head) 
  {
    block[0] = block[1];
    block[1] = block[2];
    block[2] = &block_buffer[block_index];
    planner_forward_pass_kernel(block[0],block[1],block[2]);
    block_index = next_block_index(block_index);
  }
  planner_forward_pass_kernel(block[1], block[2], NULL);
}

// Recalculates the trapezoid speed profiles for all blocks in the plan according to the 
// entry_factor for each junction. Must be called by planner_recalculate() after 
// updating the blocks.
void planner_recalculate_trapezoids(void) 
{
  u8 block_index = block_buffer_tail;
  
  block_t *current = NULL;
  block_t *next = NULL;

  while(block_index != block_buffer_head) 
  {
    current = next;
    next = &block_buffer[block_index];
    
    if(current != NULL) 
    {
      if(current->recalculate_flag || next->recalculate_flag) 
      {
        calculate_trapezoid_for_block(current, current->entry_speed,next->entry_speed);
        current->recalculate_flag = false; 
      }
    }
    block_index = next_block_index( block_index );
  }
  if(next != NULL) 
  {
    calculate_trapezoid_for_block(next, next->entry_speed,next->safe_speed);
    next->recalculate_flag = false;
  }
}


void planner_recalculate(void)
{   
  planner_reverse_pass();
  planner_forward_pass();
  planner_recalculate_trapezoids();
}

void plan_init(void) 
{
  block_buffer_head = 0;
  block_buffer_tail = 0;
  #ifdef DELTA
   for(u8 i=0;i<3;i++)
	position[i] = lround(delta[i]*Setting.steps_per_mm[i]);
  #else
  if(Setting.home_position[X_AXIS] == MINENDSTOP)
      position[X_AXIS] = 0;
  else if(Setting.home_position[X_AXIS] == MAXENDSTOP)
      position[X_AXIS] = lround(Setting.max_position[X_AXIS]*Setting.steps_per_mm[X_AXIS]);
    
  if(Setting.home_position[Y_AXIS] == MINENDSTOP)  
      position[Y_AXIS] = 0;
  else if(Setting.home_position[Y_AXIS] == MAXENDSTOP)   
      position[Y_AXIS] = lround(Setting.max_position[Y_AXIS]*Setting.steps_per_mm[Y_AXIS]);
  
  if(Setting.home_position[Z_AXIS] == MINENDSTOP)
      position[Z_AXIS] = 0;
  else if(Setting.home_position[Z_AXIS] == MAXENDSTOP)
      position[Z_AXIS] = lround(Setting.max_position[Z_AXIS]*Setting.steps_per_mm[Z_AXIS]);
  #endif    
  position[E_AXIS] = 0;
       
  memset(previous_vector_factor,0.0,sizeof(previous_vector_factor));
  previous_speed[0] = 0.0;
  previous_speed[1] = 0.0;
  previous_speed[2] = 0.0;
  previous_speed[3] = 0.0;
  previous_nominal_speed = 0.0;
}
void plan_initXY(void) 
{
  block_buffer_head = 0;
  block_buffer_tail = 0;
  

  
  if(Setting.home_position[X_AXIS] == MINENDSTOP)
      position[X_AXIS] = 0;
  else if(Setting.home_position[X_AXIS] == MAXENDSTOP)
      position[X_AXIS] = lround(Setting.max_position[X_AXIS]*Setting.steps_per_mm[X_AXIS]);
    
  if(Setting.home_position[Y_AXIS] == MINENDSTOP)  
      position[Y_AXIS] = 0;
  else if(Setting.home_position[Y_AXIS] == MAXENDSTOP)   
      position[Y_AXIS] = lround(Setting.max_position[Y_AXIS]*Setting.steps_per_mm[Y_AXIS]);
  
  
  position[E_AXIS] = 0;
  Current_Position[E_AXIS]=0;
  memset(previous_vector_factor,0.0,sizeof(previous_vector_factor));
  previous_speed[0] = 0.0;
  previous_speed[1] = 0.0;
  previous_speed[2] = 0.0;
  previous_speed[3] = 0.0;
  previous_nominal_speed = 0.0;
}

void signal_axis_plan_init(u8 axis)
{
	switch(axis)
	{
		case X_AXIS:       

    	case Y_AXIS:
      		
    	case Z_AXIS: 

			if(Setting.home_position[axis] == MINENDSTOP)
      			position[axis] = 0;
  			else if(Setting.home_position[axis] == MAXENDSTOP)
      			position[axis] = lround(Setting.max_position[axis]*Setting.steps_per_mm[axis]);

			previous_vector_factor[axis] = 0.0;
			previous_speed[axis] = 0.0;
			break;
    	default:
      		break;
	}
}


extern DWORD sd_line_num;
extern DWORD fptr_buf;
extern DWORD sd_byte;
extern char Command_Buffer[1280];//llll
u8 test_times=0;
/**********************************************************
***Function:     Plan_buffer_line
***Description: G0/G1command ;Move command parsed and added to move command block queue
***Input:  x,y,z,e:Moving target;eed_rate:printing speed
***Output: 
***Return:
***********************************************************/
void Plan_buffer_line(float x, float y, float z, const float e, float feed_rate, const uint8_t extruder)
{
    char test_Str[500];
    static char Num_Unexpected_events=0;//Abnormal event
   // u16 text_length;
  u8 i;
  u8 next_buffer_head = next_block_index(block_buffer_head);
  while(block_buffer_tail == next_buffer_head);

  block_t *block = &block_buffer[block_buffer_head];
#if (defined BOARD_A30_MINI_S)
  if(test_times>0)
  {
  	Num_Unexpected_events=0;
  }
#ifdef ENABLE_AUTO_BED_LEVELING
  if(system_infor.Auto_Levele_Flag==1)
  {
 	block->bed_ForCalulate_point_x=x; 
	block->bed_ForCalulate_point_y=y; 
	block->bed_ForCalulate_point_z=z;
   	if(system_infor.auto_leveling_calculate_enable==ENABLE)
  	apply_rotation_xyz(Setting.plan_bed_level_matrix, &x, &y, &z);
  }
  #endif
#endif
  block->point_x = x;
  block->point_y = y;
  block->point_z = z;
  block->point_e = e;
  block->feed_rate = feed_rate;
  block->sd_byte = sd_byte;
  
  block->busy = false;
#ifdef ENABLE_AUTO_BED_LEVELING
  if(system_infor.Auto_Levele_Flag==1)
  {
	float Cal_delta_mm[4];
      u8 last_buffer_head=block_buffer_head;
	if(last_buffer_head==0)
       last_buffer_head=BLOCK_BUFFER_SIZE-1;
	Cal_delta_mm[X_AXIS]=block->bed_ForCalulate_point_x-block_buffer[last_buffer_head].bed_ForCalulate_point_x;
	Cal_delta_mm[Y_AXIS]=block->bed_ForCalulate_point_y-block_buffer[last_buffer_head].bed_ForCalulate_point_y;
	Cal_delta_mm[Z_AXIS]=block->bed_ForCalulate_point_z-block_buffer[last_buffer_head].bed_ForCalulate_point_z;
	Cal_delta_mm[E_AXIS]=block->point_e-block_buffer[last_buffer_head].point_e;
	if((system_infor.sd_print_status==0x02||system_infor.serial_printf_flag == 1)&&system_infor.auto_leveling_calculate_enable==ENABLE)//打印时回抽长度大于15mm，会有退料的风险
	{
		//Abnormal event judgment
       	if((Cal_delta_mm[E_AXIS]<-20.0)||Cal_delta_mm[E_AXIS]>(Setting.max_position[X_AXIS]+Setting.max_position[Y_AXIS])||(Cal_delta_mm[X_AXIS]==0.0&&Cal_delta_mm[Y_AXIS]==0.0&&Cal_delta_mm[E_AXIS]>20.0)||(block->sd_byte>5120&&Cal_delta_mm[Z_AXIS]<-0.5&&block->bed_ForCalulate_point_z>1.5)) 
		{
                        Num_Unexpected_events++;
			if(system_infor.sd_print_status==0x02){// Unexpected_events
 				if(Num_Unexpected_events<8)//8 unusual print events stop printing
                                {
                                	  if(test_times<2)
                                	  {
                                  	system_infor.Unexpected_events=ENABLE;
                                  	printf("erro13,%f,%f,%f,%f,%f,%ld,%f,%f\r\n",Cal_delta_mm[E_AXIS],Setting.max_position[X_AXIS],Setting.max_position[Y_AXIS],Cal_delta_mm[X_AXIS],Cal_delta_mm[Y_AXIS],block->sd_byte,Cal_delta_mm[Z_AXIS],block->bed_ForCalulate_point_z);
	
						
                                  	return; 
                                   }
                                }
                                else
                                {
                                  system_infor.sd_print_status = 0x06;//SD_PRINT_PAUSE_F;
        			  	printf("erro12\r\n");
					return;
                                }
			}
			else if(system_infor.serial_printf_flag == 1)
			{
        			printf("erro12\r\n");
				return; 
			}
		}
          Num_Unexpected_events=0;
    	}
	}
#endif
	s32 target[4];

	target[X_AXIS] = lround(x*Setting.steps_per_mm[X_AXIS]);
	target[Y_AXIS] = lround(y*Setting.steps_per_mm[Y_AXIS]);
	target[Z_AXIS] = lround(z*Setting.steps_per_mm[Z_AXIS]);     
	target[E_AXIS] = lround(e*Setting.steps_per_mm[E_AXIS]);
	
	float delta_mm[4];
#if (defined BOARD_A30_MINI_S)
	 if(test_times>0)
	 {
  		printf("target:%d,Y:%d,Z:%d,,\r\n",target[X_AXIS],target[Y_AXIS],target[Z_AXIS]);
  		printf("position:%d,Y:%d,Z:%d,,\r\n",position[X_AXIS],position[Y_AXIS],position[Z_AXIS]);
  		//test_times--;
  	}
#endif
	delta_mm[X_AXIS] = (float)(target[X_AXIS]-position[X_AXIS])/Setting.steps_per_mm[X_AXIS];
	delta_mm[Y_AXIS] = (float)(target[Y_AXIS]-position[Y_AXIS])/Setting.steps_per_mm[Y_AXIS];
	delta_mm[Z_AXIS] = (float)(target[Z_AXIS]-position[Z_AXIS])/Setting.steps_per_mm[Z_AXIS];
	delta_mm[E_AXIS] = (float)(target[E_AXIS]-position[E_AXIS])/Setting.steps_per_mm[E_AXIS];
	delta_mm[E_AXIS] *= (float)((float)(Setting.extrude_multiply)/100.0);
#if (defined BOARD_A30_MINI_S)
	if(test_times>0)
	 {
  		printf("delta_mm:%f,Y:%f,Z:%f,,\r\n\r\n",delta_mm[X_AXIS],delta_mm[Y_AXIS],delta_mm[Z_AXIS]);
  		test_times--;
  	}
   if(system_infor.Auto_Levele_Flag!=1)
   {
	if(system_infor.sd_print_status==0x02||system_infor.serial_printf_flag == 1) 
	{
	    //Abnormal event judgment
        	if((delta_mm[E_AXIS]<-20.0)||delta_mm[E_AXIS]>(Setting.max_position[X_AXIS]+Setting.max_position[Y_AXIS])||(delta_mm[X_AXIS]==0.0&&delta_mm[Y_AXIS]==0.0&&delta_mm[E_AXIS]>20.0)||(block->sd_byte>5120&&delta_mm[Z_AXIS]<-0.5&&block->point_z>1.5)) 
		{
			sprintf(test_Str, " err: delta:%fmm posX:%dmm posY:%dmm posZ:%dmm posE:%dmm\r\n X:%fmm Y:%fmm Z:%fmm E:%fmm\r\nsd_byte:%d\r\n",\
				delta_mm[E_AXIS],position[X_AXIS],position[Y_AXIS],position[Z_AXIS],position[E_AXIS],block->point_x,block->point_y,block->point_z,block->point_e,block->sd_byte);
			printf("%s",test_Str);
                        Num_Unexpected_events++;
			if(system_infor.sd_print_status==0x02){ 
				if(Num_Unexpected_events<8) 
                                {
                                  system_infor.Unexpected_events=ENABLE;
                                  printf("erro12\r\n");
                                  return; 
                                }
                                else
                                {
                                  system_infor.sd_print_status = 0x06;//SD_PRINT_PAUSE_F;
        			  	printf("erro14\r\n");
					return;
                                }
			}
			else if(system_infor.serial_printf_flag == 1)
			{
        			printf("erro15\r\n");
				return; 
			}
		}
          Num_Unexpected_events=0;
	}

	}
#endif
	block->steps_x = labs(target[X_AXIS]-position[X_AXIS]);
	block->steps_y = labs(target[Y_AXIS]-position[Y_AXIS]);
	block->steps_z = labs(target[Z_AXIS]-position[Z_AXIS]);
	block->steps_e = labs(target[E_AXIS]-position[E_AXIS]);

	block->steps_e *= Setting.extrude_multiply;
	block->steps_e /= 100;

        
    if(block->steps_x<Setting.min_segment_steps && block->steps_y<Setting.min_segment_steps &&  block->steps_z<Setting.min_segment_steps)
	block->step_event_count = MAX(block->steps_x, MAX(block->steps_y, MAX(block->steps_z, block->steps_e)));
    else
      	block->step_event_count = MAX(block->steps_x, MAX(block->steps_y, block->steps_z));

  

  
	if (block->step_event_count <= Setting.min_segment_steps) 
	{
		return; 
	}
 

	block->direction_bits = 0;

	if (target[X_AXIS] < position[X_AXIS])
		block->direction_bits |= (1<<X_AXIS); 
	if (target[Y_AXIS] < position[Y_AXIS])
		block->direction_bits |= (1<<Y_AXIS); 
	if (target[Z_AXIS] < position[Z_AXIS])
		block->direction_bits |= (1<<Z_AXIS); 
	if (target[E_AXIS] < position[E_AXIS])
		block->direction_bits |= (1<<E_AXIS); 
 
  
	block->direction_bits <<= 8;

	block->active_extruder = extruder;


	if(block->steps_x != 0) 
		Enable_X_Axis();
	if(block->steps_y != 0) 
		Enable_Y_Axis();
	if(block->steps_z != 0)
      {   
	  Enable_Z_Axis();
#if (defined BOARD_A30M_Pro_S)  || (defined BOARD_A30D_Pro_S) 
        Enable_E2_Axis() ;
#endif
      }
	if(block->steps_e != 0)
	{
		Enable_E0_Axis();
		Enable_E1_Axis();
		Enable_E2_Axis(); 
	}

	if (block->steps_e == 0)
	{
		if(feed_rate<Setting.min_travel_feedrate) 
			feed_rate=Setting.min_travel_feedrate;
	}
	else
	{
		if(feed_rate<Setting.min_feedrate) 
                {
			feed_rate=Setting.min_feedrate;
                }
	} 

	
	if(   block->steps_x <= Setting.min_segment_steps 
     && block->steps_y <= Setting.min_segment_steps 
     && block->steps_z <= Setting.min_segment_steps )
		block->millimeters = fabs(delta_mm[E_AXIS]);
	else
		block->millimeters = sqrt(  delta_mm[X_AXIS] * delta_mm[X_AXIS]
    					                + delta_mm[Y_AXIS] * delta_mm[Y_AXIS] 
    					                + delta_mm[Z_AXIS] * delta_mm[Z_AXIS]);
	
	
	float inverse_millimeters = 1.0/block->millimeters; 
	
	float inverse_second = feed_rate * inverse_millimeters;

        inverse_second=inverse_second*((float)system_infor.feed_tare)*0.01;
 
  u32 segment_time = (u32)ceil(1000000.0/inverse_second);
  
  u8 tail = block_buffer_tail;
  
	u8 moves_queued=((block_buffer_head - tail + BLOCK_BUFFER_SIZE) & (BLOCK_BUFFER_SIZE - 1));

	

	if ((moves_queued > 1) && (moves_queued < (BLOCK_BUFFER_SIZE >> 1)))
	{
		if (segment_time < Setting.min_segment_time)
			inverse_second=1000000.0/(segment_time+(u32)ceil(2*(Setting.min_segment_time-segment_time)/moves_queued));
	}

	float current_speed[4];
	float speed_factor = 1.0; 

	for(i=0; i < 4; i++)
	{
		current_speed[i] = delta_mm[i] * inverse_second;
		if(fabs(current_speed[i]) > Setting.max_feedrate[i])
		{
 
		  speed_factor = MIN(speed_factor, Setting.max_feedrate[i] / fabs(current_speed[i]));
		}
	}

	block->nominal_speed = block->millimeters * inverse_second; 
	block->nominal_rate = (u32)ceil(block->step_event_count * inverse_second); 

	if( speed_factor < 1.0)
	{
		for(i=0; i < 4; i++)
		{
			current_speed[i] *= speed_factor;
		}
		block->nominal_speed *= speed_factor;
		block->nominal_rate *= speed_factor;
	}


	block->steps_per_mm = (float)block->step_event_count/block->millimeters;

	if(block->steps_x == 0 && block->steps_y == 0 && block->steps_z == 0)
	{
		block->acceleration_st = (u32)ceil(Setting.retract_acceleration * block->steps_per_mm); // convert to: acceleration steps/sec^2
	}
	else
	{
		block->acceleration_st = (u32)ceil(Setting.acceleration * block->steps_per_mm); // convert to: acceleration steps/sec^2
		
		if(((float)block->acceleration_st * (float)block->steps_x / (float)block->step_event_count) > Setting.axis_steps_per_sqr_second[X_AXIS])
			block->acceleration_st = Setting.axis_steps_per_sqr_second[X_AXIS];
		if(((float)block->acceleration_st * (float)block->steps_y / (float)block->step_event_count) > Setting.axis_steps_per_sqr_second[Y_AXIS])
			block->acceleration_st = Setting.axis_steps_per_sqr_second[Y_AXIS];
		if(((float)block->acceleration_st * (float)block->steps_e / (float)block->step_event_count) > Setting.axis_steps_per_sqr_second[E_AXIS])
			block->acceleration_st = Setting.axis_steps_per_sqr_second[E_AXIS];
		if(((float)block->acceleration_st * (float)block->steps_z / (float)block->step_event_count ) > Setting.axis_steps_per_sqr_second[Z_AXIS])
			block->acceleration_st = Setting.axis_steps_per_sqr_second[Z_AXIS];
	}
	block->acceleration = (float)block->acceleration_st / block->steps_per_mm;
  
  block->acceleration_rate = (float)block->acceleration_st / F_TIM1;
	 	
	float vmax_junction = Setting.max_jerk; //Maximum mutation speed
	float vmax_junction_factor = 1.0;             
	float current_vector_factor[3];

	current_vector_factor[X_AXIS] = delta_mm[X_AXIS]*inverse_millimeters;
	current_vector_factor[Y_AXIS] = delta_mm[Y_AXIS]*inverse_millimeters;
	current_vector_factor[Z_AXIS] = delta_mm[Z_AXIS]*inverse_millimeters;

	if((fabs(current_speed[X_AXIS]) > Setting.max_x_jerk)||((current_speed[X_AXIS]*previous_speed[X_AXIS] <= 0)&&(current_speed[X_AXIS]!=previous_speed[X_AXIS])))
		vmax_junction = MIN(vmax_junction, Setting.max_x_jerk);

	if((fabs(current_speed[Y_AXIS]) > Setting.max_y_jerk)||((current_speed[Y_AXIS]*previous_speed[Y_AXIS] <= 0)&&(current_speed[Y_AXIS]!=previous_speed[Y_AXIS])))
		vmax_junction = MIN(vmax_junction, Setting.max_y_jerk);

	if((fabs(current_speed[Z_AXIS]) > Setting.max_z_jerk)||((current_speed[Z_AXIS]*previous_speed[Z_AXIS] <= 0)&&(current_speed[Z_AXIS]!=previous_speed[Z_AXIS])))
		vmax_junction = MIN(vmax_junction, Setting.max_z_jerk);

	if((fabs(current_speed[E_AXIS]) > Setting.max_e_jerk)||((current_speed[E_AXIS]*previous_speed[E_AXIS] <= 0)&&(current_speed[E_AXIS]!=previous_speed[E_AXIS])))
		vmax_junction = MIN(vmax_junction, Setting.max_e_jerk);

	vmax_junction = MIN(vmax_junction, block->nominal_speed);

	block->safe_speed = vmax_junction;

	if((moves_queued > 1)&&(previous_nominal_speed > 0.0)) 
	{
		vmax_junction_factor = current_vector_factor[X_AXIS]*previous_vector_factor[X_AXIS]
		                      +current_vector_factor[Y_AXIS]*previous_vector_factor[Y_AXIS]
		                      +current_vector_factor[Z_AXIS]*previous_vector_factor[Z_AXIS];

		if(vmax_junction_factor < 0) vmax_junction_factor = 0;

		vmax_junction = MAX(block->safe_speed,(vmax_junction_factor*previous_nominal_speed));
		vmax_junction = MIN(vmax_junction,block->nominal_speed);  
	}

	block->max_entry_speed = vmax_junction;

	float v_allowable = max_allowable_speed(-block->acceleration,block->safe_speed,block->millimeters);

  block->entry_speed = MIN(vmax_junction, v_allowable);

	if (block->nominal_speed <= v_allowable) 
	{ 
		block->nominal_length_flag = true; 
	}
	else 
	{ 
		block->nominal_length_flag = false; 
	}
	
	block->recalculate_flag = true; 

	previous_nominal_speed = block->nominal_speed;

	memcpy(previous_speed, current_speed, sizeof(previous_speed)); // previous_speed[] = current_speed[]

	memcpy(previous_vector_factor,current_vector_factor,sizeof(previous_vector_factor));

	memcpy(position, target, sizeof(position)); 
	memcpy(block->position_block, position, sizeof(block->position_block)); //Save the number of steps in the current position
	

	calculate_trapezoid_for_block(block,block->entry_speed, block->safe_speed);

	block_buffer_head = next_buffer_head;
	
	planner_recalculate();
}

void Plan_mix_rate(mixer_t *m)
{
  u8 i;
  u8 next_buffer_head = next_block_index(block_buffer_head);
  
  while(block_buffer_tail == next_buffer_head);

  block_t *block = &block_buffer[block_buffer_head];
  
  block->busy = false;
  block->tools_chang_flag = true;

  //u8 target[EXT_NUM];
  for(i=0;i<EXT_NUM;i++)
  {
    block->mix_rate[i] = m->rate_buf[i];
  }
       
  u8 tail = block_buffer_tail;
  block_buffer_head = next_buffer_head;
}


void Block_clean(void)
{
  memset(block_buffer,0,sizeof(block_t)*BLOCK_BUFFER_SIZE);
  block_buffer_tail = 0;
  block_buffer_head = 0;
}


void Axis_home_control(void)
{
    All_axis_go_origin();
}

static void Axis_go_origin(u8 axis,u8 dir, u16 speed, u8 home, float origin_coordinate )    //axis:   dir:   peed:    home:    origin_coordinate:  
{    	  
    u16 distance;

	
    if((Autohome.count%Autohome.period[axis] == 0)&&(Autohome.flag[axis])) 
    {
   
      dir = dir&0x0001;
      if(home == MAXENDSTOP)
      {
        dir = ~dir;
      }   
      if(axis == X_AXIS) 
      {
         Enable_X_Axis();
         distance = STEPS_PER_mm_FOR_X;
      }
      else if(axis == Y_AXIS) 
      {
         Enable_Y_Axis();
         distance = STEPS_PER_mm_FOR_Y;
      }
      else if(axis == Z_AXIS) 
      {
         Enable_Z_Axis();
#if (defined BOARD_A30M_Pro_S  )  || (defined BOARD_A30D_Pro_S) 
        Enable_E2_Axis() ;
#endif
         distance = STEPS_PER_mm_FOR_Z;
      }    

       if(Autohome.step[axis] == 0)
       {
               
           if(endstop_status[axis] == touch_end_signal[axis])  
          { 
         
              Autohome.step[axis] = 1;
              Autohome.move_distance[axis] = distance*4;   //4
          }  
          else Autohome.step[axis] = 2;
       }
       if(Autohome.step[axis] == 1)  
       {
    
          axis_move(axis, (dir&0x0001), speed,1);
          Autohome.move_distance[axis]--;
          if(Autohome.move_distance[axis] ==0) 
          {
		  	Autohome.step[axis] = 4; 
#if (defined BOARD_A30_MINI_S) || (defined BOARD_A30M_Pro_S)   || (defined BOARD_A30D_Pro_S) 
            Autohome.period[axis] = Autohome.period[axis]*4;
#elif BOARD_E180_MINI_S
			Autohome.period[axis] = Autohome.period[axis]*2;
#elif BOARD_M301_Pro_S
			Autohome.period[axis] = Autohome.period[axis]*6;
#endif
          }
       }
        if(Autohome.step[axis] == 2)
        {
       
           axis_move(axis, ((~dir)&0x0001), speed,1);
           //if(get_endstop_state(axis,home) != normal_end_signal[axis]) 
           if(endstop_status[axis] != normal_end_signal[axis]) 
           {
              Autohome.step[axis] = 3; 
              Autohome.move_distance[axis] = distance*4;
           }
        }
      
       if(Autohome.step[axis] == 3) 
       {
      
          axis_move(axis, (dir&0x0001), speed,1);
          Autohome.move_distance[axis]--;
          if(Autohome.move_distance[axis] ==0) 
          {
            Autohome.step[axis] = 4; 
#if (defined BOARD_A30_MINI_S) || (defined BOARD_A30M_Pro_S)  || (defined BOARD_A30D_Pro_S) 
            Autohome.period[axis] = Autohome.period[axis]*4;
#elif BOARD_E180_MINI_S
			Autohome.period[axis] = Autohome.period[axis]*2;
#elif BOARD_M301_Pro_S
			Autohome.period[axis] = Autohome.period[axis]*6;
#endif
          }
       }
       if(Autohome.step[axis] == 4) 
       {
     
           axis_move(axis, ((~dir)&0x0001), speed,1);
           //if(get_endstop_state(axis,home) != normal_end_signal[axis]) 
           if(endstop_status[axis] != normal_end_signal[axis]) 
           {
              Autohome.step[axis] = 5; 
              Autohome.move_distance[axis] = distance/2;
           } 
       }    
       if(Autohome.step[axis] == 5) 
       {
          
          axis_move(axis, (~dir&0x0001), speed,1);
          Autohome.move_distance[axis]--;
          if(Autohome.move_distance[axis] ==0) 
          {
            Autohome.step[axis] = 0xff;   //
            Autohome.flag[axis] = 0;  
	       axis_known_position[axis]=true;
          }
       }
       
    }
}

static void X_go_origin(float origin_coordinate)
{
	 Axis_go_origin(X_AXIS,Setting.motor_direction[X_AXIS],20,Setting.home_position[X_AXIS],origin_coordinate);
}

/******************************************************************************/

static void Y_go_origin(float origin_coordinate)
{
    Axis_go_origin(Y_AXIS,Setting.motor_direction[Y_AXIS],200,Setting.home_position[Y_AXIS],origin_coordinate);     
}
 /******************************************************************************/
static void Z_go_origin(float origin_coordinate)
{
  Axis_go_origin(Z_AXIS,Setting.motor_direction[Z_AXIS],10,Setting.home_position[Z_AXIS],origin_coordinate); 
}
 
void Signal_axis_go_origin(float origin_coordinate,u8 axis)
{
  //Close_Global_Interrupt();
  TIM_Cmd(TIM8, DISABLE);
  Autohome.period[axis] = (int)(100000/(Setting.steps_per_mm[axis]*Setting.home_speed[axis]));            
  Autohome.step[axis] = 0;              
  Autohome.flag[axis] = 1; 
  Autohome.count = 0;
  Autohome.timer = 899;
  TIM8->ARR = Autohome.timer;
  TIM_Cmd(TIM8, ENABLE);
  switch(axis)
  {
  	
    case X_AXIS:       
      X_go_origin(origin_coordinate);
      break;
    case Y_AXIS:
      Y_go_origin(origin_coordinate);
      break;
    case Z_AXIS:
      Z_go_origin(origin_coordinate);
      break;
    default:
      break;
  }
  //Open_Global_Interrupt();
}

void All_axis_go_origin(void)
{
  TIM_ClearITPendingBit(TIM8,TIM_IT_Update);  
    Enable_X_Axis();
    Enable_Y_Axis();
    Enable_Z_Axis();
#if (defined BOARD_A30M_Pro_S) || (defined BOARD_A30D_Pro_S) 
    Enable_E2_Axis() ;
#endif
  
  //if(Autohome.count%Autohome.period[X_AXIS] == 0)   
    X_go_origin(0.0);
  //if(Autohome.count%Autohome.period[Y_AXIS] == 0)   
    Y_go_origin(0.0);
  //if(Autohome.count%Autohome.period[Z_AXIS] == 0)   
#if (defined BOARD_A30_MINI_S) || (defined BOARD_A30M_Pro_S) || (defined BOARD_A30D_Pro_S) 
  if((Autohome.flag[0]==0) && (Autohome.flag[1]==0))
#endif
    Z_go_origin(0.0);
  previous_nominal_speed = 0.0; 
  previous_speed[0] = 0.0;
  previous_speed[1] = 0.0;
  previous_speed[2] = 0.0;
  previous_speed[3] = 0.0;
  
  //Print_time_C = 0;
  //Print_time_S = 0;
  //Print_time_M = 0;
  //Print_time_H = 0;
  
  //LCD_2004_Init_Print_Pattern();
  //Open_Global_Interrupt();
}


void Single_axis_reset_coordinate(float restcoordinate,char axis)
{
  previous_nominal_speed = 0.0; 
  switch(axis)
  {
    case X_AXIS:
      Current_Position[X_AXIS] = restcoordinate;
      position[X_AXIS] = lround(restcoordinate*Setting.steps_per_mm[X_AXIS]);
      previous_speed[X_AXIS] = 0.0;
      previous_vector_factor[X_AXIS] = 0.0;
      return;
    case Y_AXIS:
      Current_Position[Y_AXIS] = restcoordinate;
      position[Y_AXIS] = lround(restcoordinate*Setting.steps_per_mm[Y_AXIS]);
      previous_speed[Y_AXIS] = 0.0;
      previous_vector_factor[Y_AXIS] = 0.0;
      return;
    case Z_AXIS:
   // printf("KKKKL:ZZ\r\n");
      Current_Position[Z_AXIS] = restcoordinate;
      position[Z_AXIS] = lround(restcoordinate*Setting.steps_per_mm[Z_AXIS]);
      previous_speed[Z_AXIS] = 0.0;
      previous_vector_factor[Z_AXIS] = 0.0;
      return;
    case E_AXIS:
     // printf("KKKKL:EE\r\n");
      Current_Position[E_AXIS] = restcoordinate;
      position[E_AXIS] = lround(restcoordinate*Setting.steps_per_mm[E_AXIS]);
      previous_speed[E_AXIS] = 0.0;
      //int_Log(position[E_AXIS]);
      return;
    default:
      break;
  }
}
/******************************************************************************/

void axis_move(u8 axis, u8 dir, u16 speed, u16 distance)
{
  u16 i;
  for(i=0;i<distance;i++)   //
  {  
    if(dir == POSITIVE)
    {
        switch(axis)
        {
        case 0: X_DIR_L;
                X_STEP_H;
				//delay_us(1);
                X_STEP_L;
                break;
        case 1: Y_DIR_L;
                Y_STEP_H;
				//delay_us(1);
                Y_STEP_L;
                break;
        case 2: 
            Z_DIR_H;
#if (defined BOARD_A30M_Pro_S) || (defined BOARD_A30D_Pro_S) 
            E2_DIR_H; 
#endif
                Z_STEP_H;
#if (defined BOARD_A30M_Pro_S) || (defined BOARD_A30D_Pro_S) 
                E2_STEP_H; 
#endif
				//delay_us(1);
                Z_STEP_L;
#if (defined BOARD_A30M_Pro_S) || (defined BOARD_A30D_Pro_S) 
        E2_STEP_L; 
#endif
                break;
        default: break;
        }
    }
     else
    {
        switch(axis)
        {
        case 0: X_DIR_H;
                X_STEP_H;
				//delay_us(1);
                X_STEP_L;
                break;
        case 1: Y_DIR_H;
                Y_STEP_H;
				//delay_us(1);
                Y_STEP_L;
                break;
        case 2:
                Z_DIR_L;
#if (defined BOARD_A30M_Pro_S) || (defined BOARD_A30D_Pro_S) 
                E2_DIR_L ;
#endif
                Z_STEP_H;
#if (defined BOARD_A30M_Pro_S) || (defined BOARD_A30D_Pro_S) 
                E2_STEP_H; 
#endif
				//delay_us(1);
                Z_STEP_L;
#if (defined BOARD_A30M_Pro_S) || (defined BOARD_A30D_Pro_S) 
                E2_STEP_L; 
#endif
                break;
        }   
    }
    //delay_us(speed);
  }
}



void All_axis_reset_coordinate(void)
{
  Current_Position[X_AXIS] = 0;
  Current_Position[Y_AXIS] = 0;
  Current_Position[Z_AXIS] = 0;
  Current_Position[E_AXIS] = 0;
  position[X_AXIS] = 0;
  position[Y_AXIS] = 0;
  position[Z_AXIS] = 0;
  position[E_AXIS] = 0;
  previous_nominal_speed = 0.0; 
  previous_speed[X_AXIS] = 0.0;
  previous_speed[Y_AXIS] = 0.0;
  previous_speed[Z_AXIS] = 0.0;
  previous_speed[E_AXIS] = 0.0;
  previous_vector_factor[X_AXIS] = 0.0;
  previous_vector_factor[Y_AXIS] = 0.0;
  previous_vector_factor[Z_AXIS] = 0.0;
}

extern char com_cpy[200];
void  Filement_Constol_Function(u8 flag)
{
	static u16 Sent_Times=0;
	if(Sent_Times>2000)
	{
		
	      Sent_Times =0;
		switch(flag)
		{
			case 1:
				//if(Get_Motor_Status()!=0)
				/*Enable_E0_Axis();
				E0_DIR_H;
				E0_STEP_H;
				E0_STEP_L;
				if(Get_Motor_Status()!=0)
					Disable_E0_Axis();*/
				strcpy(com_cpy,"G92 E0.5\r\n");         
            			command_process(com_cpy);
	   			strcpy(com_cpy,"G1 E0 F100\r\n");         
            			command_process(com_cpy);
				break;
			case 2:
				//if(Get_Motor_Status()!=0)
				/*Enable_E0_Axis();
				E0_DIR_L;
				E0_STEP_H;
				E0_STEP_L;
				if(Get_Motor_Status()!=0)
					Disable_E0_Axis();*/
				strcpy(com_cpy,"G92 E-0.5\r\n");         
            			command_process(com_cpy);
	   			strcpy(com_cpy,"G1 E0 F100\r\n");         
            			command_process(com_cpy);
				break;
			default :break;
		}
	}
	else
		Sent_Times++;
}
void st_set_position(const long x, const long y, const long z, const long e)
{
  //CRITICAL_SECTION_START;
  count_position[X_AXIS] = x;
  count_position[Y_AXIS] = y;
  count_position[Z_AXIS] = z;
  count_position[E_AXIS] = e;
  //CRITICAL_SECTION_END;
}

void st_set_e_position(const long e)
{
  //CRITICAL_SECTION_START;
  count_position[E_AXIS] = e;
  //CRITICAL_SECTION_END;
}
void st_set_z_position(const long e)
{
  //CRITICAL_SECTION_START;
  count_position[Z_AXIS] = e;
  //CRITICAL_SECTION_END;
}

long st_get_position(u8 axis)
{
  long count_pos;
  //CRITICAL_SECTION_START;
  count_pos = count_position[axis];
  //CRITICAL_SECTION_END;
  return count_pos;
}

float st_get_position_mm(u8 axis)
{
  float steper_position_in_steps = st_get_position(axis);
 // if(axis==2)
  //{
//	steper_position_in_steps=(Setting.max_position[Z_AXIS]+Setting.zprobe_zoffset)*Setting.steps_per_mm[X_AXIS]-st_get_position(axis);
 // }
  return (steper_position_in_steps / Setting.steps_per_mm[axis]);
}
#ifdef ENABLE_AUTO_BED_LEVELING
void plan_get_position(vector_3 *position) {
	matrix_3x3 inverse;
	
	vector_3_init(position, Current_Position[X_AXIS], Current_Position[Y_AXIS],  Current_Position[Z_AXIS]);

	matrix_3x3_transpose(&inverse, Setting.plan_bed_level_matrix);

	vector_3_apply_rotation(inverse,position);

}
#endif
void plan_set_position(float x, float y, float z, const float e)
{
  //apply_rotation_xyz(Setting.plan_bed_level_matrix, &x, &y, &z);

  position[X_AXIS] = lround(x*Setting.steps_per_mm[X_AXIS]);
  position[Y_AXIS] = lround(y*Setting.steps_per_mm[Y_AXIS]);
  position[Z_AXIS] = lround(z*Setting.steps_per_mm[Z_AXIS]);     
  position[E_AXIS] = lround(e*Setting.steps_per_mm[E_AXIS]);  
  st_set_position(position[X_AXIS], position[Y_AXIS], position[Z_AXIS], position[E_AXIS]); 
  previous_nominal_speed = 0.0; // Resets planner junction speeds. Assumes start from rest.
  previous_speed[0] = 0.0;
  previous_speed[1] = 0.0;
  previous_speed[2] = 0.0;
  previous_speed[3] = 0.0;
}
