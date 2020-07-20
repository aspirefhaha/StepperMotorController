/* mbed specific header files. */
#include "mbed.h"

/* Helper header files. */
#include "DevSPI.h"

/* Component specific header files. */
#include "L6474.h"

#include "serialproto/stepmotor/mavlink.h"

/* Initialization parameters. */
L6474_init_t init = {
    400,                              /* Acceleration rate in pps^2. Range: (0..+inf). */
    400,                              /* Deceleration rate in pps^2. Range: (0..+inf). */
    400,                             /* Maximum speed in pps. Range: (30..10000]. */
    10,                              /* Minimum speed in pps. Range: [30..10000). */
    906.25f,                              /* Torque regulation current in mA. Range: 31.25mA to 4000mA. */
	L6474_OCD_TH_6000mA,               /* Overcurrent threshold (OCD_TH register). */
    L6474_CONFIG_OC_SD_ENABLE,        /* Overcurrent shutwdown (OC_SD field of CONFIG register). */
    L6474_CONFIG_EN_TQREG_TVAL_USED,  /* Torque regulation method (EN_TQREG field of CONFIG register). */
	L6474_STEP_SEL_1_2,              /* Step selection (STEP_SEL field of STEP_MODE register). */
    L6474_SYNC_SEL_1_2,               /* Sync selection (SYNC_SEL field of STEP_MODE register). */
    L6474_FAST_STEP_12us,             /* Fall time value (T_FAST field of T_FAST register). Range: 2us to 32us. */
    L6474_TOFF_FAST_8us,              /* Maximum fast decay time (T_OFF field of T_FAST register). Range: 2us to 32us. */
    3,                                /* Minimum ON time in us (TON_MIN register). Range: 0.5us to 64us. */
    21,                               /* Minimum OFF time in us (TOFF_MIN register). Range: 0.5us to 64us. */
    L6474_CONFIG_TOFF_044us,          /* Target Swicthing Period (field TOFF of CONFIG register). */
    L6474_CONFIG_SR_320V_us,          /* Slew rate (POW_SR field of CONFIG register). */
    L6474_CONFIG_INT_16MHZ,           /* Clock setting (OSC_CLK_SEL field of CONFIG register). */
    L6474_ALARM_EN_OVERCURRENT |
    L6474_ALARM_EN_THERMAL_SHUTDOWN |
    L6474_ALARM_EN_THERMAL_WARNING |
    L6474_ALARM_EN_UNDERVOLTAGE |
    L6474_ALARM_EN_SW_TURN_ON |
    L6474_ALARM_EN_WRONG_NPERF_CMD    /* Alarm (ALARM_EN register). */
};

/* Motor Control Component. */
L6474 *motor;

Serial pc(PA_11, PA_12,256000);

InterruptIn up_lock_pos(PB_7);
InterruptIn down_lock_pos(PC_12);
AnalogIn OpAmp(PC_3);

Thread thRead;
Thread thRunCommand;
EventQueue  cmdqueue;
Ticker mytimer;

bool uplock = false;
bool downlock = false;

int upPos = 300;
int downPos = 0;
bool inScanMode = false;

unsigned int mytickercount = 0;

void attime() {
     ++mytickercount;
}

void sm_poststatus()
{
	mavlink_message_t msg;
	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
	mavlink_l6474status_t packet_in ;
	packet_in.status = motor->get_status();
	mavlink_msg_l6474status_encode(1, 1, &msg, &packet_in);
	unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buffer, &msg);
	pc.write((const unsigned char *)buffer,len,NULL);
	printf("Post Status\r\n");
}

void sm_mvfwd()
{
	motor->run(StepperMotor::FWD);
	cmdqueue.call(sm_poststatus);
}

void sm_mvbwd()
{
	motor->run(StepperMotor::BWD);
	cmdqueue.call(sm_poststatus);
}

void sm_hardstop()
{
	motor->hard_stop();
}

void up_lock_pos_rise()
{
	if(up_lock_pos.read() != 0){
		cmdqueue.call(sm_hardstop);
	}
	//if(motor->isInCustomMode2()){
	motor->touchlocked();
	//}
	uplock = true;
}


void down_lock_pos_rise()
{
	if(down_lock_pos.read() != 0){
		cmdqueue.call(sm_hardstop);
	}
	motor->touchlocked();
	downlock= true;
}

void sm_stop()
{
	motor->soft_stop();
	cmdqueue.call(sm_poststatus);
}

void sm_enable()
{
	motor->enable();
	cmdqueue.call(sm_poststatus);
}

void sm_disable()
{
	motor->disable();
	cmdqueue.call(sm_poststatus);
}

void sm_sethome()
{
	motor->set_home();
}

void sm_gopos(int value)
{
	motor->go_to(value);
	cmdqueue.call(sm_poststatus);
}


void sm_setacc(int value)
{
	motor->set_acceleration(value);
}

void sm_setdec(int value)
{
	motor->set_deceleration(value);
}

void sm_gomark()
{
	motor->go_mark();
	cmdqueue.call(sm_poststatus);
}

void sm_setmark(int markpos)
{
	motor->set_mark((markpos & 0x3fffff));
}

void sm_postcust()
{
	mavlink_message_t msg;
	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
	mavlink_custstep_t packet_in ;

	motor->get_cust(&packet_in.wait1,&packet_in.wait2,&packet_in.wait3,&packet_in.wait4,&packet_in.wait5,
			&packet_in.step1,&packet_in.step2,&packet_in.step3,&packet_in.step4,&packet_in.step5);
	mavlink_msg_custstep_encode(1, 1, &msg, &packet_in);
	unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buffer, &msg);
	pc.write((const unsigned char *)buffer,len,NULL);
}

void sm_setmaxspeed(unsigned int maxspeed)
{
	motor->set_max_speed(maxspeed);
}

void sm_setminspeed(unsigned int minspeed)
{
	motor->set_min_speed(minspeed);
}

void sm_setstepmode(unsigned int stepmode)
{

	motor->set_step_mode((StepperMotor::step_mode_t)stepmode);
	cmdqueue.call(sm_poststatus);
}

void sm_settval(unsigned int tval)
{
	motor->set_parameter(L6474_TVAL,(tval+1)*31.25f+2);
}

void sm_step(StepperMotor::direction_t dir,unsigned int st)
{
	motor->move(dir,st);
	cmdqueue.call(sm_poststatus);
}
unsigned char isReboot = 1;
void sm_postconfig()
{
	mavlink_message_t msg;
	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
	mavlink_config_t packet_in;

	packet_in.stepmode = motor->get_parameter(L6474_STEP_MODE);
	packet_in.acc = motor->get_acceleration();
	packet_in.dec = motor->get_deceleration();
	packet_in.config = motor->get_parameter(L6474_CONFIG);
	packet_in.mark = motor->get_mark();
	packet_in.maxspeed = motor->get_max_speed();
	packet_in.minspeed = motor->get_min_speed();
	packet_in.isRboot = isReboot;
	//packet_in.stallth = motor->get_parameter(L6474_RESERVED_REG12);
	packet_in.tval = motor->get_parameter(L6474_TVAL);
	packet_in.ocdth = motor->get_parameter(L6474_OCD_TH);
	packet_in.uppos = upPos;
	packet_in.downpos = downPos;
	mavlink_msg_config_encode(1, 1, &msg, &packet_in);
	unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buffer, &msg);
	// Write buffer to serial port, locks port while writing
	pc.write((const unsigned char *)buffer,len,NULL);
	isReboot = 0;
	//printf("send config len %d ok\r\n",len);

}

void sm_heartbeat()
{
	mavlink_message_t msg;
	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
	mavlink_heartbeat_t packet_in ;
	packet_in.tick = mytickercount;
	packet_in.position = motor->get_position();
	packet_in.dynamic = OpAmp.read()*3.3;
	packet_in.speed = motor->get_speed();
	packet_in.runstatus = motor->get_device_state();
	packet_in.lockstate = 0;
	if(uplock)
		packet_in.lockstate |= 0x1;
	if(downlock)
		packet_in.lockstate |= 0x2;
	mavlink_msg_heartbeat_encode(1, 1, &msg, &packet_in);
	unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buffer, &msg);
	pc.write((const unsigned char *)buffer,len,NULL);
}

void thread_read(void)
{
	while(1){
		cmdqueue.dispatch(15);
		sm_heartbeat();
	}
}

bool runprog1 = false;

void thread_runcommand(void)
{
	while(1){
		if(runprog1){
			for(int i =0 ;i<400;i++){
				cmdqueue.call(sm_step,StepperMotor::FWD,1);
				wait_ms(200);
			}
			runprog1 = false;
		}
		if(inScanMode){
			cmdqueue.call(sm_mvfwd);
			while(!uplock)
				wait_ms(500);
			upPos = motor->get_position();
			wait_ms(3000);
			uplock = false;
			cmdqueue.call(sm_mvbwd);
			while(!downlock){
				wait_ms(500);
			}
			downPos = motor->get_position();
			cmdqueue.call(sm_postconfig);
			wait_ms(3000);
			downlock = false;
			inScanMode = false;
		}
		wait_ms(100);
	}

}

void sm_stopengine()
{
	motor->stopengine();
}

void sm_startengine()
{
	motor->startengine();
}

void sm_runprog1(int val)
{
	//int value  = motor->get_parameter(L6474_INEXISTENT_REG);
	//value = value * 2;
	runprog1 = true;
}

void myL6474_flag_irq()
{
	cmdqueue.call(callback(sm_poststatus));
}


int main()
{
	uint8_t msgReceived = false;
	mavlink_status_t status;
	mavlink_message_t message;
	DevSPI dev_spi(D11, D12, D13);

	/* Initializing Motor Control Component. */
	motor = new L6474(D2, D8, D7, D9, D10, dev_spi);

	if (motor->init(&init) != COMPONENT_OK) {
		exit(EXIT_FAILURE);
	}

	motor->attach_flag_irq(myL6474_flag_irq);
	motor->enable_flag_irq();

	up_lock_pos.mode(PushPullPullDown);
	up_lock_pos.rise(up_lock_pos_rise);
	up_lock_pos.enable_irq();
	down_lock_pos.mode(PushPullPullDown);
	down_lock_pos.rise(down_lock_pos_rise);
	down_lock_pos.enable_irq();

	motor->wait_while_active();

	/* Printing to the console. */
	printf("--> Setting Home.\r\n");

	/* Setting the current position to be the home position. */
	motor->set_home();

	motor->wait_while_active();
	//printf("step mode %x\r\n",(int)motor->get_parameter(L6474_STEP_MODE));

	//motor->run(StepperMotor::FWD);
	//motor->move(StepperMotor::FWD,3200u);
	mytimer.attach_us(&attime, 1000);
	thRead.start(thread_read);
	thRunCommand.start(thread_runcommand);
	cmdqueue.call(callback(sm_postconfig));
	//serialRead.start(serial_read);
	while(1){
		//wait_ms(1);
		char tmpc = pc.getc();
		msgReceived = mavlink_parse_char(MAVLINK_COMM_0,tmpc,&message,&status);
		if(msgReceived){
			switch(message.msgid){
			case MAVLINK_MSG_ID_CUSTSTEP:{
				mavlink_custstep_t custstep;
				mavlink_msg_custstep_decode(&message,&custstep);
				motor->set_cust(custstep.wait1,custstep.wait2,custstep.wait3,custstep.wait4,custstep.wait5,
								custstep.step1,custstep.step2,custstep.step3,custstep.step4,custstep.step5);
				break;
			}
			case MAVLINK_MSG_ID_RUNCMD:{
					mavlink_runcmd_t runcmd;
					mavlink_msg_runcmd_decode(&message,&runcmd);
					switch(runcmd.cmd){
					case SMCMD_STOPENGINE:
					{
						cmdqueue.call(sm_stopengine);
						break;
					}
					case SMCMD_STARTENGINE:
						cmdqueue.call(sm_startengine);
						break;
					case SMCMD_GETCUSTSTEP:
						cmdqueue.call(sm_postcust);
						break;
					case SMCMD_GETCONFIG:
						cmdqueue.call(callback(sm_postconfig));
						printf("Recv Get Config Cmd\r\n");
						break;
					case SMCMD_MOVEFWD:
						if(!uplock && !downlock)
							cmdqueue.call(callback(sm_mvfwd));
						printf("Recv Move Forward Cmd\r\n");
						break;
					case SMCMD_SOFTSTOP:
						cmdqueue.call(callback(sm_stop));
						printf("Recv SoftStop Cmd\r\n");
						break;
					case SMCMD_MOVEBWD:
						if(!uplock && !downlock)
							cmdqueue.call(callback(sm_mvbwd));
						printf("Recv Move Backward Cmd\r\n");
						break;
					case SMCMD_HARDSTOP:
						cmdqueue.call(callback(sm_hardstop));
						printf("Recv HardStop Cmd\r\n");
						break;
					case SMCMD_SETHOME:
						cmdqueue.call(sm_sethome);
						printf("Recv Set Home Cmd \r\n");
						break;
					case SMCMD_ENABLE:
						cmdqueue.call(sm_enable);
						printf("Recv Enable Cmd\r\n");
						break;
					case SMCMD_DISABLE:
						cmdqueue.call(sm_disable);
						printf("Recv Disable Cmd\r\n");
						break;
					case SMCMD_MOVESTEP:
					{
						StepperMotor::direction_t dir;
						unsigned int mstep = 0;
						dir = runcmd.dir == 0? StepperMotor::BWD:StepperMotor::FWD;
						mstep = abs(runcmd.distance);
						if(!uplock && !downlock)
							cmdqueue.call(sm_step,dir,mstep);
						printf("Recv MoveStep  Cmd dir %d step %d \r\n",dir,mstep);
						break;
					}
					case SMCMD_SETMARK:
					{
						int markpos = runcmd.distance;
						cmdqueue.call(sm_setmark,markpos);
						printf("Recv SetMark Cmd %d\r\n",markpos);
						break;
					}
					case SMCMD_UNLOCKUP:
					{
						uplock = false;
						break;
					}
					case SMCMD_UNLOCKDOWN:
					{
						downlock = false;
						break;
					}
					case SMCMD_GOPOS:
					{
						int pos = runcmd.distance;
						if(!uplock && !downlock)
							cmdqueue.call(sm_gopos,pos);
						printf("Recv Go Pos Cmd %d\r\n",pos);
						break;
					}
					case SMCMD_GETSTATUS:
					{
						cmdqueue.call(sm_poststatus);
						printf("Recv GetStatus Cmd\r\n");
						break;
					}
					case SMCMD_SETACC:
					{
						int acc = runcmd.distance;
						if(!uplock && !downlock)
							cmdqueue.call(sm_setacc,acc);
						printf("Recv Set Acc Cmd %d\r\n",acc);
						break;
					}
					case SMCMD_SETDEC:
					{
						int dec = runcmd.distance;
						if(!uplock && !downlock)
							cmdqueue.call(sm_setdec,dec);
						printf("Recv Set Dec Cmd %d\r\n",dec);
						break;
					}
					case SMCMD_GOMARK:
					{
						if(!uplock && !downlock)
							cmdqueue.call(sm_gomark);
						printf("Recv GoMark\r\n");
						break;
					}
					case SMCMD_SETMAXSPEED:
					{
						unsigned int maxspeed = abs(runcmd.distance);
						if(!uplock && !downlock)
							cmdqueue.call(sm_setmaxspeed,maxspeed);
						printf("Recv Set MaxSpeed Cmd %d\r\n",maxspeed);
						break;
					}
					case SMCMD_SETTVAL:
					{
						unsigned int tval = abs(runcmd.distance);
						cmdqueue.call(sm_settval,tval);
						printf("Recv Set Tval Cmd %d\r\n",tval);
						break;
					}
					case SMCMD_SETSTEPMODE:
					{
						unsigned int stepmode = runcmd.distance & 0x7;
						cmdqueue.call(sm_setstepmode,stepmode);
						printf("Recv Set StepMode Cmd %x\r\n",stepmode);
						break;
					}
					case SMCMD_SETMINSPEED:
					{
						unsigned int minspeed = abs(runcmd.distance);
						if(!uplock && !downlock)
							cmdqueue.call(sm_setminspeed,minspeed);
						printf("Recv Set MinSpeed Cmd %d\r\n",minspeed);
						break;
					}
					case SMCMD_SCANUPDOWNLOCK:
					{
						inScanMode = true;
						break;
					}
					case SMCMD_SETUPLOCKPOS:
					{
						upPos = runcmd.distance;
						cmdqueue.call(sm_postconfig);
						break;
					}
					case SMCMD_SETDOWNLOCKPOS:
					{
						downPos = runcmd.distance;
						cmdqueue.call(sm_postconfig);
						break;
					}
					case SMCMD_RUNPROG1:
					{
						if(!uplock && !downlock)
							cmdqueue.call(sm_runprog1,0);
						printf("Recv PROG1 Cmd\r\n");
						break;
					}
					default:
						break;
					}
				}
				break;
			default:
				break;
			}
		}
	}
}
