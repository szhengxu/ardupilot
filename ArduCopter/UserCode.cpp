#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
uint8_t on_off_switck=1;
void Copter::userhook_50Hz()
{
    // put your 20Hz code here
	// UESTC Start
	// 50hz handle pips rx
    // process uart rx to gcs
	AP_HAL::UARTDriver *user_uart1,*user_uart2;
	static uint8_t  cmd_count=0,type=10,send_count=0;
	static uint16_t log_count=0;
	static uint64_t last_time=0;

	user_uart1 = serial_manager.find_serial(AP_SerialManager::SerialProtocol_PIPS, 0);
	user_uart2 = serial_manager.find_serial(AP_SerialManager::SerialProtocol_GAS_FLOW, 0);

	if (user_uart1 != nullptr) {
		//get rx count
		uint16_t uart_count = user_uart1->available();

		if (uart_count > 0 && type!=8 && type!=9) {
			uint8_t data192[192] = { 0 };
			uart_count=(uart_count>192)?192:uart_count;
			for (uint16_t i = 0; i < uart_count; i++) {
				data192[i] = user_uart1->read();
			}
			if(cmd_count<=5 || uart_count<20){
				//rx cmd callback
				data192[80]=send_count;
				send_count++;
				gcs().pips_send(1,data192, uart_count);
			}else{
				//write sd log
				Log_Write_PIPS(log_count,type,data192,uart_count);

				//send mavlink
				uint8_t data96[96] = { 0 };
				for(uint8_t i=0;i<uart_count/2;i++)
				{
					data96[i] = ((((uint16_t)data192[2*i+1])<<8) | data192[2*i])/257;
				}
				if(type==20)
				{
					data96[80]=send_count;
					send_count++;
				}
				gcs().pips_send(type,data96, uart_count/2);
				type++;
			}
		}
		if(type==9 || type==8)
		{
			type++;
		}
		//tx
		if((AP_HAL::millis() - last_time) > 1000 && on_off_switck==1){
			log_count++;
			last_time=AP_HAL::millis();
			uint8_t str1[4] = {'#','A','0','\n'};
			uint8_t str2[4] = {'#','C','0','\n'};
			uint8_t str3[4] = {'#','D','0','\n'};
			uint8_t str4[6] = {'#','V','1','2','3','\n'};
			uint8_t str5[7] = {'#','T','1','2','3','4','\n'};
			uint8_t str6[4] = {'#','K','\n'};
			uint8_t str7[3] = {'#','E','\n'};
			switch(cmd_count)
			{
			case 0:user_uart1->write(str1, 4);cmd_count++;break;
			case 1:user_uart1->write(str2, 4);cmd_count++;break;
			case 2:user_uart1->write(str3, 4);cmd_count++;break;
			case 3:user_uart1->write(str4, 6);cmd_count++;break;
			case 4:user_uart1->write(str5, 7);cmd_count++;break;
			case 5:user_uart1->write(str6, 3);cmd_count++;break;
			case 6:user_uart1->write(str7, 3);type=8;break;
			default:break;
			}
		}
	}

	if (user_uart2 != nullptr) {
		//get rx count
		uint16_t uart_count = user_uart2->available();

		if (uart_count >= 1) {
			uint8_t data96[96] = { 0 };
			uart_count = (uart_count > 96) ? 96 : uart_count;
			for (uint16_t i = 0; i < uart_count; i++) {
				data96[i] = user_uart2->read();
			}
			if (cmd_count <= 5 || uart_count < 50) {
				//rx cmd callback
				data96[80]=send_count;
				send_count++;
				gcs().gas_flow_send(1, data96, uart_count);
			}
		}

	}
	//UESTC End
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(uint8_t ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(uint8_t ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(uint8_t ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif
