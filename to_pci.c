/*

	version 2.6
	04.04.17

	scale - in hal(num imp per mm)
	32 out //
	PWM.outscale
	step_dir.sdscale - Fmax/10
	sd_control - velocity
	parameters t_imp, s_max
	encoder_velocity
	change outputs
	Vel + ostatok
	new sd_variant



	adress 0 - encoder0 counter
	adress 1 - encoder1 counter
	adress 2 - encoder2 counter
	adress 3 - encoder3 counter
	adress 4 - encoder4 counter
	adress 5 -
	adress 6 - Zero
	adress 7 - port in

	adress 8 - out0 shim
	adress 9 - out1 shim
	adress 10 - out2 shim
	adress 11 - out3 shim
	adress 12 - out4 shim
	adress 13 -
	adress 14 - out_ pin(16)
	adress 15 - index
	reg 28 -reserved
	adress 29 - reg_regim
	adress 30 - reg_enable(write only)
	adress 31 - wathDog

*/

#include "rtapi_ctype.h"		/* isspace() */
#include "rtapi.h"		/* RTAPI realtime OS API */
#include "rtapi_app.h"		/* RTAPI realtime module decls */
#include "hal.h"			/* HAL public API decls */
#include <linux/types.h>
#include <linux/pci.h>

/* module information */
MODULE_AUTHOR("");
MODULE_DESCRIPTION("");
MODULE_LICENSE("GPL");


/***********************************************************************
*                STRUCTURES AND GLOBAL VARIABLES                       *
************************************************************************/
#define Max_sdchanel 4


struct pci_dev *to_io;

/* this structure contains the runtime data needed by the
   driver for a single port/channel
*/

typedef struct {
	void *mem_base;
  	__u32 io_base;
	int len;
	hal_bit_t *digital_in[32];    /* ptrs for digital input pins 0 - 31 */
	hal_bit_t *digital_in_n[32];
  	hal_bit_t *digital_out[32];    /* ptrs for digital output pins 0 - 31 */
	hal_bit_t *enable_dr;
	hal_bit_t *index_en[5];

	hal_float_t *enccounts[5];
	hal_float_t *encscale[5];
	hal_float_t *encvel[5];

	hal_float_t *outscale[5];
	hal_float_t *dcontrol[5];


	hal_float_t *sdscale[Max_sdchanel];
	hal_float_t *digital_out_step[Max_sdchanel];
	hal_float_t *sd_counts[Max_sdchanel];
	hal_u32_t 	*sd_max[Max_sdchanel];
	hal_u32_t 	*L_imp[Max_sdchanel];
	float 		ostatok[Max_sdchanel];

	__s32 enc_lv[5];

} to_pci_t;

/* pointer to array of structs in shared memory, 1 per port */
static to_pci_t *device_data;


/* other globals */
static int comp_id;		/* component ID */
static int num_ports;		/* number of ports configured */

int WD_start =0;

/***********************************************************************
*                  LOCAL FUNCTION DECLARATIONS                         *
************************************************************************/
/* These is the functions that actually do the I/O
   everything else is just init code
*/
static void update_port(void *arg, long period);

/***********************************************************************
*                       INIT AND EXIT CODE                             *
************************************************************************/
#define ustreg 1      // 0-regim PWM     1-regim step/dir

#define reg_enc0 0
#define reg_enc1 1
#define reg_enc2 2
#define reg_enc3 3
#define reg_enc4 4

#define reg_zero 6

#define reg_in 7

#define reg_dcont0 8
#define reg_dcont1 9
#define reg_dcont2 10
#define reg_dcont3 11
#define reg_dcont4 12

#define reg_stepdir0 16
#define reg_stepdir1 17
#define reg_stepdir2 18
#define reg_stepdir3 19

#define reg_out 14
#define reg_out2 13
#define reg_index 15
#define reg_SWch 23
#define control_reg 30
#define reg_WD 31
// reg 28 -reserved

#define VENDORID_dev   (0x2105)
#define num_dev        (0x5555)

#define driver_NAME "to_pci"

#define SERV 1000000


#define SERV_o 1000000/SERV
#define T_zero 2


__u8 rstdr[] = {reg_stepdir0,reg_stepdir1,reg_stepdir2,reg_stepdir3};
__u8 rdctr[] = {reg_dcont0,reg_dcont1,reg_dcont2,reg_dcont3,reg_dcont4};
__u8 renci[] = {reg_enc0,reg_enc1,reg_enc2,reg_enc3,reg_enc4};

/////###########################################################################################################################

int rtapi_app_main(void)
{
    char name[HAL_NAME_LEN + 1];
    int n,i,r ;
    int retval=0;

    /* only one port at the moment */
    num_ports = 1;
    n = 0;

//	printk(KERN_ALERT "NNN2 = %X\n",ukr->timp);

    /* STEP 1: initialise the driver */
    comp_id = hal_init(driver_NAME);
	if (comp_id < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,"to_pci: ERROR: hal_init() failed\n");
    	return -1;
    }

    /* STEP 2: allocate shared memory for to_hal data */
    device_data = hal_malloc(num_ports * sizeof(to_pci_t));
    if (device_data == 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,"to_pci: ERROR: hal_malloc() failed\n");
		r = -1;
		goto fail0;
    }


////////////PCI INIT...

    to_io = pci_get_device(VENDORID_dev, num_dev, to_io);

    if (NULL == to_io) {
		rtapi_print_msg(RTAPI_MSG_ERR,"to_io == NULL\n");
		hal_exit(comp_id);
		return -1;
    }

    device_data->io_base = pci_resource_start(to_io, 0);

    device_data->len = pci_resource_len(to_io, 0);

    rtapi_print_msg(RTAPI_MSG_INFO,"to_pci: io_base: %X \n", device_data->io_base);

    device_data->mem_base = ioremap_nocache( device_data->io_base,device_data->len);

	printk(KERN_ALERT "to_pci: io_base: %X, mem_base: %p\n", device_data->io_base, device_data->mem_base);

	if (device_data->mem_base == NULL) {
                rtapi_print_msg(RTAPI_MSG_ERR,"not_remap\n");
                r = -ENODEV;
                goto fail0;
	}

//    writel(0x6000000A,(device_data->mem_base)+(reg_WD*4));

	if(ustreg)
		writel(0x6001000F,(device_data->mem_base)+(reg_SWch*4));  //regim step/dir
	else
		writel(0x60000000,(device_data->mem_base)+(reg_SWch*4));  //regim PWM

//////////////////////
//////////////////////
//////////////////////

/* ///////////////// Export IO pin's ////////////////////////////////////////////*/

    /* export write only HAL pin's for the input bit */
	for ( i=0; i<=31;i++) {
		retval = hal_pin_bit_newf(HAL_OUT, &(device_data->digital_in[i]),comp_id, "to_pci.%d.pins.pin-%02d-in", 1, i);
		if (retval < 0) {
			rtapi_print_msg(RTAPI_MSG_ERR, "to_pci: ERROR: port %d var export failed with err=%i\n", n + 1,retval);
			r = -1;
			goto fail1;
		}

		retval = hal_pin_bit_newf(HAL_OUT, &(device_data->digital_in_n[i]),comp_id, "to_pci.%d.pins.pin-%02d-in-n", 1, i);
      	if (retval < 0) {
        	rtapi_print_msg(RTAPI_MSG_ERR, "to_pci: ERROR: port %d var export failed with err=%i\n", n + 1,retval);
        	r = -1;
        	goto fail1;
      	}
	}

    /* export read only HAL pin's for the output bit */
	for ( i=0; i<=31;i++) {
		retval = hal_pin_bit_newf(HAL_IN, &(device_data->digital_out[i]),comp_id, "to_pci.%d.pins.pin-%02d-out", 1, i);
		if (retval < 0) {
			rtapi_print_msg(RTAPI_MSG_ERR,"to_pci: ERROR: port %d var export failed with err=%i\n", n + 1,retval);
			r = -1;
			goto fail1;
		}
	}


///////////////////////////
///////////////////////////
///////////////////////////

	/* export read only HAL pin's DRIVE_ENABLE */

	retval = hal_pin_bit_newf(HAL_IN, &(device_data->enable_dr),comp_id, "to_pci.%d.enable_drive", 1);

	if (retval < 0) {
	  rtapi_print_msg(RTAPI_MSG_ERR,"to_pci: ERROR: enable_dr var export failed with err=%i\n",retval);
  	  r = -1;
	  goto fail1;
	}

/////////////////////////////
/////////////////////////////
/////////////////////////////

	/* export encoder signal */

	for ( i=0; i<=4;i++) {
		/* encoder_count */
		retval = hal_pin_float_newf(HAL_OUT, &(device_data-> enccounts[i]), comp_id, "to_pci.%d.feedback.encoder%d", 1, i);
		if (retval < 0) {
			rtapi_print_msg(RTAPI_MSG_ERR,"to_pci: ERROR:  err encoder=%i\n", retval);
			r = -1;
			goto fail1;
		}
				/* encoder_scale */
		retval = hal_pin_float_newf(HAL_IN, &(device_data-> encscale[i]), comp_id, "to_pci.%d.feedback.enc_scale%d", 1, i);
		if (retval < 0) {
			rtapi_print_msg(RTAPI_MSG_ERR,"to_pci: ERROR:  err enc_scale=%i\n", retval);
			r = -1;
			goto fail1;
		}
				/* index */
		retval = hal_pin_bit_newf(HAL_IO, &(device_data->index_en[i]),comp_id, "to_pci.%d.feedback.index_en%01d", 1, i);
		if (retval < 0) {
			rtapi_print_msg(RTAPI_MSG_ERR,"to_pci: ERROR: port %d var export failed with err=%i\n", n + 1,retval);
			r = -1;
			goto fail1;
		}
			/* encoder_velocity */
		retval = hal_pin_float_newf(HAL_IN, &(device_data-> encvel[i]), comp_id, "to_pci.%d.feedback.enc_vel%d", 1, i);
		if (retval < 0) {
			rtapi_print_msg(RTAPI_MSG_ERR,"to_pci: ERROR:  err enc_velocity=%i\n", retval);
			r = -1;
			goto fail1;
		}
	}

//////////////////////////////
/////////////////////////////
/////////////////////////////
	/* export control drive PWM*/


	for ( i=0; i<=4;i++) {
		retval = hal_pin_float_newf(HAL_IN, &(device_data-> dcontrol[i]), comp_id, "to_pci.%d.PWM.dcontrol%d", 1, i);
		if (retval < 0) {
			rtapi_print_msg(RTAPI_MSG_ERR,"to_pci: ERROR:  err dcontrol=%i\n", retval);
			r = -1;
			goto fail1;
		}

		retval = hal_pin_float_newf(HAL_IN, &(device_data-> outscale[i]), comp_id, "to_pci.%d.PWM.out_scale%d", 1, i);
		if (retval < 0) {
			rtapi_print_msg(RTAPI_MSG_ERR,"to_pci: ERROR:  err out_scale=%i\n", retval);
			r = -1;
			goto fail1;
		}
	}

//////////////////////////////
/////////////////////////////
/////////////////////////////
	/* export control drive STEP_DIR*/

	for ( i=0; i<=(Max_sdchanel-1);i++) {
			/* out signals step/dir */
		retval = hal_pin_float_newf(HAL_IN, &(device_data->digital_out_step[i]),comp_id, "to_pci.%d.step_dir.outstep%d", 1, i);
		if (retval < 0) {
			rtapi_print_msg(RTAPI_MSG_ERR,"to_pci: ERROR:  err out_step=%i\n", retval);
			r = -1;
			goto fail1;
		}
			/* feed back step/dir */
		retval = hal_pin_float_newf(HAL_OUT, &(device_data-> sd_counts[i]), comp_id, "to_pci.%d.step_dir.count%d", 1, i);
		if (retval < 0) {
			rtapi_print_msg(RTAPI_MSG_ERR,"to_pci: ERROR:  err encoder=%i\n", retval);
			r = -1;
			goto fail1;
		}
			/* scale step/dir */
		retval = hal_pin_float_newf(HAL_IN, &(device_data-> sdscale[i]), comp_id, "to_pci.%d.step_dir.scale%d", 1, i);
		if (retval < 0) {
			rtapi_print_msg(RTAPI_MSG_ERR,"to_pci: ERROR:  err scale=%i\n", retval);
			r = -1;
			goto fail1;
		}
			/* max freq step/dir */
		retval = hal_pin_u32_newf(HAL_IN, &(device_data-> sd_max[i]), comp_id, "to_pci.%d.step_dir.sd_max%d", 1, i);
		if (retval < 0) {
			rtapi_print_msg(RTAPI_MSG_ERR,"to_pci: ERROR:  err sd_max=%i\n", retval);
			r = -1;
			goto fail1;
		}
			/* lenght impuls */
		retval = hal_pin_u32_newf(HAL_IN, &(device_data-> L_imp[i]), comp_id, "to_pci.%d.step_dir.L_imp%d", 1, i);
		if (retval < 0) {
			rtapi_print_msg(RTAPI_MSG_ERR,"to_pci: ERROR:  err len imp=%i\n", retval);
			r = -1;
			goto fail1;
		}
	}

/////////////////////////
////UPDATE
////////////////////////
    /* STEP 4: export function */
	rtapi_snprintf(name, sizeof(name), "to_pci.%d.update", n + 1);
	retval = hal_export_funct(name, update_port, device_data, 1, 0,comp_id);
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,"to_pci: ERROR: port %d write funct export failed\n", n + 1);
		r = -1;
		goto fail1;
    }

    rtapi_print_msg(RTAPI_MSG_INFO,"to_pci: installed driver for %d card(s)\n", num_ports);
	printk(KERN_ALERT "to_pci: installed driver for %d card(s)\n", num_ports);
    hal_ready(comp_id);
    return 0;

fail1:

iounmap((void*)device_data->mem_base);

fail0:

device_data->mem_base = NULL;
hal_exit(comp_id);
return r;

}

void rtapi_app_exit(void)
{
//	writel(0x60000000,(device_data->mem_base)+(reg_WD*4));
	iounmap((void*)device_data->mem_base);
	hal_exit(comp_id);
	printk(KERN_ALERT "to_pci: exit\n");
}


//###########################################################################################################
//
//			FUNCTIONS
//
//###########################################################################################################

int ABS_SUB(__s32 lt1, __s32 lt2)
{
	__s32 lt3;
	if(lt1<lt2)
		lt3 = lt2-lt1;
	else
		lt3 = lt1-lt2;

	if(lt3 < 5)
		return 0;
	else
		return 1;
}

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///    STEP DIR FUNCTIONS
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

__s32 DDcontr(void *farg,__u8 exr)
{

	// Step Dir Command format
	//
	// 31 ---30 ----- 29 28 --- 27 - 24 ----------- 23 - 17 --- 16 - 0
	// dir - enable - rez ----- imp_len_counter --- rez ------- pause_counter
	// impulse_length = imp_len_counter * IMP_SAMPLE ns
	// pause_length = pause_counter *  MIN_SAMPLE ns + imp_len_counter * IMP_SAMPLE ns
	//

	#define ONE_SEC 1000000000 //ns in one second
	#define IMP_SAMPLE 500 //ns
	#define MIN_SAMPLE 20 //ns
	#define MAX_PAUSE_COUNTER 131071 //17bit
	#define PID_MAX_OUTPUT 1000

	to_pci_t *data_sd;

	//__u8 smax; // now not in use
	__u8 dir,enable,imp_len_counter;
	__u32 pause_counter;
	float V_cmd;
	int max_freq;

	data_sd = farg;
	V_cmd = *(data_sd-> digital_out_step[exr]);
	//smax = *data_sd->sd_max[exr]; //now not in use
	imp_len_counter = *data_sd->L_imp[exr];

	if(imp_len_counter > 15)
		imp_len_counter = 15;

	// Zero V_cmd - output disabled
	if(V_cmd == 0)
		return imp_len_counter << 24;
		//return 0 ???

	if(V_cmd < 0){
	  V_cmd = -V_cmd;
	  dir = 1;
	}else{
	  dir = 0;
	}

	if(V_cmd > PID_MAX_OUTPUT)
		V_cmd = PID_MAX_OUTPUT;

	max_freq = ONE_SEC/(IMP_SAMPLE*2*imp_len_counter); // Hz

	pause_counter = (ONE_SEC/(max_freq*(V_cmd/PID_MAX_OUTPUT))-IMP_SAMPLE*2*imp_len_counter)/MIN_SAMPLE;
	if(pause_counter > MAX_PAUSE_COUNTER){
	  pause_counter = MAX_PAUSE_COUNTER;
	  enable = 0;
	}else{
	  enable = 1;
	}

	return dir << 31 | enable << 30 | imp_len_counter << 24 | pause_counter;
}
/**************************************************************
* REALTIME PORT WRITE FUNCTION                                *
**************************************************************/

void update_port(void *arg, long period){
	to_pci_t *port;


	__u32 tmp,mask,tmask,tanmask;
	__s32 ikor;
	int pin,k;

	int flagok;
	__s32 temp1,temp2;

	port = arg;
	tmask = 0x60000000;
	tanmask = 0xEFFFFFFF;

// WD

	writel(0x6000000A,(device_data->mem_base)+(reg_WD*4));

//////////////////////////////////////////////////////////////////////////////
////////////////////ALL INPUTS///////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// read digital inputs
	tmp = readl((port->mem_base)+(reg_in*4));
      	mask = 0x00000001;
	for (pin=0 ; pin < 32 ; pin++) {
		*(port->digital_in[pin]) = (tmp & mask) ? 1:0 ;
		*(port->digital_in_n[pin]) = (tmp & mask) ? 0:1 ;
		mask <<= 1;
	}

// Index....
	tmp = readl((port->mem_base)+(reg_zero*4));
	mask = 0x01;
	for (pin=0; pin < 5; pin++) {
		if(*port->index_en[pin] && (tmp & mask)){
			*port->index_en[pin] = 0;
		}
		mask <<= 1;
	}

///////////// read encoders /////////////////////////////////////////////

	for ( k=0; k<=4;k++) {
		flagok =1;
		temp1=(__s32)readl((port->mem_base)+(renci[k]*4));

		while (flagok)
		{
			temp2=(__s32)readl((port->mem_base)+(renci[k]*4));
			flagok = ABS_SUB(temp1,temp2);
			temp1 = temp2;
		}
		*port->enccounts[k] = temp1/(*port->encscale[k]);

		temp2 = temp1 - (port->enc_lv[k]);

		port->enc_lv[k] = temp1;

		temp2 = temp2*1000;

		*port->encvel[k] = (temp2/(*port->encscale[k]))*SERV_o;
	}

///////////// read s/d count /////////////////////////////////////////////

	for ( k=0; k<=(Max_sdchanel-1);k++) {
		flagok =1;
		temp1=(__s32)readl((port->mem_base)+(rstdr[k]*4));

		while (flagok)
		{
			temp2=(__s32)readl((port->mem_base)+(rstdr[k]*4));
			flagok = ABS_SUB(temp1,temp2);
			temp1 = temp2;
		}
		*port->sd_counts[k] = temp1/(*port->sdscale[k]);
	}

//////////////////////////////////////////////////////////////////////////////
////////////////////ALL OUTPUTS///////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// write index_enable outputs
     	tmp = 0x0;
     	mask = 0x01;
     	for (pin=0; pin < 5; pin++) {
        	if (*port->index_en[pin]) {
        		tmp |= mask;
        	}
			mask <<= 1;
     	}

		tmp |= tmask;
		writel(tmp,(port->mem_base)+(reg_index*4));

// write digital outputs
     	tmp = 0x0;
     	mask = 0x01;
     	for (pin=0; pin < 16; pin++) {
        	if (*port->digital_out[pin]) {
        		tmp |= mask;
        	}
			mask <<= 1;
     	}
		tmp |= tmask;
		writel(tmp,(port->mem_base)+(reg_out*4));
/*
		tmp = 0x0;
     	mask = 0x01;
     	for (pin=16; pin < 32; pin++) {
        	if (*port->digital_out[pin]) {
        		tmp |= mask;
        	}
			mask <<= 1;
     	}
		tmp |= tmask;
		writel(tmp,(port->mem_base)+(reg_out2*4));
*/


// write control reg  control_reg
	tmp = 0x0;
	mask = 0x01;
	for (pin=0; pin < 32; pin++) {
        	if (*port->enable_dr && pin==4) {
				tmp |= mask;
        	}
			mask <<= 1;
     	}

	tmp |= tmask;
	writel(tmp,(port->mem_base)+(control_reg*4));

// out to drive

	for ( k=0; k<=4;k++) {
		ikor = (*port->outscale[k])*((*(port-> dcontrol[k]))/10)*0xffff;
		ikor |= tmask;
		ikor &= tanmask;
		writel( ikor,(port->mem_base)+(rdctr[k]*4));
	}

// Step dir

	for ( k=0; k<=(Max_sdchanel-1);k++) {
		ikor = DDcontr(port,k);
		writel( ikor,(port->mem_base)+(rstdr[k]*4));
	}
}
