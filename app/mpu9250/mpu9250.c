/****************************************Copyright (c)************************************************
**                                      [°¬¿ËÄ·¿Æ¼¼]
**                                        IIKMSIK 
**                            ¹Ù·½µêÆÌ£ºhttps://acmemcu.taobao.com
**                            ¹Ù·½ÂÛÌ³£ºhttp://www.e930bbs.com
**                                   
**--------------File Info-----------------------------------------------------------------------------
** File name:			     main.c
** Last modified Date: 2019-12-25         
** Last Version:		   
** Descriptions:		   Ê¹ÓÃµÄSDK°æ±¾-SDK_16.0
**						
**----------------------------------------------------------------------------------------------------
** Created by:			
** Created date:		2019-4-3
** Version:			    1.0
** Descriptions:		mpu6050Çı¶¯³ÌĞò
**---------------------------------------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nrf_drv_twi.h"
#include "mpu9250.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
//LogĞèÒªÒıÓÃµÄÍ·ÎÄ¼ş
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
//APP¶¨Ê±Æ÷ĞèÒªÒıÓÃµÄÍ·ÎÄ¼ş
#include "app_timer.h"

#include "bsp_btn_ble.h"
//¹ã²¥ĞèÒªÒıÓÃµÄÍ·ÎÄ¼ş
#include "ble_advdata.h"
#include "ble_advertising.h"
//µçÔ´¹ÜÀíĞèÒªÒıÓÃµÄÍ·ÎÄ¼ş
#include "nrf_pwr_mgmt.h"
//SoftDevice handler configurationĞèÒªÒıÓÃµÄÍ·ÎÄ¼ş
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
//ÅÅĞòĞ´ÈëÄ£¿éĞèÒªÒıÓÃµÄÍ·ÎÄ¼ş
#include "nrf_ble_qwr.h"
//GATTĞèÒªÒıÓÃµÄÍ·ÎÄ¼ş
#include "nrf_ble_gatt.h"
//Á¬½Ó²ÎÊıĞ­ÉÌĞèÒªÒıÓÃµÄÍ·ÎÄ¼ş
#include "ble_conn_params.h"
//´®¿ÚÍ¸´«ĞèÒªÒıÓÃµÄÍ·ÎÄ¼ş
#include "my_ble_uarts.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif
#include "app_uart.h"
#include "mpu9250.h"

//TWIÇı¶¯³ÌĞòÊµÀıID,IDºÍÍâÉè±àºÅ¶ÔÓ¦£¬0:TWI0  1:TWI1
#define TWI_INSTANCE_ID     0

//TWI´«ÊäÍê³É±êÖ¾
static volatile bool m_xfer_done = false;
//¶¨ÒåTWIÇı¶¯³ÌĞòÊµÀı£¬Ãû³ÆÎªm_twi
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

//TWIÊÂ¼ş´¦Àíº¯Êı
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    //ÅĞ¶ÏTWIÊÂ¼şÀàĞÍ
	  switch (p_event->type)
    {
        //´«ÊäÍê³ÉÊÂ¼ş
			  case NRF_DRV_TWI_EVT_DONE:
            m_xfer_done = true;//ÖÃÎ»´«ÊäÍê³É±êÖ¾
            break;
        default:
            break;
    }
}
//TWI³õÊ¼»¯
void twi_master_init(void)
{
    ret_code_t err_code;
    //¶¨Òå²¢³õÊ¼»¯TWIÅäÖÃ½á¹¹Ìå
    const nrf_drv_twi_config_t twi_config = {
       .scl                = TWI_SCL_M,  //¶¨ÒåTWI SCLÒı½Å
       .sda                = TWI_SDA_M,  //¶¨ÒåTWI SDAÒı½Å
       .frequency          = NRF_DRV_TWI_FREQ_100K, //TWIËÙÂÊ
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH, //TWIÓÅÏÈ¼¶
       .clear_bus_init     = false//³õÊ¼»¯ÆÚ¼ä²»·¢ËÍ9¸öSCLÊ±ÖÓ
    };
    //³õÊ¼»¯TWI
    err_code = nrf_drv_twi_init(&m_twi, &twi_config, twi_handler, NULL);
	//¼ì²é·µ»ØµÄ´íÎó´úÂë
    APP_ERROR_CHECK(err_code);
    //Ê¹ÄÜTWI
    nrf_drv_twi_enable(&m_twi);
}

/*************************************************************************
 * ¹¦  ÄÜ : Ğ´MPU9250¼Ä´æÆ÷
 * ²Î  Êı : register_address[in]£º¼Ä´æÆ÷µØÖ·
 *        : value[in]£ºĞ´ÈëµÄÊı¾İ
 * ·µ»ØÖµ : true:Ğ´Êı¾İ³É¹¦,false£ºĞ´ÈëÊ§°Ü
 *************************************************************************/ 
bool mpu9250_register_write(uint8_t register_address, uint8_t value)
{
	  ret_code_t err_code;
	  uint8_t tx_buf[MPU9250_ADDRESS_LEN+1];
	
	  //×¼±¸Ğ´ÈëµÄÊı¾İ
		tx_buf[0] = register_address;
    tx_buf[1] = value;
	  //TWI´«ÊäÍê³É±êÖ¾ÉèÖÃÎªfalse
		m_xfer_done = false;
		//Ğ´ÈëÊı¾İ
    err_code = nrf_drv_twi_tx(&m_twi, MPU9250_ADDRESS, tx_buf, MPU9250_ADDRESS_LEN+1, false);
	  //µÈ´ıTWI×ÜÏß´«ÊäÍê³É
		while (m_xfer_done == false){}
	  if (NRF_SUCCESS != err_code)
    {
        return false;
    }
		return true;	
}
/*************************************************************************
 * ¹¦  ÄÜ : ¶ÁMPU9250¼Ä´æÆ÷
 * ²Î  Êı : register_address[in]£º¼Ä´æÆ÷µØÖ·
 *        : * destination[out]  £ºÖ¸Ïò±£´æ¶ÁÈ¡Êı¾İµÄ»º´æ
 *        : number_of_bytes[in] £º¶ÁÈ¡µÄÊı¾İ³¤¶È
 * ·µ»ØÖµ : true:²Ù×÷³É¹¦,false£º²Ù×÷Ê§°Ü
 *************************************************************************/ 
bool mpu9250_register_read(uint8_t register_address, uint8_t * destination, uint8_t number_of_bytes)
{
	  ret_code_t err_code;
	  //TWI´«ÊäÍê³É±êÖ¾ÉèÖÃÎªfalse
		m_xfer_done = false;
	  err_code = nrf_drv_twi_tx(&m_twi, MPU9250_ADDRESS, &register_address, 1, true);
	  //µÈ´ıTWI×ÜÏß´«ÊäÍê³É
		while (m_xfer_done == false){}
    if (NRF_SUCCESS != err_code)
    {
        return false;
    }
		//TWI´«ÊäÍê³É±êÖ¾ÉèÖÃÎªfalse
		m_xfer_done = false;
	  err_code = nrf_drv_twi_rx(&m_twi, MPU9250_ADDRESS, destination, number_of_bytes);
		//µÈ´ıTWI×ÜÏß´«ÊäÍê³É
		while (m_xfer_done == false){}
		if (NRF_SUCCESS != err_code)
    {
        return false;
    }
		return true;
}
/*************************************************************************
 * ¹¦  ÄÜ : Ğ´AK8963¼Ä´æÆ÷
 * ²Î  Êı : register_address[in]£º¼Ä´æÆ÷µØÖ·
 *        : value[in]£ºĞ´ÈëµÄÊı¾İ
 * ·µ»ØÖµ : true:Ğ´Êı¾İ³É¹¦,false£ºĞ´ÈëÊ§°Ü
 *************************************************************************/ 
bool AK8963_register_write(uint8_t register_address, uint8_t value)
{
	  ret_code_t err_code;
	  uint8_t tx_buf[MPU9250_ADDRESS_LEN+1];
	
	  //×¼±¸Ğ´ÈëµÄÊı¾İ
		tx_buf[0] = register_address;
    tx_buf[1] = value;
	  //TWI´«ÊäÍê³É±êÖ¾ÉèÖÃÎªfalse
		m_xfer_done = false;
		//Ğ´ÈëÊı¾İ
    err_code = nrf_drv_twi_tx(&m_twi, AK8963_MAGN_ADDRESS, tx_buf, MPU9250_ADDRESS_LEN+1, false);
	  //µÈ´ıTWI×ÜÏß´«ÊäÍê³É
		while (m_xfer_done == false){}
	  if (NRF_SUCCESS != err_code)
    {
        return false;
    }
		return true;	
}
/*************************************************************************
 * ¹¦  ÄÜ : ¶ÁAK8963¼Ä´æÆ÷
 * ²Î  Êı : register_address[in]£º¼Ä´æÆ÷µØÖ·
 *        : * destination[out]  £ºÖ¸Ïò±£´æ¶ÁÈ¡Êı¾İµÄ»º´æ
 *        : number_of_bytes[in] £º¶ÁÈ¡µÄÊı¾İ³¤¶È
 * ·µ»ØÖµ : true:²Ù×÷³É¹¦,false£º²Ù×÷Ê§°Ü
 *************************************************************************/ 
bool AK8963_register_read(uint8_t register_address, uint8_t * destination, uint8_t number_of_bytes)
{
	  ret_code_t err_code;
	  //TWI´«ÊäÍê³É±êÖ¾ÉèÖÃÎªfalse
		m_xfer_done = false;
	  err_code = nrf_drv_twi_tx(&m_twi, AK8963_MAGN_ADDRESS, &register_address, 1, true);
	  //µÈ´ıTWI×ÜÏß´«ÊäÍê³É
		while (m_xfer_done == false){}
    if (NRF_SUCCESS != err_code)
    {
        return false;
    }
		//TWI´«ÊäÍê³É±êÖ¾ÉèÖÃÎªfalse
		m_xfer_done = false;
	  err_code = nrf_drv_twi_rx(&m_twi, AK8963_MAGN_ADDRESS, destination, number_of_bytes);
		//µÈ´ıTWI×ÜÏß´«ÊäÍê³É
		while (m_xfer_done == false){}
		if (NRF_SUCCESS != err_code)
    {
        return false;
    }
		return true;
}

/*************************************************************************
 * ¹¦  ÄÜ : ¶Á¼ÓËÙ¶ÈÔ­Ê¼Öµ
 * ²Î  Êı : pACC_X[in]£º¼ÓËÙ¶ÈxÖáµÄÔ­Ê¼Êı¾İ£¨´ø·ûºÅ£©
 *        : pACC_Y[in]£º¼ÓËÙ¶ÈyÖáµÄÔ­Ê¼Êı¾İ£¨´ø·ûºÅ£©
 *        : pACC_Z[in]£º¼ÓËÙ¶ÈzÖáµÄÔ­Ê¼Êı¾İ£¨´ø·ûºÅ£©
 * ·µ»ØÖµ : true:¶ÁÈ¡³É¹¦£¬false£º¶ÁÈ¡Ê§°Ü
 *************************************************************************/ 
/*************************************************************************
 * ¹¦  ÄÜ : ¶ÁÈ¡mpu9250ID
 * ²Î  Êı : ÎŞ
 * ·µ»ØÖµ : true:¶ÁÈ¡³É¹¦£¬false£º¶ÁÈ¡Ê§°Ü
 *************************************************************************/ 
bool mpu9250_verify_product_id(void)
{
    uint8_t who_am_i;

    if (mpu9250_register_read(ADDRESS_WHO_AM_I, &who_am_i, 1))
    {
        if (who_am_i != MPU9250_WHO_AM_I)
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    else
    {
        return false;
    }
}
/*************************************************************************
 * ¹¦  ÄÜ : ³õÊ¼»¯MPU6050
 * ²Î  Êı : ÎŞ
 * ·µ»ØÖµ : true:³õÊ¼»¯³É¹¦£¬false£º³õÊ¼»¯Ê§°Ü
 *************************************************************************/ 
/*************************************************************************
 * ¹¦  ÄÜ : ³õÊ¼»¯MPU9250
 * ²Î  Êı : ÎŞ
 * ·µ»ØÖµ : true:³õÊ¼»¯³É¹¦£¬false£º³õÊ¼»¯Ê§°Ü
 *************************************************************************/ 
bool mpu9250_init(void)
{  
  
  bool transfer_succeeded = true;
  //ÑéÖ¤MPU9250 ID
  transfer_succeeded &= mpu9250_verify_product_id();
	if(mpu9250_verify_product_id() == false)
	{
		return false;
	}
	//¸´Î»gyro, accelerometer temperature sensor
	(void)mpu9250_register_write(MPU_SIGPATH_RST_REG , 0x07); 
  //»½ĞÑMPU9250
	(void)mpu9250_register_write(MPU_PWR_MGMT1_REG , 0x00); 
	//ÉèÖÃGYRO
	(void)mpu9250_register_write(MPU_SAMPLE_RATE_REG , 0x07); //ÉèÖÃ²ÉÑùÂÊ(Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV))£º1KHz 
	(void)mpu9250_register_write(MPU_CFG_REG , 0x01); //ÉèÖÃµÍÍ¨ÂË²¨Æ÷£¬½ØÖ¹ÆµÂÊÊÇ1K£¬´ø¿íÊÇ5K
	(void)mpu9250_register_write(MPU_INT_EN_REG, 0x00); //¹Ø±ÕÖĞ¶
  (void)mpu9250_register_write(MPU_GYRO_CFG_REG , 0x18); //ÍÓÂİÒÇ×Ô¼ì¼°²âÁ¿·¶Î§£¬µäĞÍÖµ£º0x18(²»×Ô¼ì£¬2000deg/s)
  (void)mpu9250_register_write(MPU_ACCEL_CFG_REG,0x18); //ÅäÖÃ¼ÓËÙ¶È´«¸ĞÆ÷Á¿³Ì +16G s£¬²»×Ô¼ì  		
  (void)mpu9250_register_write(MPU_INTBP_CFG_REG,0x02);//Ê¹ÄÜBy pass
	(void)AK8963_register_write(MPU9250_REG_AK8963_CNTL,0x11);//ÅäÖÃ´ÅÁ¦¼ÆÎªµ¥´Î²âÁ¿Ä£Ê½£¬16Î»Êä³ö
  return transfer_succeeded;
}
/*************************************************************************
 * ¹¦  ÄÜ : ¶ÁÍÓÂİÒÇÔ­Ê¼Öµ
 * ²Î  Êı : pGYRO_X[in]£ºÍÓÂİÒÇxÖáµÄÔ­Ê¼Êı¾İ£¨´ø·ûºÅ£©
 *        : pGYRO_Y[in]£ºÍÓÂİÒÇyÖáµÄÔ­Ê¼Êı¾İ£¨´ø·ûºÅ£©
 *        : pGYRO_Z[in]£ºÍÓÂİÒÇzÖáµÄÔ­Ê¼Êı¾İ£¨´ø·ûºÅ£©
 * ·µ»ØÖµ : true:¶ÁÈ¡³É¹¦£¬false£º¶ÁÈ¡Ê§°Ü
 *************************************************************************/ 
bool MPU9250_ReadGyro(int16_t *pGYRO_X , int16_t *pGYRO_Y , int16_t *pGYRO_Z )
{
	uint8_t buf[6]; 
  bool ret = false;	
	
	if(mpu9250_register_read(MPU9250_GYRO_OUT,  buf, 6) == true)
	{
		*pGYRO_X = (buf[0] << 8) | buf[1];
//	  if(*pGYRO_X & 0x8000) *pGYRO_X-=65536;
		
	  *pGYRO_Y= (buf[2] << 8) | buf[3];
 //   if(*pGYRO_Y & 0x8000) *pGYRO_Y-=65536;
	
    *pGYRO_Z = (buf[4] << 8) | buf[5];
//	  if(*pGYRO_Z & 0x8000) *pGYRO_Z-=65536;
		
		ret = true;
	}

	return ret;
}			   
/*************************************************************************
 * ¹¦  ÄÜ : ¶Á¼ÓËÙ¶ÈÔ­Ê¼Öµ
 * ²Î  Êı : pACC_X[in]£º¼ÓËÙ¶ÈxÖáµÄÔ­Ê¼Êı¾İ£¨´ø·ûºÅ£©
 *        : pACC_Y[in]£º¼ÓËÙ¶ÈyÖáµÄÔ­Ê¼Êı¾İ£¨´ø·ûºÅ£©
 *        : pACC_Z[in]£º¼ÓËÙ¶ÈzÖáµÄÔ­Ê¼Êı¾İ£¨´ø·ûºÅ£©
 * ·µ»ØÖµ : true:¶ÁÈ¡³É¹¦£¬false£º¶ÁÈ¡Ê§°Ü
 *************************************************************************/ 
bool MPU9250_ReadAcc( int16_t *pACC_X , int16_t *pACC_Y , int16_t *pACC_Z )
{
	uint8_t buf[6];
  bool ret = false;		
  if(mpu9250_register_read(MPU9250_ACC_OUT, buf, 6) == true)
	{
		mpu9250_register_read(MPU9250_ACC_OUT, buf, 6);
		*pACC_X = (buf[0] << 8) | buf[1];
//		if(*pACC_X & 0x8000) *pACC_X-=65536;

		*pACC_Y= (buf[2] << 8) | buf[3];
//		if(*pACC_Y & 0x8000) *pACC_Y-=65536;

		*pACC_Z = (buf[4] << 8) | buf[5];
//		if(*pACC_Z & 0x8000) *pACC_Z-=65536;
		ret = true;
	}
	return ret;
}
/*************************************************************************
 * ¹¦  ÄÜ : ¶Á´ÅÁ¦¼ÆÔ­Ê¼Öµ
 * ²Î  Êı : pMAG_X[in]£º´ÅÁ¦¼ÆxÖáµÄÔ­Ê¼Êı¾İ£¨´ø·ûºÅ£©
 *        : pMAG_Y[in]£º´ÅÁ¦¼ÆyÖáµÄÔ­Ê¼Êı¾İ£¨´ø·ûºÅ£©
 *        : pMAG_Z[in]£º´ÅÁ¦¼ÆzÖáµÄÔ­Ê¼Êı¾İ£¨´ø·ûºÅ£©
 * ·µ»ØÖµ : true:¶ÁÈ¡³É¹¦£¬false£º¶ÁÈ¡Ê§°Ü
 *************************************************************************/ 
bool MPU9250_ReadMag( int16_t *pMAG_X , int16_t *pMAG_Y , int16_t *pMAG_Z )
{
	uint8_t buf[6];
	uint8_t x_axis[1],y_axis[1],z_axis[1];
	AK8963_register_read(MPU9250_REG_AK8963_ASAX, x_axis, 1);// XÖáÁéÃô¶Èµ÷ÕûÖµ
  AK8963_register_read(MPU9250_REG_AK8963_ASAY, y_axis, 1);
  AK8963_register_read(MPU9250_REG_AK8963_ASAZ, z_axis, 1);
	
	
  bool ret = false;		
  if(AK8963_register_read(MPU9250_REG_AK8963_XOUT_L, buf, 6) == true)
	{
		AK8963_register_read(MPU9250_REG_AK8963_XOUT_L, buf, 6);
		*pMAG_X = ((buf[1] << 8) | buf[0])*(((x_axis[0]-128)>>8)+1);
//		if(*pMAG_X & 0x8000) *pMAG_X-=65536;

		*pMAG_Y= ((buf[3] << 8) | buf[2])*(((y_axis[0]-128)>>8)+1);
//		if(*pMAG_Y & 0x8000) *pMAG_Y-=65536;

		*pMAG_Z = ((buf[5] << 8) | buf[4])*(((z_axis[0]-128)>>8)+1);
//		if(*pMAG_Z & 0x8000) *pMAG_Z-=65536;
		ret = true;
		(void)AK8963_register_write(MPU9250_REG_AK8963_CNTL,0x11);//Ã¿´Î¶ÁÍê¶¼ÒªÖØĞÂÅäÖÃ´ÅÁ¦¼ÆÎªµ¥´Î²âÁ¿Ä£Ê½
	}
	return ret;
	
}



