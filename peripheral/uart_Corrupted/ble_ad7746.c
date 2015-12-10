/* Copyright (c) <2015> <Shun Bai (wanyancan at gmail)>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

 */
 

#include "ble_ad7746.h"
#include "app_error.h"  // for APP_ERROR_CHECK

static ble_AD7746_t m_AD7746;

static void on_connect(MAKE_SERVICE_STRUCT_NAME(SERV_BASE) * p_lbs, ble_evt_t * p_ble_evt)
{
    p_lbs->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}
static void on_disconnect(MAKE_SERVICE_STRUCT_NAME(SERV_BASE) * p_lbs, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_lbs->conn_handle = BLE_CONN_HANDLE_INVALID;
}
static void on_write(MAKE_SERVICE_STRUCT_NAME(SERV_BASE) * p_lbs, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    
    if ((p_evt_write->handle == p_lbs->CONFIG_char_handleID.value_handle) &&
        (p_evt_write->len == 12) &&
        (p_lbs->on_write_config_callback != NULL))
    {
        p_lbs->on_write_config_callback(p_lbs, (CONFIG_bytes_t *)p_evt_write->data, p_evt_write->len);
    }
}

// to be written, like led display char
// it needs on_written callback function to be defined in the application 
// to do something when new written data is received
uint32_t config_char_add(ble_AD7746_t * p_ad7746, const ble_AD7746_init_t * p_ad7746_init)
{
	uint32_t       err_code = NRF_SUCCESS;
    ble_gatts_char_md_t char_md;

		// no need notify, so no CCCD attr
	  // ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;


		// set character metadata
    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;  /**< Characteristic Properties. */
    char_md.char_props.write  = 1;  /** TODO: change the char property */
    char_md.p_char_user_desc  = NULL;  /**< Pointer to a UTF-8 encoded string (non-NULL terminated), NULL if the descriptor is not required. */
    char_md.p_char_pf         = NULL;  /**< Pointer to a presentation format structure or NULL if the CPF descriptor is not required. */
    char_md.p_user_desc_md    = NULL;  /**< Attribute metadata for the User Description descriptor, or NULL for default values. */  
    char_md.p_cccd_md         = NULL;  /**< Attribute metadata for the Client Characteristic Configuration Descriptor,  */
    char_md.p_sccd_md         = NULL;	 /**< Attribute metadata for the Server Characteristic Configuration Descriptor, */

		ble_uuid.type = p_ad7746->uuid_type; /** filled by sd_ble_uuid_vs_add when adding service to GATTS */
		ble_uuid.uuid = AD7746_UUID_CONFIG_CHAR; /** TODO: change char UUID here */
    
		// set attribute metadata
    memset(&attr_md, 0, sizeof(attr_md));
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm); /** TODO: change attr perm here */
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);  /** TODO: change attr perm here */
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

		// set attribute itself
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 12*sizeof(uint8_t);  /** TODO: change data size here */
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = 12*sizeof(uint8_t);
    attr_char_value.p_value   = NULL; 

    err_code = sd_ble_gatts_characteristic_add(p_ad7746->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_ad7746->CONFIG_char_handleID); /** TODO: store char handle here */
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
	

	return err_code;
}

// periodically notify , like battery level char, button char
uint32_t cap_char_add(ble_AD7746_t * p_ad7746, const ble_AD7746_init_t * p_ad7746_init)
{
	uint32_t       err_code = NRF_SUCCESS;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;


		// set CCCD attribute metadata
    // Add notification will require CCCD attribute
    if (1)
    {
        memset(&cccd_md, 0, sizeof(cccd_md));

				// CCCD should be open to read
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
        
        cccd_md.vloc       = BLE_GATTS_VLOC_STACK;
    }

		// set character metadata
    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;  /**< Characteristic Properties. */
    char_md.char_props.notify = 1;  
    char_md.p_char_user_desc  = NULL;  /**< Pointer to a UTF-8 encoded string (non-NULL terminated), NULL if the descriptor is not required. */
    char_md.p_char_pf         = NULL;  /**< Pointer to a presentation format structure or NULL if the CPF descriptor is not required. */
    char_md.p_user_desc_md    = NULL;  /**< Attribute metadata for the User Description descriptor, or NULL for default values. */  
    char_md.p_cccd_md         = &cccd_md; /**< Attribute metadata for the Client Characteristic Configuration Descriptor,  */
    char_md.p_sccd_md         = NULL;		/**< Attribute metadata for the Server Characteristic Configuration Descriptor, */

		ble_uuid.type = p_ad7746->uuid_type; /** filled by sd_ble_uuid_vs_add when adding service to GATTS */
		ble_uuid.uuid = AD7746_UUID_CAP_CHAR; /** TODO: change char UUID here */
    
		// set attribute metadata
    memset(&attr_md, 0, sizeof(attr_md));
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

		// set attribute itself
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(unsigned long);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof(unsigned long);
    attr_char_value.p_value   = NULL; 

    err_code = sd_ble_gatts_characteristic_add(p_ad7746->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_ad7746->CAP_char_handleID); /** TODO: store char handle here */
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
	
	return err_code;
}

uint32_t temp_char_add(ble_AD7746_t * p_ad7746, const ble_AD7746_init_t * p_ad7746_init)
{
	uint32_t       err_code = NRF_SUCCESS;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;


		// set CCCD attribute metadata
    // Add notification will require CCCD attribute
    if (1)
    {
        memset(&cccd_md, 0, sizeof(cccd_md));

				// CCCD should be open to read
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
        
        cccd_md.vloc       = BLE_GATTS_VLOC_STACK;
    }

		// set character metadata
    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;  /**< Characteristic Properties. */
    char_md.char_props.notify = 1;  
    char_md.p_char_user_desc  = NULL;  /**< Pointer to a UTF-8 encoded string (non-NULL terminated), NULL if the descriptor is not required. */
    char_md.p_char_pf         = NULL;  /**< Pointer to a presentation format structure or NULL if the CPF descriptor is not required. */
    char_md.p_user_desc_md    = NULL;  /**< Attribute metadata for the User Description descriptor, or NULL for default values. */  
    char_md.p_cccd_md         = &cccd_md; /**< Attribute metadata for the Client Characteristic Configuration Descriptor,  */
    char_md.p_sccd_md         = NULL;		/**< Attribute metadata for the Server Characteristic Configuration Descriptor, */

		ble_uuid.type = p_ad7746->uuid_type; /** filled by sd_ble_uuid_vs_add when adding service to GATTS */
		ble_uuid.uuid = AD7746_UUID_TEMP_CHAR;   /** TODO: change char UUID here */
    
		// set attribute metadata
    memset(&attr_md, 0, sizeof(attr_md));
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

		// set attribute itself
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(unsigned long);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof(unsigned long);
    attr_char_value.p_value   = NULL; 

    err_code = sd_ble_gatts_characteristic_add(p_ad7746->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_ad7746->TEMP_char_handleID); /** TODO: store char handle here */
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }	
	return err_code;
}


uint32_t ble_AD7746_init(ble_AD7746_t * p_ad7746, const ble_AD7746_init_t * p_ad7746_init)
{
	    uint32_t       err_code = NRF_SUCCESS;
	    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_ad7746->on_write_config_callback = p_ad7746_init->on_write_config_callback;
    p_ad7746->conn_handle                 = BLE_CONN_HANDLE_INVALID;

		ble_uuid128_t base_uuid;
		for(int i=0; i<16; i++)
			base_uuid.uuid128[i] = AD7746_UUID_BASE[i];
	
    // Add service
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_ad7746->uuid_type);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
		ble_uuid.type = p_ad7746->uuid_type;
    ble_uuid.uuid = AD7746_UUID_SERVICE;
		

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_ad7746->service_handle);

    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add CONFIG characteristic
    err_code = config_char_add(p_ad7746, p_ad7746_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add Cap characteristic
    err_code = cap_char_add(p_ad7746, p_ad7746_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
		
    // Add temp characteristic
    err_code = temp_char_add(p_ad7746, p_ad7746_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }		

    return NRF_SUCCESS;

}
void ble_AD7746_on_ble_evt(ble_AD7746_t * p_service, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_service, p_ble_evt);
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_service, p_ble_evt);
            break;
            
        case BLE_GATTS_EVT_WRITE:
            on_write(p_service, p_ble_evt);
            break;
            
        default:
            // No implementation needed.
            break;
    }
}

uint32_t ble_AD7746_send_temp_notify(MAKE_SERVICE_STRUCT_NAME(SERV_BASE) * p_service, float tempValue)
{
		uint32_t err_code = NRF_SUCCESS;
	
    ble_gatts_value_t gatts_value;
		float localV = tempValue;
		// Initialize value struct.
		memset(&gatts_value, 0, sizeof(gatts_value));

		gatts_value.len     = sizeof(float);
		gatts_value.offset  = 0;
		gatts_value.p_value = (uint8_t*)&localV;

		// Update database.
		err_code = sd_ble_gatts_value_set(p_service->conn_handle,
																			p_service->TEMP_char_handleID.value_handle,
																			&gatts_value);
		APP_ERROR_CHECK(err_code);

		// Send value if connected and notifying.
		if ((p_service->conn_handle != BLE_CONN_HANDLE_INVALID))
		{
				ble_gatts_hvx_params_t hvx_params;

				memset(&hvx_params, 0, sizeof(hvx_params));

				hvx_params.handle = p_service->TEMP_char_handleID.value_handle;
				hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
				hvx_params.offset = gatts_value.offset;
				hvx_params.p_len  = &gatts_value.len;
				hvx_params.p_data = gatts_value.p_value;

				err_code = sd_ble_gatts_hvx(p_service->conn_handle, &hvx_params);

		}


		return err_code;
	
}
