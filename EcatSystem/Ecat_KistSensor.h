/**
 * @file Ecat_Elmo.h
 * @date 2018-11-15
 * @author Junho Park
 */

#ifndef ECATSYSTEM_ECAT_KIST_SENSOR_H_
#define ECATSYSTEM_ECAT_KIST_SENSOR_H_

#include "Ecat_Slave.h"

//#define _ELMO_DEBUG_

namespace hyuEcat{

/**
 * @brief Modified version of SimpleCAT
 * @details EtherCAT API for Elmo Gold-DC-Whitle
 * @version 1.1.0
 */
class EcatKistSensor : public Slave
{
public:
public:
	EcatKistSensor() : Slave(KistSensor_VendorID, KistSensor_ProductCode) {}
    virtual ~EcatKistSensor() {}

    /**
     * @brief Check the Elmo is Initialized
     * @details Returns true if Elmo has reached "operation enabled" state. The transition through the state machine is handled automatically.
     * @return mixedboolean Success:1, False:0
     */
    bool initialized() const {return initialized_;}

    /**
     * @brief Show the statusword in string(int)
     * @return state_
     */
    int Elmo_DeviceState(void) const {return state_;};

    /**
     * @brief Define a TPDO & RPDO process
     * @param[in] index
     * @param[in] domain_address
     * @return void
     */
	virtual void processData(size_t index, uint8_t* domain_address) //read and write PDO and if index is 8,
	{                                                               //check the state and change the flag of state
		// DATA READ WRITE
		switch(index)
		{
		//RxPDO
		case 0:
			EC_WRITE_S16(domain_address, command[0]);
			break;
		case 1:
			EC_WRITE_S16(domain_address, command[1]);
			break;
		case 2:
			EC_WRITE_S16(domain_address, command[2]);
			break;
		case 3:
			EC_WRITE_S16(domain_address, command[3]);
			break;
		//TxPDO
		case 4:
			SensorL[0] = EC_READ_U16(domain_address);
			break;
		case 5:
			SensorL[1] = EC_READ_U16(domain_address);
			break;
		case 6:
			SensorL[2] = EC_READ_U16(domain_address);
			break;
		case 7:
			SensorL[3] = EC_READ_U16(domain_address);
			break;
		case 8:
			SensorL[4] = EC_READ_U16(domain_address);
			break;
		case 9:
			SensorL[5] = EC_READ_U16(domain_address);
			break;
		case 10:
			SensorL[6] = EC_READ_U16(domain_address);
			break;
		case 11:
			SensorL[7] = EC_READ_U16(domain_address);
			break;
		case 12:
			SensorL[8] = EC_READ_U16(domain_address);
			break;
		case 13:
			SensorR[0] = EC_READ_U16(domain_address);
			break;
		case 14:
			SensorR[1] = EC_READ_U16(domain_address);
			break;
		case 15:
			SensorR[2] = EC_READ_U16(domain_address);
			break;
		case 16:
			SensorR[3] = EC_READ_U16(domain_address);
			break;
		case 17:
			SensorR[4] = EC_READ_U16(domain_address);
			break;
		case 18:
			SensorR[5] = EC_READ_U16(domain_address);
			break;
		case 19:
			SensorR[6] = EC_READ_U16(domain_address);
			break;
		case 20:
			SensorR[7] = EC_READ_U16(domain_address);
			break;
		case 21:
			SensorR[8] = EC_READ_U16(domain_address);
			break;


		default:
			std::cout << "WARNING. Kist Sensor pdo index out of range." << std::endl;
			break;
		}
    }

	std::string GetDevState(void)
	{
		return device_state_str_[state_];
	}

	/**
	 * @brief manage the syncs
	 * @return address of Elmo_syncs[0]
	 * @see PDOConfig.h
	 */
    virtual const ec_sync_info_t* syncs() { return &KistSensor_syncs[0]; }

    /**
     * @brief size of sync
     * @return normalized size of Elmo_sync
     * @see PDOConfig.h
     */
    virtual size_t syncSize() {
        return sizeof(KistSensor_syncs)/sizeof(ec_sync_info_t);
    }


    virtual const ec_pdo_entry_info_t* channels() {
        return KistSensor_pdo_entries;
    }

    virtual void domains(DomainMap& domains) const {
        domains = domains_;
    }

    int16_t command[4] = {0,};
    uint16_t SensorL[9] = {0,};
    uint16_t SensorR[9] = {0,};

    /**
     * @brief Description of Mode of Operation(Predefined in terms of CiA402 Profile)
     */
    enum ModeOfOperation
    {
        MODE_NO_MODE                = 0,
        MODE_PROFILED_POSITION      = 1,
        MODE_PROFILED_VELOCITY      = 3,
        MODE_PROFILED_TORQUE        = 4,
        MODE_HOMING                 = 6,
        MODE_INTERPOLATED_POSITION  = 7,
        MODE_CYCLIC_SYNC_POSITION   = 8,
        MODE_CYCLIC_SYNC_VELEOCITY  = 9,
        MODE_CYCLIC_SYNC_TORQUE     = 10
    };

private:

    /**
     * @brief Description of PDO entries.
     * @details The number of domains MUST BE EQUAL TO THE SIZE OF PDO process
     */
    DomainMap domains_ = {
    	{0, {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21}}
    };

//========================================================
// ELMO SPECIFIC
//========================================================

    /**
     * @brief Description of status word in enum struct
     */
    enum DeviceState
    {
        STATE_UNDEFINED 					= 0,
        STATE_START 						= 1,
        STATE_NOT_READY_TO_SWITCH_ON		= 3,
        STATE_SWITCH_ON_DISABLED			= 4,
        STATE_READY_TO_SWITCH_ON			= 5,
        STATE_SWITCH_ON						= 6,
        STATE_OPERATION_ENABLED				= 7,
        STATE_QUICK_STOP_ACTIVE				= 8,
        STATE_FAULT_REACTION_ACTIVE			= 9,
        STATE_FAULT							= 10,
		STATE_HOMING_PROGRESS				= 11,
		STATE_HOMING_NOT_START				= 12,
		STATE_HOMING_ATTAINED_NOT_REACHED 	= 13,
		STATE_HOMING_COMPLITE				= 14,
		STATE_HOMING_ERROR					= 15,
		STATE_HOMING_UNDIFINED
    };

    std::map<DeviceState,std::string> device_state_str_ = {
         {STATE_START,                  		"Start"},
         {STATE_NOT_READY_TO_SWITCH_ON, 		"Not Ready to Switch On"},
         {STATE_SWITCH_ON_DISABLED,     		"Switch on Disabled"},
         {STATE_READY_TO_SWITCH_ON,     		"Ready to Switch On"},
         {STATE_SWITCH_ON,              		"Switch On"},
         {STATE_OPERATION_ENABLED,      		"Operation Enabled"},
         {STATE_QUICK_STOP_ACTIVE,      		"Quick Stop Active"},
         {STATE_FAULT_REACTION_ACTIVE,  		"Fault Reaction Active"},
         {STATE_FAULT,                  		"Fault"},
		 {STATE_HOMING_PROGRESS,        		"Homing Progress"},
		 {STATE_HOMING_NOT_START,       		"Homing Not Start"},
		 {STATE_HOMING_ATTAINED_NOT_REACHED, 	"Homing Attained not reached"},
		 {STATE_HOMING_COMPLITE,        		"Homing Finished"},
		 {STATE_HOMING_ERROR,           		"Homing Error"},
		 {STATE_HOMING_UNDIFINED,       		"Homing Undefined"}
    };

    DeviceState deviceState(uint16_t status_word)
    {
        if      ((status_word & 0b01001111) == 0b00000000){
            return STATE_NOT_READY_TO_SWITCH_ON;
        }
        else if ((status_word & 0b01001111) == 0b01000000){
            return STATE_SWITCH_ON_DISABLED;
        }
        else if ((status_word & 0b01101111) == 0b00100001){
            return STATE_READY_TO_SWITCH_ON;
        }
        else if ((status_word & 0b01101111) == 0b00100011){
        	return STATE_SWITCH_ON;
        }
        else if ((status_word & 0b01101111) == 0b00100111){
        	return STATE_OPERATION_ENABLED;
        }
        else if ((status_word & 0b01101111) == 0b00000111){
            return STATE_QUICK_STOP_ACTIVE;
        }
        else if ((status_word & 0b01001111) == 0b00001111){
            return STATE_FAULT_REACTION_ACTIVE;
        }
        else if ((status_word & 0b01001111) == 0b00001000){
            return STATE_FAULT;
        }
        return STATE_UNDEFINED;
    }

    uint16_t transition(DeviceState state, uint16_t control_word)
    {
        switch(state)
        {
        case STATE_START:                   // -> STATE_NOT_READY_TO_SWITCH_ON (automatic)
            return control_word;
        case STATE_NOT_READY_TO_SWITCH_ON:  // -> STATE_SWITCH_ON_DISABLED (automatic)
        	return control_word;
        case STATE_SWITCH_ON_DISABLED:      // -> STATE_READY_TO_SWITCH_ON
            return ((control_word & 0b01111110) | 0b00000110);
        case STATE_READY_TO_SWITCH_ON:      // -> STATE_SWITCH_ON
            return ((control_word & 0b01110111) | 0b00000111);
        case STATE_SWITCH_ON:               // -> STATE_OPERATION_ENABLED
            return ((control_word & 0b01110000) | 0b00001111);
        case STATE_OPERATION_ENABLED:       // -> GOOD
        	return ((control_word & 0b00000000) | 0b00001111);
        case STATE_QUICK_STOP_ACTIVE:       // -> STATE_OPERATION_ENABLED
            return ((control_word & 0b01111111) | 0b00001111);
        case STATE_FAULT_REACTION_ACTIVE:   // -> STATE_FAULT (automatic)
            return control_word;
        case STATE_FAULT:                   // -> STATE_SWITCH_ON_DISABLED
            return ((control_word & 0b11111111) | 0b10000000);
        case STATE_HOMING_PROGRESS:
        	return ((control_word & 0b00011111) | 0b00011111);
        case STATE_HOMING_NOT_START:
        	return control_word;
        case STATE_HOMING_ATTAINED_NOT_REACHED:
        	return ((control_word & 0b00011111) | 0b00011111);
        case STATE_HOMING_COMPLITE:
        	return ((control_word & 0b00001111) | 0b00001111);
        case STATE_HOMING_ERROR:
        	return ((control_word & 0b00001111) | 0b00001111);
        case STATE_HOMING_UNDIFINED:
        	return ((control_word & 0b00011111) | 0b00011111);
        default:
            break;
        }
        return control_word;
    }

    int last_status_word_ = -1;
    DeviceState last_state_ = STATE_START;
    DeviceState state_ = STATE_START;

    bool initialized_ = false;

};

}


#endif /* ECATSYSTEM_ECAT_ELMO_H_ */
