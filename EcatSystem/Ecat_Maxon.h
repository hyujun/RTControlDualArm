/**
 * @file Ecat_Maxon.h
 * @brief Modified version of SimpleCAT
 * @date 2018-11-15
 * @author Junho Park
 *
 */

#ifndef ECATSYSTEM_ECAT_MAXON_H_
#define ECATSYSTEM_ECAT_MAXON_H_

#include "Ecat_Slave.h"
#include "PDOConfig.h"

namespace hyuEcat{

/**
 * @brief EtherCAT API for Maxon Epos4-EtherCAT 50/15
 * @version 1.1.0
 */
class EcatMaxon : public Slave
{
public:
public:
	/**
	 * @brief EcatMaxon constructor
	 */
	EcatMaxon() : Slave(Maxon_VenderID, Maxon_ProductCode) {}
    virtual ~EcatMaxon() {}

    /**
     * @brief Check the Elmo is Initialized
     * @details Returns true if Elmo has reached "operation enabled" state. The transition through the state machine is handled automatically.
     * @return mixedboolean Success:1, False:0
     */
    bool initialized() const {return initialized_;}

    /**
     * @brief Check the Elmo acheived Homing
	 * @details Returns true if Elmo has reached "operation enabled" state. The transition through the state machine is handled automatically.
	 * @return mixedboolean Success:1, False:0
	 */
    bool isHoming() const {return homing_;}

    /**
     * @brief Show the statusword in string(int)
     * @return state_
     */
    int Maxon_DeviceState(void) const {return state_;};

    /**
     * @brief Write Output Torque in 1000 percentage
     * @details User must first set max_torque.
     * @param[in] torque int16_t
     */
    void writeTorque(int16_t torque)
   {
	   target_torque_ = torque;
   }

    /**
     * @brief Pre-defined Parameter for Homing Process
     * @param[in] position Client position
     * @return HomingOffset
     */
    int32_t HomingOffset(int position)
    {
    	return (unsigned int)m_HomingOffset[position];
    }

    /**
     * @brief Pre-defined Parameter for Homing Process
     * @param[in] position Client Position
     * @return HomingMethod
     */
    int8_t HomingMethod(int position)
    {
    	return (unsigned char)m_HomingMethod[position];
    }

    /**
     * @brief Pre-defined Parameter for Homing Process
     * @param[in] position Client Position
     * @return HomingSpeed
     */
    uint32_t HomingSpeed(int position)
    {
    	return (unsigned int)m_HomingSpeed[position];
    }

    /**
     * @brief Pre-defined Parameter for Homing Process
     * @param[in] position Client Position
     * @return HomingCurrentLimit
     */
    uint16_t HomingCurrentLimit(int position)
    {
    	return (unsigned short)m_HomingCurrentLimit[position];
    }

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
			EC_WRITE_S32(domain_address, target_position_);
			break;
		case 1:
			EC_WRITE_S32(domain_address, target_velocity_);
			break;
		case 2:
			EC_WRITE_S16(domain_address, target_torque_);
			break;
		case 3:
			control_word_ = EC_READ_U16(domain_address);

			//initialization sequence
			control_word_ = transition(state_, control_word_);
			EC_WRITE_U16(domain_address, control_word_);
			break;
		case 4:
			EC_WRITE_S8(domain_address, mode_of_operation_);
			break;
		//TxPDO
		case 5:
			position_ = EC_READ_S32(domain_address);
			break;
		case 6:
			velocity_ = EC_READ_S32(domain_address);
			break;
		case 7:
			torque_ = EC_READ_S16(domain_address);
			break;
		case 8:
			status_word_ = EC_READ_U16(domain_address);
			state_ = deviceState(status_word_);
			break;
		case 9:
			mode_of_operation_display_= EC_READ_S8(domain_address);
			break;
		default:
			std::cout << "WARNING. Maxon Epos4 pdo index out of range." << std::endl;
			break;
		}

		// CHECK FOR STATE CHANGE
		if (index==8) //if last entry  in domain
		{
			if (status_word_ != last_status_word_){
				state_ = deviceState(status_word_);
				if (state_ != last_state_){
					#if 1
						std::cout << "MAXON_POS: " << slave_position << " " << ", MAXON_STATE: " << device_state_str_[state_] << std::endl;
					#endif
				}
			}
			if ((state_ == STATE_OPERATION_ENABLED) && (last_state_ == STATE_OPERATION_ENABLED)){
				initialized_ = true;
			}
			else {
				initialized_ = false;
			}
			if ((state_ == STATE_HOMING_NOT_START) && (last_state_ == STATE_HOMING_NOT_START)){
				homing_ready_ = true;
			}
			else {
				homing_ready_ = false;
			}
			last_status_word_ = status_word_;
			last_state_ = state_;
		}
    }

	/**
	 * @brief manage the syncs
	 * @return address of Elmo_syncs[0]
	 * @see PDOConfig.h
	 */
    virtual const ec_sync_info_t* syncs() { return &Maxon_syncs[0]; }

    /**
     * @brief size of sync
     * @return normalized size of Elmo_sync
     * @see PDOConfig.h
     */
    virtual size_t syncSize() {
        return sizeof(Maxon_syncs)/sizeof(ec_sync_info_t);
    }

    virtual const ec_pdo_entry_info_t* channels() {
        return Maxon_pdo_entries;
    }

    virtual void domains(DomainMap& domains) const {
        domains = domains_;
    }



    int32_t  target_position_           = 0; 						/**<write*/
    int32_t  target_velocity_           = 0; 						/**<write*/
    int16_t  target_torque_             = 0; 						/**<write (max torque (max current) = 1000)*/
    uint16_t max_torque_                = 1800; 					/**<write (max current = 1000, as set in Elmo Motion Studio)*/
    uint16_t control_word_              = 0; 						/**<write*/
    int8_t   mode_of_operation_         = MODE_CYCLIC_SYNC_TORQUE; 	/**<write (use enum ModeOfOperation for convenience)*/

    int32_t  position_                  = 0; 		/**<read*/
    int32_t  velocity_					= 0;		/**<read*/
    int16_t  torque_                    = 0; 		/**<read*/
    uint16_t status_word_               = 0; 		/**<read*/
    int8_t   mode_of_operation_display_ = 0; 		/**<read*/

    /**
     * @brief Description of Mode of Operation
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

    uint32_t digital_output_            = 0; // write
    uint32_t digital_input_             = 0; // read (must be enabled in Elmo Motion Studio)

    DomainMap domains_ = {
    	{0, {0,1,2,3,4,5,6,7,8,9}}
    };


//========================================================
// ELMO SPECIFIC
//========================================================

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
         {STATE_START,                  	"Start"},
         {STATE_NOT_READY_TO_SWITCH_ON, 	"Not Ready to Switch On"},
         {STATE_SWITCH_ON_DISABLED,     	"Switch on Disabled"},
         {STATE_READY_TO_SWITCH_ON,     	"Ready to Switch On"},
         {STATE_SWITCH_ON,              	"Switch On"},
         {STATE_OPERATION_ENABLED,      	"Operation Enabled"},
         {STATE_QUICK_STOP_ACTIVE,      	"Quick Stop Active"},
         {STATE_FAULT_REACTION_ACTIVE,  	"Fault Reaction Active"},
         {STATE_FAULT,                  	"Fault"},
		 {STATE_HOMING_PROGRESS,        	"Homing Progress"},
		 {STATE_HOMING_NOT_START,       	"Homing Not Start"},
		 {STATE_HOMING_ATTAINED_NOT_REACHED, "Homing Attained not reached"},
		 {STATE_HOMING_COMPLITE,        	"Homing Finished"},
		 {STATE_HOMING_ERROR,           	"Homing Error"},
		 {STATE_HOMING_UNDIFINED,       	"Homing Undefined"}
    };

    /**
     * returns device state based upon the status_word
     */
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
        	if(mode_of_operation_display_ == MODE_HOMING){
        		if     ((((status_word & 0xff00)>>8) & 0b00110100 ) == 0b00000000)
        			return STATE_HOMING_PROGRESS;
        		else if((((status_word & 0xff00)>>8) & 0b00110100 ) == 0b00000100)
        			return STATE_HOMING_NOT_START;
        		else if((((status_word & 0xff00)>>8) & 0b00110100 ) == 0b00010000)
        			return STATE_HOMING_ATTAINED_NOT_REACHED;
        		else if((((status_word & 0xff00)>>8) & 0b00110100 ) == 0b00010100){
        			homing_ = true;
        			return STATE_HOMING_COMPLITE;
        		}
        		else if((((status_word & 0xff00)>>8) & 0b00110100 ) == 0b00100000)
        		    return STATE_HOMING_ERROR;
        		else if((((status_word & 0xff00)>>8) & 0b00110100 ) == 0b00100100)
        		    return STATE_HOMING_ERROR;
        		else
        			return STATE_HOMING_UNDIFINED;
        	}
        	else{
        		return STATE_OPERATION_ENABLED;
        	}
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
        	if(homing_ready_)
        		return ((control_word & 0b00011111) | 0b00011111);
        	else
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
    bool homing_ = false;
    bool homing_ready_ = false;

    // 1axis :
    // 2axis : 30.07+1    	160*2*2000
    // 3axis : 36.17    	160*2000
    // 4axis : 62.29    	100*2*2048
    // 5axis : 35.63    	160*2*2000
    // 6axis : 20.75    	160*2000

	int32_t 	m_HomingOffset[3] = { -136490, 75727, 120023 };				// cnt
	int8_t 		m_HomingMethod[3] = { -4, -3, -3};							//
	uint32_t 	m_HomingSpeed[3] = { 17000, 17000, 17000 };					// RPM
	uint16_t	m_HomingCurrentLimit[3] = {700, 700, 700}; 					// Per Thoussand of rated torque


};

}


#endif /* ECATSYSTEM_ECAT_ELMO_H_ */
