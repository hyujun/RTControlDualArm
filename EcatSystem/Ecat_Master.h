/**
 * @file Ecat_Master.h
 * @brief EtherCAT master from SimpleCAT
 * @date 2018-11-11
 * @author Junho Park
 */
#ifndef ECATSYSTEM_ECAT_MASTER_H_
#define ECATSYSTEM_ECAT_MASTER_H_

#include "ecrt.h"
#include <string>
#include <vector>
#include <map>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <sys/resource.h>
#include <sys/mman.h>
#include "Ecat_Slave.h"
#include "Ecat_Elmo.h"

//#define _ECAT_MASTER_DEBUG_
/**
 * @brief EherCAT namespace for EtherLAB API
 * @version 1.0.0
 */
namespace hyuEcat {

class Slave;

/**
 * @brief EtherCAT Master Client of EtherLAB API
 * @version 1.2.0
 */
class Master {
public:
	/**
	 * @brief Master class constructor
	 * @param[in] master do not need to set the value
	 */
	Master(const int master = 0);
	virtual ~Master();
	/**
	 * @brief Read SDO Object
	 * @param[in] position
	 * @param[in] index
	 * @param[in] subindex
	 * @param[in] *data address of data maximum 4 bytes
	 * @see CiA-402 document
	 *
	 */
	void SDOread(uint16_t position, uint16_t index, uint8_t subindex, uint8_t *data);
	/**
	 * @brief Write SDO Object
	 * @param[in] position
	 * @param[in] index
	 * @param[in] subindex
	 * @param[in] *data address of data maximum 4 bytes
	 * @see CiA-402 document
	 */
	void SDOwrite(uint16_t position, uint16_t index, uint8_t subindex, uint8_t *data);

	/**
	 * @brief Add EtherCAT slaves to EtherCAT master
	 * @param[in] alias alias of client
	 * @param[in] position position of client
	 * @param[in] slave info of client
	 * @return void
	 */
	void addSlave(uint16_t alias, uint16_t position, Slave* slave);

	/**
	 * @brief Add EtherCAT Elmo slaves to EtherCAT master
	 * @param[in] alias
	 * @param[in] position
	 * @param[in] slave info of Elmo client
	 */
	void addSlaveWithHoming(uint16_t alias, uint16_t position, EcatElmo* slave);

	/**
	 * @brief Start Activate the client
	 */
	void activate(void);

	/**
	 * @brief Start Activate the client with Distributed Clock Mode
	 * @param[in] RefPosition reference client's position
	 * @param[in] SyncCycleNano Control Cycle time in nano seconds
	 * @return void
	 */
	void activateWithDC(uint8_t RefPosition, uint32_t SyncCycleNano);

	/**
	 * @brief synchronize master with each client
	 * @param[in] RefTime current time ex) rt_timer_read()
	 * @return void
	 */
	void SyncEcatMaster(uint64_t RefTime);

	/**
	 * @brief deactivate the client
	 */
	void deactivate(void);

	/**
	 * @brief TPDO, RPDO process execute
	 * @param[in] domain
	 */
	void update(unsigned int domain = 0);

	/**
	 * @brief RPDO process execute
	 * @param[in] domain initial value = 0
	 */
	void TxUpdate(unsigned int domain = 0);

	/**
	 * @brief TPDO process execute
	 * @param[in] domain initial value = 0
	 */
	void RxUpdate(unsigned int domain = 0);

	std::string GetEcatMasterLinkState(void)
	{
		return EcatMasterLinkState;
	}

	unsigned int GetEcatMasterState(void)
	{
		return EcatMasterState;
	}

	unsigned int GetConnectedSlaves(void)
	{
		return ConnectedSlaves;
	}

	unsigned int GetSlaveState(int _pos)
	{
		return m_slave_info[_pos].SlaveState;
	}

	std::string GetSlaveConnected(int _pos)
	{
		return m_slave_info[_pos].SlaveConnected;
	}

	std::string GetSlaveNMT(int _pos)
	{
		return m_slave_info[_pos].SlaveNMT;
	}

private:

	/**
	 * @brief Check domain state
	 * @param[in] domain initial value = 0
	 */
	void checkDomainState(unsigned int domain = 0);

	/**
	 * @brief Check master state
	 */
	void checkMasterState(void);

	/**
	 * @brief Check slave state
	 */
	void checkSlaveStates(void);

	/**
	 * @brief Confirm the EtherCAT Master is Running
	 */
	volatile bool isRunning = false;

	std::string EcatMasterLinkState="down";
	unsigned int EcatMasterState=0;
	unsigned int ConnectedSlaves=0;

	/**
	 * @brief Structure for regist PDOs
	 */
	struct DomainInfo;

	/**
	 * @brief resiger for PDO domain of each client
	 * @param[in] alias  alias of client
	 * @param[in] position  position of clinet
	 * @param[in] channel_indices  number of pdos
	 * @param[in] domain_info  regist each info of client
	 * @param[in] slave  info of client
	 * @see Ecat_Slave.h
	 */
	void registerPDOInDomain(uint16_t alias, uint16_t position, std::vector<unsigned int>& channel_indices,	DomainInfo* domain_info, Slave* slave);


	static void printWarning(const std::string& message);
	static void printWarning(const std::string& message, int position);

	ec_master_t *p_master;
	ec_master_state_t m_master_state = {};

	struct DomainInfo{
		DomainInfo(ec_master_t* master);
		~DomainInfo();

		uint32_t domainWorkingCounter=0;
		uint32_t domainWCState=0;

		ec_domain_t *domain = NULL;
		ec_domain_state_t domain_state = {};
		uint8_t *domain_pd = NULL;

		std::vector<ec_pdo_entry_reg_t> domain_regs;

		struct Entry{
			Slave* slave = NULL; 				/**<slave pointer*/
			int num_pdos = 0;					/**<number of pdo entries */
			unsigned int* offset = NULL; 		/**<alias of slave*/
			unsigned int* bit_position = NULL; 	/**<position of slave*/
		};

		std::vector<Entry> entries;
	};

	std::map<unsigned int, DomainInfo*> m_domain_info;

	struct SlaveInfo{
		Slave* slave = NULL;
		ec_slave_config_t* config = NULL;
		ec_slave_config_state_t config_state = {0};
		unsigned int SlaveState=0;
		std::string SlaveConnected="offline";
		std::string SlaveNMT="PreOP";
	};

	std::vector<SlaveInfo> m_slave_info;

	unsigned short INDEX_HOMING_METHOD = 0x6098U; 		/**<Index for homing method*/
	unsigned short INDEX_HOMING_SPEED =	0x6099;			/**<Index for homing speed*/
	unsigned short SUBINDEX_HOMING_HIGHSPEED = 0x01; 	/**<Subindex for homing high speed*/
	unsigned short SUBINDEX_HOMING_LOWSPEED = 0x02; 	/**<Subindex for homing low speed*/
	unsigned short INDEX_HOMING_OFFSET = 0x607CU;		/**<Index for homing offset*/
	unsigned short INDEX_HOMING_CURRENT_LIMIT = 0x2020U;/**<Index for homing current limit*/

};

} /* namespace hyuEcat */

#endif /* ECATSYSTEM_ECAT_MASTER_H_ */
