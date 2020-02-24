/**
 * @file Ecat_Slave.h
 * @brief EtherCAT slave base class
 * @date 2018-11-11
 * @author Junho Park
 */

#ifndef ECATSYSTEM_ECAT_SLAVE_H_
#define ECATSYSTEM_ECAT_SLAVE_H_

#include "ecrt.h"
#include <map>
#include <vector>
#include <iostream>

namespace hyuEcat{
/**
 * @brief EtherCAT base class for each specific client i.e., ELMO, MAXON..
 */
class Slave
{
public:

	/**
	 * @brief the slave class constructor for EtherLAB API
	 * @details before use this code, slave configuration SHOULD BE DONE using EtherLAB API
	 * @param[in] vendor_id
	 * @param[in] product_id
	 * @see PDOConfig.h
	 */
	Slave(uint32_t vendor_id, uint32_t product_id): m_vendor_id(vendor_id), m_product_id(product_id){}
	virtual ~Slave(){}

	/**
	 * @brief base method of PDO processdata
	 * @param[in] index
	 * @param[in] domain_address
	 * @see PDOConfig.h
	 */
	virtual void processData(size_t index, uint8_t* domain_address){}

	/**
	 * @brief base method for Sync
	 * @return NULL
	 */
	virtual const ec_sync_info_t* syncs(){
		return NULL;
	}
	/**
	 * @brief base method for Sync size
	 * @return 0
	 */
	virtual size_t syncSize(){
		return 0;
	}

	/**
	 * @brief base method for PDO entries
	 * @return NULL
	 */
	virtual const ec_pdo_entry_info_t* channels(){
		return NULL;
	}

	/**
	 * @brief base member for PDO domain
	 */
	typedef std::map<unsigned int, std::vector<unsigned int>> DomainMap;

	/**
	 * @brief base method for pdo domains
	 * @param[in] domains
	 */
	virtual void domains(DomainMap& domains) const{}


	/**
	 * @brief base method for set the slave alias
	 * @param[in] alias
	 */
	void setSlaveAlias(int alias){
		slave_alias = alias;
		return;
	}

	/**
	 * @brief base method for set the slave position
	 * @param[in] position
	 */
	void setSlavePosition(int position){
		slave_position = position;
		return;
	}

	/**
	 * @brief base method for get the slave alias
	 * @return slave_alias
	 */
	int getSlaveAlias(void){
		return slave_alias;
	}

	/**
	 * @brief base method for get the slave position
	 * @return slave_position
	 */
	int getSlavePosition(void){
		return slave_position;
	}

	const uint32_t m_vendor_id; /**<vendor id from slave configuration  */
	const uint32_t m_product_id;/**<product id from slave configuration  */

protected:

	int slave_alias=0; /**<slave alias from slave configuration */
	int slave_position=0; /**<slave position from slave configuration  */

};

}


#endif /* ECATSYSTEM_ECAT_SLAVE_H_ */
