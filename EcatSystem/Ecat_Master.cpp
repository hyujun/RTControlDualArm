#include "Ecat_Master.h"


namespace hyuEcat {

Master::DomainInfo::DomainInfo(ec_master_t* master)
{
	domain = ecrt_master_create_domain(master);
	if(domain == NULL){
		printWarning("Err: Failed to create domain!");
		return;
	}
	const ec_pdo_entry_reg_t empty = {0};
	domain_regs.push_back(empty);
}

Master::DomainInfo::~DomainInfo(void)
{
	for(Entry& entry : entries){
		delete [] entry.offset;
		delete [] entry.bit_position;
	}
}


Master::Master(const int master) {
	p_master = ecrt_request_master(master);
	if(p_master == NULL){
		printWarning("Err: Failed to obtain master!");
		ecrt_release_master(p_master);
		return;
	}
}

Master::~Master() {

	//for(SlaveInfo& slave : m_slave_info){
	//}
	for(auto& domain : m_domain_info){
		delete domain.second;
	}

}

void Master::SDOread(uint16_t position, uint16_t index, uint8_t subindex, uint8_t *data)
{
	size_t result_size;
	uint32_t abort_data;

	if(ecrt_master_sdo_upload(p_master, position, index, subindex, data, sizeof(data), &result_size, &abort_data) < 0){
		printf("WARNING. Master. [SDOread], index:0x%04X, subindex:0x%02X, unable to process\n", index, subindex);
	}
	return;
}

void Master::SDOwrite(uint16_t position, uint16_t index, uint8_t subindex, uint8_t *data)
{
	uint32_t abort_data;

	if(ecrt_master_sdo_download(p_master, position, index, subindex, data, sizeof(data), &abort_data) < 0){
		printf("WARNING. Master. [SDOwrite], index:0x%04X, subindex:0x%02X, unable to process\n", index, subindex);
	}
	return;
}

void Master::addSlave(uint16_t alias, uint16_t position, Slave* slave)
{
	slave->setSlaveAlias(alias);
	slave->setSlavePosition(position);

	SlaveInfo slave_info;
	slave_info.slave = slave;
	slave_info.config = ecrt_master_slave_config(p_master, alias, position, slave->m_vendor_id, slave->m_product_id);

	if(slave_info.config == NULL){
		printWarning("Err: Add slave, Failed to get slave configuration.");
		return;
	}
	m_slave_info.push_back(slave_info);

	size_t num_syncs = slave->syncSize();
	const ec_sync_info_t* syncs = slave->syncs();
	if(num_syncs > 0){
		int pdos_status = ecrt_slave_config_pdos(slave_info.config, num_syncs, syncs);
		if(pdos_status){
			printWarning("Err: Add slave Failed to configure PDOs");

			return;
		}
	}
	else{
		printWarning("Add slave. Sync size is zero for "
				+ static_cast<std::ostringstream*>( &(std::ostringstream() << alias) )->str()
				+ ":"
				+ static_cast<std::ostringstream*>( &(std::ostringstream() << position) )->str());
	}

	Slave::DomainMap domain_map;
	slave->domains(domain_map);
	for(auto& iter : domain_map){
		unsigned int domain_index = iter.first;
		DomainInfo* domain_info = m_domain_info[domain_index];
		if(domain_info == NULL){
			domain_info = new DomainInfo(p_master);
			m_domain_info[domain_index] = domain_info;
		}
		registerPDOInDomain(alias, position, iter.second, domain_info, slave);
	}
}

void Master::addSlaveWithHoming(uint16_t alias, uint16_t position, EcatElmo* slave)
{
	slave->setSlaveAlias(alias);
	slave->setSlavePosition(position);

	SlaveInfo slave_info;  //instance of struct
	slave_info.slave = slave;
	slave_info.config = ecrt_master_slave_config(p_master, alias, position, slave->m_vendor_id, slave->m_product_id);

	if(slave_info.config == NULL){
		printWarning("Err: Add slave, Failed to get slave configuration.");
		return;
	}
	m_slave_info.push_back(slave_info);

	if(slave->isHomingEnable())
	{
		//Homing Precess
		fprintf(stdout, "\n-- Homming Param Setup-> Alias:%d, Position:%d", alias, position);

		uint8_t* si32_data = new uint8_t[4];
		EC_WRITE_S32(si32_data, slave->getHomingOffset());
		SDOwrite(position, INDEX_HOMING_OFFSET, 0, si32_data);
		delete[] si32_data;

		uint8_t h1_data = slave->getHomingMethod();
		SDOwrite(position, INDEX_HOMING_METHOD, 0, &h1_data);

		uint8_t *u32_data = new uint8_t[4];
		EC_WRITE_U32(u32_data, slave->getHomingSpeed());
		SDOwrite(position, INDEX_HOMING_SPEED, 2, u32_data);
		delete[] u32_data;

		uint8_t *u16_data = new uint8_t[2];
		EC_WRITE_U16(u16_data, slave->getHomingCurrentLimit());
		SDOwrite(position, INDEX_HOMING_CURRENT_LIMIT, 1, u16_data);
		delete[] u16_data;
	}

	size_t num_syncs = slave->syncSize();
	const ec_sync_info_t* syncs = slave->syncs();
	if(num_syncs > 0){
		int pdos_status = ecrt_slave_config_pdos(slave_info.config, num_syncs, syncs);
		if(pdos_status){
			printWarning("Err: Add slave Failed to configure PDOs");

			return;
		}
	}
	else{
		printWarning("Add slave. Sync size is zero for "
				+ static_cast<std::ostringstream*>( &(std::ostringstream() << alias) )->str()
				+ ":"
				+ static_cast<std::ostringstream*>( &(std::ostringstream() << position) )->str());
	}

	Slave::DomainMap domain_map;
	slave->domains(domain_map);
	for(auto& iter : domain_map){
		unsigned int domain_index = iter.first;
		DomainInfo* domain_info = m_domain_info[domain_index];
		if(domain_info == NULL){
			domain_info = new DomainInfo(p_master);
			m_domain_info[domain_index] = domain_info;
		}
		registerPDOInDomain(alias, position, iter.second, domain_info, slave);
	}
}

void Master::registerPDOInDomain(uint16_t alias, uint16_t position, std::vector<unsigned int>& channel_indices, DomainInfo* domain_info, Slave* slave)
{
    // expand the size of the domain
    unsigned int num_pdo_regs = channel_indices.size();
    size_t start_index = domain_info->domain_regs.size()-1; //empty element at end
    domain_info->domain_regs.resize(domain_info->domain_regs.size()+num_pdo_regs);

    // create a new entry in the domain
    DomainInfo::Entry domain_entry;
    domain_entry.slave        = slave;
    domain_entry.num_pdos     = num_pdo_regs;
    domain_entry.offset       = new unsigned int[num_pdo_regs];
    domain_entry.bit_position = new unsigned int[num_pdo_regs];
    domain_info->entries.push_back(domain_entry);

    Slave::DomainMap domain_map;
    slave->domains(domain_map);

    // add to array of pdos registrations
    const ec_pdo_entry_info_t* pdo_regs = slave->channels();
    for (size_t i=0; i<num_pdo_regs; ++i)
    {
        // create pdo entry in the domain
        ec_pdo_entry_reg_t& pdo_reg = domain_info->domain_regs[start_index+i];
        pdo_reg.alias       = alias;
        pdo_reg.position    = position;
        pdo_reg.vendor_id   = slave->m_vendor_id;
        pdo_reg.product_code= slave->m_product_id;
        pdo_reg.index       = pdo_regs[channel_indices[i]].index;
        pdo_reg.subindex    = pdo_regs[channel_indices[i]].subindex;
        pdo_reg.offset      = &(domain_entry.offset[i]);
        pdo_reg.bit_position= &(domain_entry.bit_position[i]);

        // print the domain pdo entry
#if defined(_ECAT_MASTER_DEBUG_)
        std::cout << "{" << pdo_reg.alias <<", "<< pdo_reg.position;
        std::cout << ", 0x" << std::hex << pdo_reg.vendor_id;
        std::cout << ", 0x" << std::hex << pdo_reg.product_code;
        std::cout << ", 0x" << std::hex << pdo_reg.index;
        std::cout << ", 0x" << std::hex << (int)pdo_reg.subindex;
        std::cout << ", " << pdo_reg.offset;
        std::cout << ", " << pdo_reg.bit_position;
        std::cout << "}" << std::dec << std::endl;
#endif
    }

    // set the last element to null
    ec_pdo_entry_reg_t empty = {0};
    domain_info->domain_regs.back() = empty;
}

void Master::activate(void)
{
    // register domain
    for (auto& iter : m_domain_info){
        DomainInfo* domain_info = iter.second;
        bool domain_status = ecrt_domain_reg_pdo_entry_list(domain_info->domain, &(domain_info->domain_regs[0]));
        if (domain_status){
            printWarning("Activate. Failed to register domain PDO entries.");
            return;
        }
    }

    // activate master
    bool activate_status = ecrt_master_activate(p_master);
    if (activate_status){
        printWarning("Activate. Failed to activate master.");
        deactivate();
        return;
    }

    // retrieve domain data
    for (auto& iter : m_domain_info){
        DomainInfo* domain_info = iter.second;
        domain_info->domain_pd = ecrt_domain_data(domain_info->domain);
        if (domain_info->domain_pd==NULL){
            printWarning("Activate. Failed to retrieve domain process data.");
            return;
        }
    }
}

void Master::activateWithDC(uint8_t RefPosition, uint32_t SyncCycleNano)
{
    // register domain
    for (auto& iter : m_domain_info){
        DomainInfo* domain_info = iter.second;
        bool domain_status = ecrt_domain_reg_pdo_entry_list(domain_info->domain, &(domain_info->domain_regs[0]));
        if (domain_status){
            printWarning("Activate. Failed to register domain PDO entries.");
            return;
        }
    }

    for (SlaveInfo& slave : m_slave_info)
    {
    	ecrt_slave_config_dc(slave.config, 0x0300, SyncCycleNano, 0, 0, 0 );
    }
    fprintf(stdout, "\n-- activeWithDC: ecrt_slave config dc is done");
    int res = ecrt_master_select_reference_clock(p_master, m_slave_info.at(RefPosition).config );  //error point
    if(res < 0) {
    	fprintf(stdout, "\n-- activeWithDC: Failed to select reference clock:%d", res);
    }
    fprintf(stdout, "\n-- activeWithDC: ecrt_slave reference clock is chosen");

    bool activate_status = ecrt_master_activate(p_master);
    if (activate_status){
        printWarning("Activate. Failed to activate master.");
        deactivate();
        return;
    }

    for (auto& iter : m_domain_info){
        DomainInfo* domain_info = iter.second;
        domain_info->domain_pd = ecrt_domain_data(domain_info->domain);
        if (domain_info->domain_pd==NULL){
            printWarning("Activate. Failed to retrieve domain process data.");
            return;
        }
    }
    return;
}

void Master::SyncEcatMaster(uint64_t RefTime)
{
	struct timespec tp;

	clock_gettime(CLOCK_MONOTONIC, &tp);
	ecrt_master_application_time(p_master, RefTime);
	ecrt_master_sync_reference_clock( p_master );
	ecrt_master_sync_slave_clocks( p_master );
	return;
}

void Master::deactivate(void)
{
	ecrt_release_master(p_master);
	p_master = NULL;
	return;
}

void Master::update(unsigned int domain)
{
    // receive process data
    ecrt_master_receive(p_master);

    DomainInfo* domain_info = m_domain_info[domain];

    ecrt_domain_process(domain_info->domain);

    // check process data state (optional)
    //checkDomainState(domain);

    // check for master and slave state change

	checkMasterState();
	checkSlaveStates();

    // read and write process data
    for (DomainInfo::Entry& entry : domain_info->entries){
        for (int i=0; i<entry.num_pdos; ++i){
            (entry.slave)->processData(i, domain_info->domain_pd + entry.offset[i]);
        }
    }

    // send process data
    ecrt_domain_queue(domain_info->domain);
    ecrt_master_send(p_master);
}

void Master::TxUpdate(unsigned int domain)
{
    DomainInfo* domain_info = m_domain_info[domain];

    // read and write process data
    for (DomainInfo::Entry& entry : domain_info->entries){
        for (int i=0; i<entry.num_pdos; ++i){
            (entry.slave)->processData(i, domain_info->domain_pd + entry.offset[i]);
        }
    }

    // send process data
    ecrt_domain_queue(domain_info->domain);
    ecrt_master_send(p_master);
    return;
}

void Master::RxUpdate(unsigned int domain)
{
    ecrt_master_receive(p_master);

    DomainInfo* domain_info = m_domain_info[domain];

    ecrt_domain_process(domain_info->domain);

    // check process data state (optional)
    //checkDomainState();

    // check for master and slave state change
    checkMasterState();
	checkSlaveStates();

    // read and write process data
    for (DomainInfo::Entry& entry : domain_info->entries){
        for (int i=0; i<entry.num_pdos; ++i){
            (entry.slave)->processData(i, domain_info->domain_pd + entry.offset[i]);
        }
    }
    return;
}


void Master::checkDomainState(unsigned int domain)
{
    DomainInfo* domain_info = m_domain_info[domain];

    ec_domain_state_t ds;
    ecrt_domain_state(domain_info->domain, &ds);

    if (ds.working_counter != domain_info->domain_state.working_counter){
    	domain_info->domainWorkingCounter = ds.working_counter;
#if defined(_ECAT_MASTER_DEBUG_)
        printf("Domain: WC %u.\n", ds.working_counter);
#endif
    }
    if (ds.wc_state != domain_info->domain_state.wc_state){
    	domain_info->domainWCState = ds.wc_state;
#if defined(_ECAT_MASTER_DEBUG_)
        printf("Domain: State %u.\n", ds.wc_state);
#endif
    }
    domain_info->domain_state = ds;
}

void Master::checkMasterState()
{
    ec_master_state_t ms;
    ecrt_master_state(p_master, &ms);

    if (ms.slaves_responding != m_master_state.slaves_responding){
    	this->ConnectedSlaves = ms.slaves_responding;
#if defined(_ECAT_MASTER_DEBUG_)
        printf("%u slave(s).\n", ms.slaves_responding);
#endif
    }
    if (ms.al_states != m_master_state.al_states){
    	this->EcatMasterState = ms.al_states;
#if defined(_ECAT_MASTER_DEBUG_)
        printf("Master AL states: 0x%02X.\n", ms.al_states);
#endif
    }
    if (ms.link_up != m_master_state.link_up){
    	this->EcatMasterLinkState = ms.link_up ? "up" : "down";
#if defined(_ECAT_MASTER_DEBUG_)
        printf("Link is %s.\n", ms.link_up ? "up" : "down");
#endif
    }
    m_master_state = ms;
}


void Master::checkSlaveStates()
{
	int id_count = 0;
    for (SlaveInfo& slave : m_slave_info)
    {
        ec_slave_config_state_t s;
        ecrt_slave_config_state(slave.config, &s);

        if (s.al_state != slave.config_state.al_state){
            //this spams the terminal at initialization.
        	slave.SlaveState = s.al_state;
#if defined(_ECAT_MASTER_DEBUG_)
        	printf( "Slave %d:  ", id_count );
            printf(" State 0x%02X.\n", s.al_state);
#endif
        }
        if (s.online != slave.config_state.online){
        	slave.SlaveConnected = s.online ? "online" : "offline";
#if defined(_ECAT_MASTER_DEBUG_)
        	printf( "Slave %d:  ", id_count );
            printf(" %s.\n", s.online ? "online" : "offline");
#endif
        }
        if (s.operational != slave.config_state.operational){
        	slave.SlaveNMT = s.operational ? "Operational" : "Not Operational";
#if defined(_ECAT_MASTER_DEBUG_)
        	printf( "Slave %d:  ", id_count );
            printf(" %s operational.\n", s.operational ? "" : "Not ");
#endif
        }
        slave.config_state = s;
        ++id_count;
    }
}

void Master::printWarning(const std::string& message)
{
    std::cout << "WARNING. Master. " << message << std::endl;
}

void Master::printWarning(const std::string& message, int position)
{
    std::cout << "WARNING. Master. " << "slave ID: " <<  position << message << std::endl;
}

} /* namespace hyuEcat */
