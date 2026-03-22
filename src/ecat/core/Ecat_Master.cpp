#include "ecat/core/Ecat_Master.h"


namespace hyuEcat {

// ============================================================
// DomainInfo
// ============================================================

Master::DomainInfo::DomainInfo(ec_master_t* master)
{
	domain = ecrt_master_create_domain(master);
	if (domain == nullptr) {
		Master::printWarning("Failed to create domain!");
		return;
	}
	const ec_pdo_entry_reg_t empty = {};
	domain_regs.push_back(empty);
}

Master::DomainInfo::~DomainInfo()
{
	for (Entry& entry : entries) {
		delete[] entry.offset;
		delete[] entry.bit_position;
	}
}

// ============================================================
// Master ctor / dtor
// ============================================================

Master::Master(const int master)
{
	p_master = ecrt_request_master(master);
	if (p_master == nullptr) {
		printWarning("Failed to obtain master!");
		return;
	}
}

Master::~Master()
{
	for (auto& [idx, di] : m_domain_info) {
		delete di;
	}
	deactivate();
}

// ============================================================
// SDO access  (init-time only, not RT)
// sizeof(data) where data is a pointer would always return the
// pointer width (8 bytes on x86-64), so callers must pass the
// actual buffer size explicitly.
// ============================================================

void Master::SDOread(uint16_t position, uint16_t index, uint8_t subindex,
                     uint8_t *data, size_t data_size)
{
	size_t   result_size;
	uint32_t abort_data;

	if (ecrt_master_sdo_upload(p_master, position, index, subindex,
	                           data, data_size,
	                           &result_size, &abort_data) < 0) {
		fprintf(stderr,
		        "WARNING. Master. [SDOread] index:0x%04X sub:0x%02X – failed\n",
		        index, subindex);
	}
}

void Master::SDOwrite(uint16_t position, uint16_t index, uint8_t subindex,
                      uint8_t *data, size_t data_size)
{
	uint32_t abort_data;

	if (ecrt_master_sdo_download(p_master, position, index, subindex,
	                              data, data_size, &abort_data) < 0) {
		fprintf(stderr,
		        "WARNING. Master. [SDOwrite] index:0x%04X sub:0x%02X – failed\n",
		        index, subindex);
	}
}

// ============================================================
// Slave registration  (init-time only, not RT)
// ============================================================

void Master::addSlave(uint16_t alias, uint16_t position, Slave* slave)
{
	slave->setSlaveAlias(alias);
	slave->setSlavePosition(position);

	SlaveInfo slave_info;
	slave_info.slave  = slave;
	slave_info.alias  = alias;
	slave_info.config = ecrt_master_slave_config(p_master, alias, position,
	                                              slave->m_vendor_id,
	                                              slave->m_product_id);
	if (slave_info.config == nullptr) {
		printWarning("addSlave: failed to get slave configuration.");
		return;
	}
	m_slave_info.push_back(slave_info);

	const size_t num_syncs = slave->syncSize();
	if (num_syncs > 0) {
		if (ecrt_slave_config_pdos(slave_info.config, EC_END, slave->syncs())) {
			printWarning("addSlave: failed to configure PDOs.");
			return;
		}
	} else {
		fprintf(stderr, "WARNING. Master. addSlave: sync size is zero for %u:%u\n",
		        alias, position);
	}

	// Enable CoE Emergency ring (8 messages deep) for each slave.
	// Allows popEmergency() to be called after activate.
	ecrt_slave_config_emerg_size(slave_info.config, 8);

	Slave::DomainMap domain_map;
	slave->domains(domain_map);
	for (auto& [domain_index, channels] : domain_map) {
		DomainInfo*& domain_info = m_domain_info[domain_index];
		if (domain_info == nullptr) {
			domain_info = new DomainInfo(p_master);
		}
		registerPDOInDomain(alias, position, channels, domain_info, slave);
	}
}

void Master::addSlaveWithHoming(uint16_t alias, uint16_t position, EcatElmo* slave)
{
	slave->setSlaveAlias(alias);
	slave->setSlavePosition(position);

	SlaveInfo slave_info;
	slave_info.slave  = slave;
	slave_info.alias  = alias;
	slave_info.config = ecrt_master_slave_config(p_master, alias, position,
	                                              slave->m_vendor_id,
	                                              slave->m_product_id);
	if (slave_info.config == nullptr) {
		printWarning("addSlaveWithHoming: failed to get slave configuration.");
		return;
	}
	m_slave_info.push_back(slave_info);

	if (slave->isHomingEnable()) {
		fprintf(stdout, "\n-- Homing param setup -> alias:%u position:%u", alias, position);

		// Use stack buffers – no heap allocation needed here
		uint8_t s32_buf[4];
		EC_WRITE_S32(s32_buf, slave->getHomingOffset());
		SDOwrite(position, INDEX_HOMING_OFFSET, 0, s32_buf, sizeof(s32_buf));

		uint8_t h1_buf = static_cast<uint8_t>(slave->getHomingMethod());
		SDOwrite(position, INDEX_HOMING_METHOD, 0, &h1_buf, sizeof(h1_buf));

		uint8_t u32_buf[4];
		EC_WRITE_U32(u32_buf, slave->getHomingSpeed());
		SDOwrite(position, INDEX_HOMING_SPEED, SUBINDEX_HOMING_LOWSPEED,
		         u32_buf, sizeof(u32_buf));

		uint8_t u16_buf[2];
		EC_WRITE_U16(u16_buf, slave->getHomingCurrentLimit());
		SDOwrite(position, INDEX_HOMING_CURRENT_LIMIT, 1,
		         u16_buf, sizeof(u16_buf));
	}

	// Enable CoE Emergency ring
	ecrt_slave_config_emerg_size(slave_info.config, 8);

	const size_t num_syncs = slave->syncSize();
	if (num_syncs > 0) {
		if (ecrt_slave_config_pdos(slave_info.config, EC_END, slave->syncs())) {
			printWarning("addSlaveWithHoming: failed to configure PDOs.");
			return;
		}
	} else {
		fprintf(stderr, "WARNING. Master. addSlaveWithHoming: sync size is zero for %u:%u\n",
		        alias, position);
	}

	Slave::DomainMap domain_map;
	slave->domains(domain_map);
	for (auto& [domain_index, channels] : domain_map) {
		DomainInfo*& domain_info = m_domain_info[domain_index];
		if (domain_info == nullptr) {
			domain_info = new DomainInfo(p_master);
		}
		registerPDOInDomain(alias, position, channels, domain_info, slave);
	}
}

void Master::registerPDOInDomain(uint16_t alias, uint16_t position,
                                  std::vector<unsigned int>& channel_indices,
                                  DomainInfo* domain_info, Slave* slave)
{
	const unsigned int num_pdo_regs = static_cast<unsigned int>(channel_indices.size());
	const size_t start_index = domain_info->domain_regs.size() - 1; // sentinel at end

	domain_info->domain_regs.resize(domain_info->domain_regs.size() + num_pdo_regs);

	DomainInfo::Entry domain_entry;
	domain_entry.slave         = slave;
	domain_entry.num_pdos      = num_pdo_regs;
	domain_entry.offset        = new unsigned int[num_pdo_regs];
	domain_entry.bit_position  = new unsigned int[num_pdo_regs];
	domain_info->entries.push_back(domain_entry);

	// Back-reference to the Entry now living inside the vector
	DomainInfo::Entry& stored_entry = domain_info->entries.back();

	const ec_pdo_entry_info_t* pdo_regs = slave->channels();
	for (size_t i = 0; i < num_pdo_regs; ++i) {
		ec_pdo_entry_reg_t& pdo_reg = domain_info->domain_regs[start_index + i];
		pdo_reg.alias        = alias;
		pdo_reg.position     = position;
		pdo_reg.vendor_id    = slave->m_vendor_id;
		pdo_reg.product_code = slave->m_product_id;
		pdo_reg.index        = pdo_regs[channel_indices[i]].index;
		pdo_reg.subindex     = pdo_regs[channel_indices[i]].subindex;
		pdo_reg.offset       = &(stored_entry.offset[i]);
		pdo_reg.bit_position = &(stored_entry.bit_position[i]);

#if defined(_ECAT_MASTER_DEBUG_)
		fprintf(stdout, "{%u, %u, 0x%08X, 0x%08X, 0x%04X, 0x%02X}\n",
		        pdo_reg.alias, pdo_reg.position,
		        pdo_reg.vendor_id, pdo_reg.product_code,
		        pdo_reg.index, (unsigned)pdo_reg.subindex);
#endif
	}

	// Restore the null sentinel at the end
	domain_info->domain_regs.back() = ec_pdo_entry_reg_t{};
}

// ============================================================
// syncSlavePointers  (init-time only — called inside activate)
//
// After domain_pd is available, builds a vector<uint8_t*> for each
// slave entry and calls slave->syncDomainMap() so the RT hot-path
// uses direct O(1) pointer dereferences.
// ============================================================

void Master::syncSlavePointers(DomainInfo* di)
{
	uint8_t* const pd = di->domain_pd;
	for (DomainInfo::Entry& entry : di->entries) {
		std::vector<uint8_t*> ptrs(entry.num_pdos);
		for (unsigned int i = 0; i < entry.num_pdos; ++i) {
			ptrs[i] = pd + entry.offset[i];
		}
		entry.slave->syncDomainMap(ptrs);
	}
}

// ============================================================
// Activate  (init-time only, not RT)
// Sets m_domain0 so the 1 kHz hot path never touches m_domain_info.
// ============================================================

void Master::activate()
{
	for (auto& [idx, di] : m_domain_info) {
		if (ecrt_domain_reg_pdo_entry_list(di->domain, di->domain_regs.data())) {
			printWarning("activate: failed to register domain PDO entries.");
			return;
		}
	}

	if (ecrt_master_activate(p_master)) {
		printWarning("activate: failed to activate master.");
		deactivate();
		return;
	}

	for (auto& [idx, di] : m_domain_info) {
		di->domain_pd = ecrt_domain_data(di->domain);
		if (di->domain_pd == nullptr) {
			printWarning("activate: failed to retrieve domain process data.");
			return;
		}
		// Cache direct PDO pointers in each slave
		syncSlavePointers(di);
	}

	// Cache domain 0 for the RT hot path
	auto it = m_domain_info.find(0);
	if (it != m_domain_info.end()) {
		m_domain0 = it->second;
	}

	isRunning = true;
}

void Master::activateWithDC(uint8_t RefPosition, uint32_t SyncCycleNano)
{
	for (auto& [idx, di] : m_domain_info) {
		if (ecrt_domain_reg_pdo_entry_list(di->domain, di->domain_regs.data())) {
			printWarning("activateWithDC: failed to register domain PDO entries.");
			return;
		}
	}

	// sync0_shift chosen as 400 µs (roughly 40% of 1 ms cycle)
	for (SlaveInfo& si : m_slave_info) {
		ecrt_slave_config_dc(si.config, 0x0300, SyncCycleNano, 400000, 0, 0);
	}
	fprintf(stdout, "\n-- activateWithDC: DC config done");

	const int res = ecrt_master_select_reference_clock(
	        p_master, m_slave_info.at(RefPosition).config);
	if (res < 0) {
		fprintf(stderr, "\n-- activateWithDC: failed to select reference clock: %d", res);
	}
	fprintf(stdout, "\n-- activateWithDC: reference clock selected");

	if (ecrt_master_activate(p_master)) {
		printWarning("activateWithDC: failed to activate master.");
		deactivate();
		return;
	}

	for (auto& [idx, di] : m_domain_info) {
		di->domain_pd = ecrt_domain_data(di->domain);
		if (di->domain_pd == nullptr) {
			printWarning("activateWithDC: failed to retrieve domain process data.");
			return;
		}
		// Cache direct PDO pointers in each slave
		syncSlavePointers(di);
	}

	// Cache domain 0 for the RT hot path
	auto it = m_domain_info.find(0);
	if (it != m_domain_info.end()) {
		m_domain0 = it->second;
	}

	isRunning = true;
}

// ============================================================
// DC synchronisation  (called every RT cycle from TxUpdate)
// ============================================================

void Master::SyncEcatMaster(unsigned long _time)
{
	ecrt_master_application_time(p_master, _time);
	if (sync_ref_counter) {
		--sync_ref_counter;
	} else {
		sync_ref_counter = 9;
		ecrt_master_sync_reference_clock(p_master);
	}
	ecrt_master_sync_slave_clocks(p_master);
}

// ============================================================
// Deactivate
// ============================================================

void Master::deactivate()
{
	if (p_master != nullptr) {
		ecrt_release_master(p_master);
		p_master  = nullptr;
		m_domain0 = nullptr;
		isRunning = false;
	}
}

// ============================================================
// RT hot path  (1 kHz)
//
// v2.0: slave->updateRx() / updateTx() replace the inner PDO
// entry loop.  Virtual dispatch is O(num_slaves) not
// O(num_slaves * num_pdos); no switch-case inside each call.
// ============================================================

void Master::RxUpdate(unsigned int domain)
{
	ecrt_master_receive(p_master);

	// Fast O(1) path for domain 0 (the common case)
	DomainInfo* di = (domain == 0 && m_domain0 != nullptr)
	                 ? m_domain0
	                 : m_domain_info[domain];

	ecrt_domain_process(di->domain);

	checkDomainState(domain);

	// Master/slave state checks at 1/10 rate to keep hot-path cost low
	if (masterslave_counter) {
		--masterslave_counter;
	} else {
		masterslave_counter = 9;
		checkMasterState();
		checkSlaveStates();
	}

	// --- O(num_slaves) RT path ---
	for (DomainInfo::Entry& entry : di->entries) {
		entry.slave->updateRx();
	}
}

void Master::TxUpdate(unsigned int domain, unsigned long _time)
{
	// Fast O(1) path for domain 0 (the common case)
	DomainInfo* di = (domain == 0 && m_domain0 != nullptr)
	                 ? m_domain0
	                 : m_domain_info[domain];

	// --- O(num_slaves) RT path ---
	for (DomainInfo::Entry& entry : di->entries) {
		entry.slave->updateTx();
	}

	ecrt_domain_queue(di->domain);
	SyncEcatMaster(_time);
	ecrt_master_send(p_master);
}

// ============================================================
// State polling helpers (called from RT loop at 1/10 rate)
// No heap allocation: booleans replace std::string fields.
// ============================================================

void Master::checkDomainState(unsigned int domain)
{
	DomainInfo* di = (domain == 0 && m_domain0 != nullptr)
	                 ? m_domain0
	                 : m_domain_info[domain];

	ec_domain_state_t ds;
	ecrt_domain_state(di->domain, &ds);

	if (ds.working_counter != di->domain_state.working_counter) {
		di->domainWorkingCounter = ds.working_counter;
#if defined(_ECAT_MASTER_DEBUG_)
		fprintf(stdout, "Domain: WC %u.\n", di->domainWorkingCounter);
#endif
	}
	if (ds.wc_state != di->domain_state.wc_state) {
		di->domainWCState = ds.wc_state;
#if defined(_ECAT_MASTER_DEBUG_)
		fprintf(stdout, "Domain: State %u.\n", di->domainWCState);
#endif
	}
	di->domain_state = ds;
}

void Master::checkMasterState()
{
	ec_master_state_t ms;
	ecrt_master_state(p_master, &ms);

	if (ms.slaves_responding != m_master_state.slaves_responding) {
		ConnectedSlaves = ms.slaves_responding;
#if defined(_ECAT_MASTER_DEBUG_)
		fprintf(stdout, "%u slave(s).\n", ConnectedSlaves);
#endif
	}
	if (ms.al_states != m_master_state.al_states) {
		EcatMasterState = ms.al_states;
#if defined(_ECAT_MASTER_DEBUG_)
		fprintf(stdout, "Master AL states: 0x%02X.\n", EcatMasterState);
#endif
	}
	if (ms.link_up != m_master_state.link_up) {
		m_link_up = (ms.link_up != 0);
#if defined(_ECAT_MASTER_DEBUG_)
		fprintf(stdout, "Link is %s.\n", m_link_up ? "up" : "down");
#endif
	}
	m_master_state = ms;
}

void Master::checkSlaveStates()
{
	for (SlaveInfo& si : m_slave_info) {
		ec_slave_config_state_t s;
		ecrt_slave_config_state(si.config, &s);
		si.position = s.position;

		if (s.al_state != si.config_state.al_state) {
			si.SlaveState = s.al_state;
#if defined(_ECAT_MASTER_DEBUG_)
			fprintf(stdout, "Slave %u: state 0x%02X.\n", si.position, si.SlaveState);
#endif
		}
		if (s.online != si.config_state.online) {
			si.online = (s.online != 0);
#if defined(_ECAT_MASTER_DEBUG_)
			fprintf(stdout, "Slave %u: %s.\n", si.position,
			        si.online ? "online" : "offline");
#endif
		}
		if (s.operational != si.config_state.operational) {
			si.operational = (s.operational != 0);
#if defined(_ECAT_MASTER_DEBUG_)
			fprintf(stdout, "Slave %u: %s.\n", si.position,
			        si.operational ? "Operational" : "Not Operational");
#endif
		}
		si.config_state = s;
	}
}

// ============================================================
// Utilities
// ============================================================

void Master::printWarning(const char* message)
{
	fprintf(stderr, "WARNING. Master. %s\n", message);
}

void Master::printWarning(const char* message, int position)
{
	fprintf(stderr, "WARNING. Master. slave %d: %s\n", position, message);
}

} /* namespace hyuEcat */
