/**
 * @file Ecat_Master.h
 * @brief EtherCAT master from SimpleCAT
 * @date 2018-11-11
 * @author Junho Park
 *
 * v2.0 – RT hot-path refactor:
 *   - RxUpdate / TxUpdate now call slave->updateRx() / updateTx() once per
 *     slave instead of looping over individual PDO entries via a virtual
 *     processData() with a switch-case.  Virtual dispatch cost reduced from
 *     O(num_slaves * num_pdos) to O(num_slaves).
 *   - activate() / activateWithDC() call slave->syncDomainMap() so that each
 *     slave caches raw domain_pd + offset[i] pointers for O(1) access.
 *   - Plan #3 additions: async SDO helpers, CoE Emergency popping, ESC
 *     register requests — exposed through Slave base class helpers that take
 *     the ec_slave_config_t* obtained at addSlave time.
 */
#pragma once

#include "ecrt.h"
#include <string>
#include <vector>
#include <map>
#include <cstdio>
#include <cstdint>
#include <unistd.h>
#include <sys/resource.h>
#include <sys/mman.h>
#include "ecat/core/Ecat_Slave.h"
#include "ecat/slaves/Ecat_Elmo.h"

// CLOCK_MONOTONIC is preferred for RT: immune to NTP/wall-clock jumps
#define CLOCK_TO_USE CLOCK_MONOTONIC
#define NSEC_PER_SEC (1000000000L)
#define TIMESPEC2NS(T) ((uint64_t) (T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)

//#define _ECAT_MASTER_DEBUG_
/**
 * @brief EherCAT namespace for EtherLAB API
 * @version 2.0.0
 */
namespace hyuEcat {

class Slave;

/**
 * @brief EtherCAT Master Client of EtherLAB API
 * @version 2.0.0
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
     * @brief Read SDO Object (blocking — init-time only, not RT)
     */
    void SDOread(uint16_t position, uint16_t index, uint8_t subindex,
                 uint8_t *data, size_t data_size);

    /**
     * @brief Write SDO Object (blocking — init-time only, not RT)
     */
    void SDOwrite(uint16_t position, uint16_t index, uint8_t subindex,
                  uint8_t *data, size_t data_size);

    /**
     * @brief Add EtherCAT slaves to EtherCAT master
     */
    void addSlave(uint16_t alias, uint16_t position, Slave* slave);

    /**
     * @brief Add EtherCAT Elmo slaves with homing to EtherCAT master
     */
    void addSlaveWithHoming(uint16_t alias, uint16_t position, EcatElmo* slave);

    /**
     * @brief Activate the master (no Distributed Clock).
     *  After ecrt_domain_data() is obtained, calls slave->syncDomainMap()
     *  for each registered slave so the RT hot-path uses direct pointers.
     */
    void activate();

    /**
     * @brief Activate the master with Distributed Clock.
     */
    void activateWithDC(uint8_t RefPosition, uint32_t SyncCycleNano);

    /**
     * @brief Synchronize master clock with slave clocks (called every RT cycle)
     */
    void SyncEcatMaster(unsigned long _time);

    /**
     * @brief Deactivate and release the master
     */
    void deactivate();

    /**
     * @brief Receive PDO data from slaves (call at start of RT cycle).
     *  Calls slave->updateRx() once per slave — O(num_slaves).
     */
    void RxUpdate(unsigned int domain = 0);

    /**
     * @brief Send PDO data to slaves (call at end of RT cycle).
     *  Calls slave->updateTx() once per slave — O(num_slaves).
     */
    void TxUpdate(unsigned int domain, unsigned long _time);

    // ---- Diagnostics (non-RT, safe to call from print task) ----

    /** @return "up" or "down" */
    const char* GetEcatMasterLinkState() const
    {
        return m_link_up ? "up" : "down";
    }

    unsigned int GetEcatMasterState() const
    {
        return EcatMasterState;
    }

    unsigned int GetConnectedSlaves() const
    {
        return ConnectedSlaves;
    }

    unsigned int GetSlaveState(int _pos) const
    {
        return m_slave_info[_pos].SlaveState;
    }

    /** @return "online" or "offline" */
    const char* GetSlaveConnected(int _pos) const
    {
        return m_slave_info[_pos].online ? "online" : "offline";
    }

    /** @return "Operational" or "Not Operational" */
    const char* GetSlaveNMT(int _pos) const
    {
        return m_slave_info[_pos].operational ? "Operational" : "Not Operational";
    }

    /**
     * @brief Scan progress (stable-1.6 feature: EC_HAVE_SCAN_PROGRESS).
     * Non-RT safe.
     */
    bool getScanProgress(ec_master_scan_progress_t& progress_out) const
    {
        if (!p_master) return false;
        ecrt_master_scan_progress(p_master, &progress_out);
        return true;
    }

private:

    void checkDomainState(unsigned int domain = 0);
    void checkMasterState();
    void checkSlaveStates();

    volatile bool isRunning = false;

    // Stored as plain POD — no heap allocation in RT path
    bool         m_link_up       = false;
    unsigned int EcatMasterState = 0;
    unsigned int ConnectedSlaves = 0;

    struct DomainInfo;

    void registerPDOInDomain(uint16_t alias, uint16_t position,
                             std::vector<unsigned int>& channel_indices,
                             DomainInfo* domain_info, Slave* slave);

    /**
     * @brief After domain_pd is valid, cache domain_pd + offset[i] into each slave.
     * Called internally from activate() / activateWithDC().
     */
    void syncSlavePointers(DomainInfo* di);

    static void printWarning(const char* message);
    static void printWarning(const char* message, int position);

    ec_master_t       *p_master       = nullptr;
    ec_master_state_t  m_master_state = {};

    struct timespec tp{};

    struct DomainInfo {
        DomainInfo(ec_master_t* master);
        ~DomainInfo();

        uint32_t domainWorkingCounter = 0;
        uint32_t domainWCState        = 0;

        ec_domain_t       *domain     = nullptr;
        ec_domain_state_t  domain_state = {};
        uint8_t           *domain_pd  = nullptr;

        std::vector<ec_pdo_entry_reg_t> domain_regs;

        struct Entry {
            Slave*        slave        = nullptr;
            unsigned int  num_pdos     = 0;
            unsigned int* offset       = nullptr;
            unsigned int* bit_position = nullptr;
        };

        std::vector<Entry> entries;
    };

    // Init-time registry (map is fine here — never accessed in RT hot path)
    std::map<unsigned int, DomainInfo*> m_domain_info;

    // Fast O(1) cache for domain 0, set during activate()
    // Used exclusively in the 1 kHz RT hot path
    DomainInfo* m_domain0 = nullptr;

    struct SlaveInfo {
        Slave*                  slave        = nullptr;
        ec_slave_config_t*      config       = nullptr;
        ec_slave_config_state_t config_state = {};
        unsigned int            SlaveState   = 0;
        // bool instead of std::string: no heap allocation in RT path
        bool                    online       = false;
        bool                    operational  = false;
        unsigned int            alias        = 0;
        unsigned int            position     = 0;
    };

    std::vector<SlaveInfo> m_slave_info;

    // Homing SDO indices (Elmo CiA 402)
    static constexpr uint16_t INDEX_HOMING_METHOD        = 0x6098U;
    static constexpr uint16_t INDEX_HOMING_SPEED         = 0x6099U;
    static constexpr uint8_t  SUBINDEX_HOMING_HIGHSPEED  = 0x01U;
    static constexpr uint8_t  SUBINDEX_HOMING_LOWSPEED   = 0x02U;
    static constexpr uint16_t INDEX_HOMING_OFFSET        = 0x607CU;
    static constexpr uint16_t INDEX_HOMING_CURRENT_LIMIT = 0x2020U;

    unsigned int sync_ref_counter    = 0;
    unsigned int masterslave_counter = 0;
};

} /* namespace hyuEcat */
