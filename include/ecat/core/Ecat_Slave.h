/**
 * @file Ecat_Slave.h
 * @brief EtherCAT slave base class
 * @date 2018-11-11
 * @author Junho Park
 *
 * v2.0 – RT hot-path refactor:
 *   - processData(index, ptr) removed from RT loop; replaced by
 *     updateRx() / updateTx() called once per slave per cycle.
 *   - syncDomainMap() called once at activate() to cache raw PDO ptrs.
 *   - Background SDO/SoE request helpers added (non-RT safe).
 *   - CoE Emergency ring helper added (popEmergency).
 *   - ESC register request support added.
 */

#pragma once

#include "ecrt.h"
#include <map>
#include <vector>
#include <array>
#include <cstdio>

#include "ecat/core/PDOConfig.h"

namespace hyuEcat {

/**
 * @brief EtherCAT base class for each specific slave driver (Elmo, Maxon, …)
 *
 * Lifecycle:
 *   1. Construction / addSlave()  → ecrt_slave_config_pdos()
 *   2. activate() in Master       → ecrt_domain_data() obtained
 *                                 → syncDomainMap(ptrs) called once
 *   3. Per-cycle RT hot path      → updateRx() then updateTx()
 *   4. Background (non-RT)        → requestSdoRead/Write, popEmergency…
 */
class Slave
{
public:
    Slave(uint32_t vendor_id, uint32_t product_id)
        : m_vendor_id(vendor_id), m_product_id(product_id) {}
    virtual ~Slave() = default;

    // ================================================================
    //  PDO configuration descriptors  (called at init)
    // ================================================================

    virtual const ec_sync_info_t*      syncs()    { return nullptr; }
    virtual size_t                     syncSize() { return 0; }
    virtual const ec_pdo_entry_info_t* channels() { return nullptr; }

    typedef std::map<unsigned int, std::vector<unsigned int>> DomainMap;
    virtual void domains(DomainMap& domains) const {}

    // ================================================================
    //  RT hot-path interface  (1 kHz)
    // ================================================================

    /**
     * @brief Cache raw process-data pointers for O(1) RT access.
     *
     * Called once by Master::activate() after ecrt_domain_data() is
     * available.  The vector is ordered identically to the channels()
     * array so that ptrs[i] == domain_pd + offset[i].
     *
     * Derived classes must store these pointers and use them directly
     * in updateRx() / updateTx() via EC_READ_* / EC_WRITE_* macros.
     *
     * @param ptrs  Vector of domain_pd + offset[i] for each PDO entry.
     */
    virtual void syncDomainMap(const std::vector<uint8_t*>& ptrs) {}

    /**
     * @brief Read incoming TxPDO bytes from the process image.
     *
     * Called at the beginning of each RT cycle (after
     * ecrt_domain_process).  Derived classes read hardware data from
     * the cached pointers set by syncDomainMap().
     */
    virtual void updateRx() {}

    /**
     * @brief Write outgoing RxPDO bytes into the process image.
     *
     * Called at the end of each RT cycle (before ecrt_domain_queue).
     * Derived classes write command data to the cached pointers.
     */
    virtual void updateTx() {}

    /**
     * @brief Setup initial SDO configuration.
     * 
     * Called once by Master::addSlave() during pre-operational state.
     * Derived classes can override this to call ecrt_slave_config_sdo* functions.
     */
    virtual void setupSDOs(ec_slave_config_t* config) {}

    // ================================================================
    //  Advanced stable-1.6 features  (non-RT)
    // ================================================================

    /**
     * @brief Register a background SDO read request (non-RT only).
     *
     * Wraps ecrt_slave_config_create_sdo_request().  Call once at
     * init; poll requestState() each cycle if needed.
     * @param config  ec_slave_config_t* obtained at addSlave.
     * @param index   CoE object index.
     * @param subindex CoE sub-index.
     * @param size    Data size in bytes.
     * @return pointer to ec_sdo_request_t, or nullptr on failure.
     */
    ec_sdo_request_t* createSdoRequest(ec_slave_config_t* config,
                                        uint16_t index, uint8_t subindex,
                                        size_t size)
    {
        ec_sdo_request_t* req = ecrt_slave_config_create_sdo_request(
                config, index, subindex, size);
        if (!req) {
            fprintf(stderr, "WARNING. Slave. createSdoRequest failed "
                            "idx=0x%04X sub=0x%02X\n", index, subindex);
        }
        return req;
    }

    /**
     * @brief Trigger an SDO read on a pre-created request (non-RT).
     */
    static void requestSdoRead(ec_sdo_request_t* req)
    {
        if (req) ecrt_sdo_request_read(req);
    }

    /**
     * @brief Trigger an SDO write on a pre-created request (non-RT).
     */
    static void requestSdoWrite(ec_sdo_request_t* req)
    {
        if (req) ecrt_sdo_request_write(req);
    }

    /**
     * @brief Poll the state of an outstanding SDO request.
     * @return EC_REQUEST_UNUSED / EC_REQUEST_BUSY /
     *         EC_REQUEST_SUCCESS / EC_REQUEST_ERROR
     */
    static ec_request_state_t checkSdoState(ec_sdo_request_t* req)
    {
        return req ? ecrt_sdo_request_state(req) : EC_REQUEST_UNUSED;
    }

    /**
     * @brief Pop one CoE Emergency message from the slave ring buffer.
     *
     * Must be called after ecrt_slave_config_emerg_size() >= 1.
     * @param config    ec_slave_config_t* for this slave.
     * @param msg_out   Buffer of at least EC_COE_EMERGENCY_MSG_SIZE bytes.
     * @return 0 on success (msg_out filled), -ENOENT if ring is empty.
     */
    static int popEmergency(ec_slave_config_t* config, uint8_t* msg_out)
    {
        return ecrt_slave_config_emerg_pop(config, msg_out);
    }

    /**
     * @brief Create an ESC register read/write request (non-RT only).
     * @param config  ec_slave_config_t* for this slave.
     * @param size    Number of bytes to transfer.
     * @return pointer to ec_reg_request_t, or nullptr on failure.
     */
    static ec_reg_request_t* createRegRequest(ec_slave_config_t* config,
                                               size_t size)
    {
        return ecrt_slave_config_create_reg_request(config, size);
    }

    // ================================================================
    //  Homing parameters  (CiA 402, init-time)
    // ================================================================

    bool isHomingEnable() const noexcept { return homing_flag_; }

    bool setHomingParam(int32_t offset, int8_t method,
                        uint32_t speed, uint16_t currentLimit) noexcept
    {
        mHomingOffset       = offset;
        mHomingMethod       = method;
        mHomingSpeed        = speed;
        mHomingCurrentLimit = currentLimit;
        homing_flag_        = true;
        return true;
    }

    int32_t  getHomingOffset()       const noexcept { return mHomingOffset;       }
    int8_t   getHomingMethod()       const noexcept { return mHomingMethod;       }
    uint32_t getHomingSpeed()        const noexcept { return mHomingSpeed;        }
    uint16_t getHomingCurrentLimit() const noexcept { return mHomingCurrentLimit; }

    // ================================================================
    //  Alias / position
    // ================================================================

    void setSlaveAlias   (int alias)    noexcept { slave_alias    = alias;    }
    void setSlavePosition(int position) noexcept { slave_position = position; }
    int  getSlaveAlias   () const noexcept       { return slave_alias;    }
    int  getSlavePosition() const noexcept       { return slave_position; }

    // ================================================================
    //  Distributed Clock (DC) configuration
    // ================================================================

    bool     dc_enabled_         = false;
    uint16_t dc_assign_activate_ = 0x0000;
    uint32_t dc_sync0_shift_     = 0;

    // ================================================================
    //  Public identifiers
    // ================================================================

    const uint32_t m_vendor_id;
    const uint32_t m_product_id;

protected:
    int slave_alias    = 0;
    int slave_position = 0;

    bool     homing_flag_        = false;
    int32_t  mHomingOffset       = 0;
    int8_t   mHomingMethod       = 0;
    uint32_t mHomingSpeed        = 0;
    uint16_t mHomingCurrentLimit = 0;
};

} // namespace hyuEcat
