#include "PDOConfig.h"

ec_pdo_entry_info_t Elmo_pdo_entries[] = {
	    {0x607a, 0x00, 32}, 	/* Target position */
	    {0x60ff, 0x00, 32}, 	/* Target velocity */
	    {0x6071, 0x00, 16}, 	/* Target torque */
	    {0x6072, 0x00, 16}, 	/* Maximal torque */
	    {0x6040, 0x00, 16}, 	/* Controlword */
	    {0x6060, 0x00, 8}, 		/* Modes of operation */
//	    {0x0000, 0x00, 8}, 		/* Gap */

	    {0x6069, 0x00, 32}, 	/* Velocity sensor actual value */

	    {0x6064, 0x00, 32}, 	/* Position actual value */
	    {0x6077, 0x00, 16}, 	/* Torque value */
	    {0x6041, 0x00, 16}, 	/* Statusword */
	    {0x6061, 0x00, 8}, 		/* Modes of operation display */
	    {0x0000, 0x00, 8}, 		/* Gap */
};

ec_pdo_info_t Elmo_pdos[] = {
	    {0x1605, 6, Elmo_pdo_entries + 0}, /* RPDO6 Mapping */
	    {0x1a0f, 1, Elmo_pdo_entries + 6}, /* TPD0F Mapping */
	    {0x1a02, 5, Elmo_pdo_entries + 7}, /* TPDO3 Mapping */

};

ec_sync_info_t Elmo_syncs[5] = {
    {0, EC_DIR_OUTPUT, 	0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 	0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 	1, Elmo_pdos + 0, EC_WD_ENABLE},
	{3, EC_DIR_INPUT, 	2, Elmo_pdos + 1, EC_WD_DISABLE},
    {0xff}
};


ec_pdo_entry_info_t KistHand_pdo_entries[] = {
    {0x7000, 0x01, 16}, /* Command 1 */
    {0x7000, 0x02, 16}, /* Command 2 */
    {0x7000, 0x03, 16}, /* Command 3 */
    {0x7000, 0x04, 16}, /* Command 4 */
    {0x6000, 0x01, 16}, /* MotorPos 1 */
    {0x6000, 0x02, 16}, /* MotorPos 2 */
    {0x6000, 0x03, 16}, /* MotorPos 3 */
    {0x6000, 0x04, 16}, /* MotorPos 4 */
    {0x6000, 0x05, 16}, /* MotorVel 1 */
    {0x6000, 0x06, 16}, /* MotorVel 2 */
    {0x6000, 0x07, 16}, /* MotorVel 3 */
    {0x6000, 0x08, 16}, /* MotorVel 4 */
    {0x6000, 0x09, 16}, /* MotorInput 1 */
    {0x6000, 0x0a, 16}, /* MotorInput 2 */
    {0x6000, 0x0b, 16}, /* MotorInput 3 */
    {0x6000, 0x0c, 16}, /* MotorInput 4 */
    {0x6000, 0x0d, 16}, /* MotorTorq 1 */
    {0x6000, 0x0e, 16}, /* MotorTorq 2 */
    {0x6000, 0x0f, 16}, /* Status */
    {0x6000, 0x10, 16}, /* Timer Tick */
};

ec_pdo_info_t KistHand_pdos[] = {
    {0x1600, 4, KistHand_pdo_entries + 0}, /* Output mapping 0 */
    {0x1a00, 16, KistHand_pdo_entries + 4}, /* Input mapping 0 */
};

ec_sync_info_t KistHand_syncs[5] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, KistHand_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, KistHand_pdos + 1, EC_WD_DISABLE},
    {0xff}
};

ec_pdo_entry_info_t KistSensor_pdo_entries[] = {
    {0x7000, 0x01, 16}, /* Command 1 */
    {0x7000, 0x02, 16}, /* Command 2 */
    {0x7000, 0x03, 16}, /* Command 3 */
    {0x7000, 0x04, 16}, /* Command 4 */
    {0x6000, 0x01, 16}, /* SensorVAL_L 0 */
    {0x6000, 0x02, 16}, /* SensorVAL_L 1 */
    {0x6000, 0x03, 16}, /* SensorVAL_L 2 */
    {0x6000, 0x04, 16}, /* SensorVAL_L 3 */
    {0x6000, 0x05, 16}, /* SensorVAL_L 4 */
    {0x6000, 0x06, 16}, /* SensorVAL_L 5 */
    {0x6000, 0x07, 16}, /* SensorVAL_L 6 */
    {0x6000, 0x08, 16}, /* SensorVAL_L 7 */
    {0x6000, 0x09, 16}, /* SensorVAL_L 8 */
    {0x6000, 0x0a, 16}, /* SensorVAL_R 0 */
    {0x6000, 0x0b, 16}, /* SensorVAL_R 1 */
    {0x6000, 0x0c, 16}, /* SensorVAL_R 2 */
    {0x6000, 0x0d, 16}, /* SensorVAL_R 3 */
    {0x6000, 0x0e, 16}, /* SensorVAL_R 4 */
    {0x6000, 0x0f, 16}, /* SensorVAL_R 5 */
    {0x6000, 0x10, 16}, /* SensorVAL_R 6 */
    {0x6000, 0x11, 16},
    {0x6000, 0x12, 16},
    {0x6000, 0x13, 16},
};

ec_pdo_info_t KistSensor_pdos[] = {
    {0x1600, 4, KistSensor_pdo_entries + 0}, /* Output mapping 0 */
    {0x1a00, 19, KistSensor_pdo_entries + 4}, /* Input mapping 0 */
};

ec_sync_info_t KistSensor_syncs[5] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, KistSensor_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, KistSensor_pdos + 1, EC_WD_DISABLE},
    {0xff}
};



