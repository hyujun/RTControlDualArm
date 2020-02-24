#pragma once 

/**
 * @file PDOConfig.h
 * @brief PDO configuration
 * @date 2019-05-30
 * @author Junho Park
 * @version 1.0.0
 */
#include "ecrt.h" 

// Vendor ID & Product Code 
#define Elmo_VendorID  0x0000009a
#define Elmo_ProductCode 0x00030924

extern ec_pdo_entry_info_t 	Elmo_pdo_entries[];
extern ec_pdo_info_t		Elmo_pdos[];
extern ec_sync_info_t		Elmo_syncs[5];


#define KistHand_VendorID 0x00000590
#define KistHand_ProductCode 0x00000001

extern ec_pdo_entry_info_t 	KistHand_pdo_entries[];
extern ec_pdo_info_t 		KistHand_pdos[];
extern ec_sync_info_t 		KistHand_syncs[5];


#define KistSensor_VendorID 0x00000581
#define KistSensor_ProductCode 0x00000001

extern ec_pdo_entry_info_t 	KistSensor_pdo_entries[];
extern ec_pdo_info_t 		KistSensor_pdos[];
extern ec_sync_info_t 		KistSensor_syncs[5];
