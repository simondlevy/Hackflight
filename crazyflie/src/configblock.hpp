#pragma once


#include <stdint.h>
#include <unistd.h>
#include <string.h>

#include <crossplatform.h>

#include <hal/eeprom.h>

#include <console.h>
#include <config.h>
#include <configblock.hpp>

/* Internal format of the config block */
#define MAGIC 0x43427830
#define VERSION 1
#define HEADER_SIZE_BYTES 5 // magic + version
#define OVERHEAD_SIZE_BYTES (HEADER_SIZE_BYTES + 1) // + cksum

#define RADIO_RATE_2M 2
#define RADIO_CHANNEL 80  //The radio channel. From 0 to 125
#define RADIO_DATARATE RADIO_RATE_2M
#define RADIO_ADDRESS 0xE7E7E7E7E7ULL

class ConfigBlock {

    public:

        int getRadioChannel(void)
        {
            if (cb_ok)
                return configblock.radioChannel;
            else
                return RADIO_CHANNEL;
        }

        int getRadioSpeed(void)
        {
            if (cb_ok)
                return configblock.radioSpeed;
            else
                return RADIO_DATARATE;
        }

        uint64_t getRadioAddress(void)
        {
            if (cb_ok)
                return ((uint64_t)configblock.radioAddress_upper << 32) | 
                    (uint64_t)configblock.radioAddress_lower;
            else
                return RADIO_ADDRESS;
        }

        float getCalibPitch(void)
        {
            if (cb_ok)
                return configblock.calibPitch;
            else
                return 0;
        }

        float getCalibRoll(void)
        {
            if (cb_ok)
                return configblock.calibRoll;
            else
                return 0;
        }

        bool test(void)
        {
            return eepromTest();
        }

        int init(void)
        {
            if(didInit)
                return 0;

            eepromInit();

            // Because of strange behavior from I2C device during expansion port test
            // the first read needs to be discarded
            eepromTestConnection();

            if (eepromTestConnection())
            {
                if (eepromReadBuffer((uint8_t *)&configblock, 0, sizeof(configblock)))
                {
                    //Verify the config block
                    if (checkMagic(&configblock))
                    {
                        if (checkVersion(&configblock))
                        {
                            if (checkChecksum(&configblock))
                            {
                                // Everything is fine
                                consolePrintf("CFGBLK v%d, verification [OK]\n", 
                                        configblock.version);
                                cb_ok = true;
                            }
                            else
                            {
                                consolePrintf("CFGBLK: Verification [FAIL]\n");
                                cb_ok = false;
                            }
                        }
                        else // configblockCheckVersion
                        {
                            // Check data integrity of old version data
                            if (configblock.version <= VERSION &&
                                    checkDataIntegrity(
                                        (uint8_t *)&configblock, configblock.version)) {
                                // Not the same version, try to upgrade
                                if (copyToNewVersion(&configblock, &configblockDefault)) {
                                    // Write updated config block to eeprom
                                    if (write(&configblock)) {
                                        cb_ok = true;
                                    }
                                }
                            }
                            else {
                                // Can't copy old version due to bad data.
                                cb_ok = false;
                            }
                        }
                    }
                }
            }

            if (cb_ok == false)
            {
                // Copy default data to used structure.
                memcpy((uint8_t *)&configblock, 
                        (uint8_t *)&configblockDefault, sizeof(configblock));
                // Write default configuration to eeprom
                if (write(&configblockDefault)) {
                    cb_ok = true;
                }
                else {
                    return -1;
                }
            }

            didInit = true;

            return 0;
        }

    private:

        // Old versions
        struct configblock_v0_s {
            /* header */
            uint32_t magic;
            uint8_t  version;
            /* Content */
            uint8_t radioChannel;
            uint8_t radioSpeed;
            float calibPitch;
            float calibRoll;
            /* Simple modulo 256 checksum */
            uint8_t cksum;
        } __attribute__((__packed__));

        // Current version
        struct configblock_v1_s {
            /* header */
            uint32_t magic;
            uint8_t  version;
            /* Content */
            uint8_t radioChannel;
            uint8_t radioSpeed;
            float calibPitch;
            float calibRoll;
            uint8_t radioAddress_upper;
            uint32_t radioAddress_lower;
            /* Simple modulo 256 checksum */
            uint8_t cksum;
        } __attribute__((__packed__));

        // Set version 1 as current version
        typedef struct configblock_v1_s configblock_t;


        configblock_t configblock;

        configblock_t configblockDefault =
        {
            .magic = MAGIC,
            .version = VERSION,
            .radioChannel = RADIO_CHANNEL,
            .radioSpeed = RADIO_DATARATE,
            .calibPitch = 0.0,
            .calibRoll = 0.0,
            .radioAddress_upper = ((uint64_t)RADIO_ADDRESS >> 32),
            .radioAddress_lower = (RADIO_ADDRESS & 0xFFFFFFFFULL),
        };

        const uint32_t configblockSizes[2] =
        {
            sizeof(struct configblock_v0_s),
            sizeof(struct configblock_v1_s),
        };

        bool didInit = false;
        bool cb_ok = false;

        uint8_t calculate_cksum(void* data, size_t len)
        {
            uint8_t * c = (uint8_t *)data;
            unsigned char cksum=0;

            for (size_t i=0; i<len; i++)
                cksum += *(c++);

            return cksum;
        }

        bool checkMagic(configblock_t *configblock)
        {
            return (configblock->magic == MAGIC);
        }

        bool checkVersion(configblock_t *configblock)
        {
            return (configblock->version == VERSION);
        }

        bool checkChecksum(configblock_t *configblock)
        {
            return (configblock->cksum == 
                    calculate_cksum(configblock, sizeof(configblock_t) - 1));
        }

        bool checkDataIntegrity(uint8_t *data, uint8_t version)
        {
            bool status = false;

            if (version == 0) {
                struct configblock_v0_s *v0 = ( struct configblock_v0_s *)data;

                status = (v0->cksum == calculate_cksum(data, 
                            sizeof(struct configblock_v0_s) - 1));
            }
            else if (version == 1) {
                struct configblock_v1_s *v1 = ( struct configblock_v1_s *)data;
                status = (v1->cksum == 
                        calculate_cksum(data, sizeof(struct configblock_v1_s) - 1));
            }

            return status;
        }

        bool write(configblock_t *configblock)
        {
            // Write default configuration to eeprom
            configblock->cksum = calculate_cksum(configblock, sizeof(configblock_t) - 1);
            if (!eepromWriteBuffer((uint8_t *)configblock, 0, sizeof(configblock_t)))
            {
                return false;
            }

            return true;
        }

        bool copyToNewVersion(
                configblock_t *configblockSaved, configblock_t *configblockNew)
        {
            configblock_t configblockTmp;

            // Copy new data to temp config memory
            memcpy((uint8_t *)&configblockTmp, 
                    (uint8_t *)configblockNew, sizeof(configblock_t));

            if (configblockSaved->version <= VERSION &&
                    sizeof(configblock_t) >= 
                    configblockSizes[configblockSaved->version]) {
                // Copy old saved eeprom data to new structure
                memcpy((uint8_t *)&configblockTmp + HEADER_SIZE_BYTES,
                        (uint8_t *)configblockSaved + HEADER_SIZE_BYTES,
                        configblockSizes[configblockSaved->version] - OVERHEAD_SIZE_BYTES);
                // Copy updated block to saved structure
                memcpy((uint8_t *)configblockSaved, 
                        (uint8_t *)&configblockTmp, sizeof(configblock_t));
            }
            else {
                return false;
            }

            return true;
        }
};
