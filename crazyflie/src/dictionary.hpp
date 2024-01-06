/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 - 2020 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

class Dictionary {

    public:

        typedef bool (*iterFunc_t)(const char *key, void *buffer, size_t length);

        typedef size_t (*readFunc_t)(size_t address, void* data, size_t length);

        typedef size_t (*writeFunc_t)(size_t address, const void* data, size_t length);

        typedef struct stats {
            size_t totalSize;
            size_t totalItems;
            size_t itemSize;
            size_t keySize;
            size_t dataSize;
            size_t metadataSize;
            size_t holeSize;
            size_t freeSpace;
            size_t fragmentation;
            size_t spaceLeftUntilForcedDefrag;

        } stats_t;

        void init(readFunc_t readFunc, writeFunc_t writeFunc, const size_t memorySize)
        {
            _readFunc = readFunc;
            _writeFunc = writeFunc;
            _memorySize = memorySize;
        }

        bool store(const char* key, const void* buffer, size_t length) 
        {
            // Search if the key is already present in the table
            auto itemAddress = findItemByKey(FIRST_ITEM_ADDRESS, key);
            if (storage_is_valid(itemAddress) == false) {
                // Item does not exit, find the end of the table to insert it
                return appendItemToEnd(FIRST_ITEM_ADDRESS, key, buffer, length);
            } else {
                // Item exist, verify that the data has the same size
                auto currentItem = getItemInfo(itemAddress);
                auto newLength = length + 3 + strlen(key);
                if (currentItem.full_length != newLength) {
                    // If not, delete the item and find the end of the table
                    writeHole(itemAddress, currentItem.full_length);
                    return appendItemToEnd(FIRST_ITEM_ADDRESS, key, buffer, length);
                } else {
                    writeItem(itemAddress, key, buffer, length);
                }
            }

            return true;
        }

        //
        // We will use the function findItemByPrefix to find the first item
        // with a key that matches our prefix.
        // The function will return the size of that item, which we can use to then
        // look for the next item that after that that matches our prefix.
        //
        // The function findItemByPrefix also gives us the key and item
        // address as out-arguments, which we can use as arguments to the supplied
        // user function that will be run for each item found.
        //
        // We keep going until we get an invalid address back, which mean we have
        // reached the end.
        //
        bool foreach(const char *prefix, iterFunc_t func)
        {
            size_t itemAddress = 0;
            static char keyBuffer[64];
            auto itemSize = findItemByPrefix(FIRST_ITEM_ADDRESS, prefix, 
                    keyBuffer, &itemAddress);

            while (storage_is_valid(itemAddress)) {

                const int bufferLength = 8;
                char buffer[bufferLength] = {};

                auto header = getItemInfo(itemAddress);

                const auto storeSize = header.full_length - 
                    header.key_length - sizeof(header);

                auto readLength = min(storeSize, bufferLength);

                if (getBuffer(itemAddress, header, buffer, readLength)) {
                    if (!func((const char *) keyBuffer, buffer, readLength)) {
                        return false;
                    }
                } else {
                    return false;
                }

                itemSize = findItemByPrefix(itemAddress + itemSize, prefix, 
                        keyBuffer, &itemAddress);
            }

            return true;
        }

        size_t fetch(const char* key, void* buffer, size_t bufferLength)
        {
            auto itemAddress = findItemByKey(FIRST_ITEM_ADDRESS, key);

            if (storage_is_valid(itemAddress)) {
                auto header = getItemInfo(itemAddress);
                const auto storeSize = header.full_length - 
                    header.key_length - sizeof(header);
                auto readLength = min(storeSize, bufferLength);
                return getBuffer(itemAddress, header, buffer, readLength);
            }
            return 0;
        }

        bool remove(const char* key) 
        {
            auto itemAddress = findItemByKey(FIRST_ITEM_ADDRESS, key);

            if (storage_is_valid(itemAddress)) {
                auto itemInfo = getItemInfo(itemAddress);
                writeHole(itemAddress, itemInfo.full_length);
                return true;
            }

            return false;
        }

        void format(void) 
        {
            auto version = VERSION;
            _writeFunc(VERSION_ADDRESS, &version, 1);
            writeEnd(FIRST_ITEM_ADDRESS);
        }


        void getStats(stats_t *stats) 
        {
            auto item_address = FIRST_ITEM_ADDRESS;

            auto end_address = findEnd(FIRST_ITEM_ADDRESS);

            size_t total_size = 0;
            size_t total_items = 0;
            size_t hole_size = 0;
            size_t item_size = 0;
            size_t data_size = 0;
            size_t key_size = 0;
            size_t metadata_size = 0;

            while (item_address < end_address) {
                itemHeader_t itemInfo = getItemInfo(item_address);

                if (itemInfo.full_length == END_TAG) {
                    break;
                }

                total_size += itemInfo.full_length;

                if (itemInfo.key_length == 0) {
                    hole_size += itemInfo.full_length;
                } else {
                    item_size += itemInfo.full_length;
                    total_items++;

                    key_size += itemInfo.key_length;
                    metadata_size += sizeof(itemInfo);
                    data_size += itemInfo.full_length - 
                        itemInfo.key_length - sizeof(itemInfo);
                }

                item_address = item_address + itemInfo.full_length;
            }

            stats->totalSize = _memorySize;
            stats->totalItems = total_items;
            stats->itemSize = item_size;
            stats->keySize = key_size;
            stats->dataSize = data_size;
            stats->metadataSize = metadata_size;
            stats->holeSize = hole_size;
            stats->freeSpace = _memorySize - item_size;
            stats->fragmentation = (hole_size * 100) / (_memorySize - item_size);
            stats->spaceLeftUntilForcedDefrag = _memorySize - total_size;
        }

        bool check(void) 
        {
            // Check version
            uint8_t version = 0;
            _readFunc(VERSION_ADDRESS, &version, 1);
            if (version != VERSION) {
                return false;
            }

            // Check table consistency
            auto endAddress = findEnd(FIRST_ITEM_ADDRESS);

            // If it is not possible to get to the end tag, the table is corupted
            if (!storage_is_valid(endAddress)) {
                return false;
            }

            return true;
        }

    private:

        static const size_t END_TAG_LENGTH = 2;

        static const uint16_t END_TAG = 0xffffu;

        static const size_t FIRST_ITEM_ADDRESS = 1;

        static const size_t VERSION_ADDRESS  = 0;

        static const uint8_t VERSION = 1;

        static const size_t STORAGE_INVALID_ADDRESS = SIZE_MAX;

        typedef struct itemHeader_s {
            uint16_t full_length;
            uint8_t key_length;
        } __attribute((packed)) itemHeader_t;

        static bool storage_is_valid(const uint32_t a)
        {
            return a != SIZE_MAX;
        }

        static size_t min(size_t a, size_t b)
        {
            return a < b ? a : b;
        }

        size_t _memorySize;
        readFunc_t _readFunc;
        writeFunc_t _writeFunc;

        uint16_t writeEnd(size_t address) 
        {
            auto endTag = END_TAG;

            _writeFunc(address, &endTag, 2);

            return 2;
        }

        uint16_t writeHole(size_t address, size_t full_length) 
        {
            itemHeader_t header = {};
            header.full_length = full_length;
            header.key_length = 0;

            _writeFunc(address, &header, sizeof(header));

            return full_length;
        }

        int writeItem(size_t address, const char* key, const void* buffer, 
                size_t length)
        {
            itemHeader_t header = {};
            header.key_length = strlen(key);
            header.full_length = 2 + 1 + header.key_length + length;

            // Write full_length, key_length, key and buffer
            _writeFunc(address, &header, sizeof(header));
            _writeFunc(address + sizeof(header), key, header.key_length);
            _writeFunc(address + sizeof(header) + header.key_length, buffer, length);

            return header.full_length;
        }

        void moveMemory(size_t sourceAddress, size_t destinationAddress, 
                size_t length)
        {
            static char moveBuffer[32];
            auto leftToMove = length;

            while (leftToMove > 0) {
                auto moving = min(leftToMove, (size_t)32);
                _readFunc(sourceAddress, moveBuffer, moving);
                _writeFunc(destinationAddress, moveBuffer, moving);

                sourceAddress += moving;
                destinationAddress += moving;
                leftToMove -= moving;
            }

        }

        size_t findEnd(size_t address) 
        {
            auto currentAddress = address;
            itemHeader_t header = {};

            while (currentAddress < (_memorySize - 2)) {
                _readFunc(currentAddress, &header, sizeof(header));
                if (header.full_length == END_TAG) {
                    return currentAddress;
                }

                // An item must at least have a key of len>=1
                if (header.full_length < (sizeof(header) + 1)) {
                    return STORAGE_INVALID_ADDRESS;
                }
                currentAddress += header.full_length;
            }

            // This is a corrupted table!
            return STORAGE_INVALID_ADDRESS;
        }

        size_t findHole(size_t address) 
        {
            auto currentAddress = address;
            itemHeader_t header = {};

            while (currentAddress < (_memorySize - 2)) {
                _readFunc(currentAddress, &header, sizeof(header));
                if (header.key_length == 0) {
                    return currentAddress;
                }
                currentAddress += header.full_length;
            }

            // This is a corrupted table!
            return STORAGE_INVALID_ADDRESS;
        }

        size_t findNextItem(size_t address)
        {
            auto currentAddress = address;

            itemHeader_t header = {};

            // Jump over the current item
            _readFunc(currentAddress, &header, sizeof(header));
            if (header.full_length == END_TAG) {
                return STORAGE_INVALID_ADDRESS;
            }
            currentAddress += header.full_length;

            while (currentAddress < (_memorySize - 3)) {
                _readFunc(currentAddress, &header, sizeof(header));


                if (header.full_length == END_TAG) {
                    return STORAGE_INVALID_ADDRESS;
                }

                if (header.key_length != 0) {
                    return currentAddress;
                }

                currentAddress += header.full_length;
            }

            return STORAGE_INVALID_ADDRESS;
        }

        void defrag(void) 
        {
            auto holeAddress = findHole(FIRST_ITEM_ADDRESS);
            size_t itemAddress = 0;
            size_t nextHoleAddress = 0;

            while(storage_is_valid(holeAddress)) {

                itemAddress = findNextItem(holeAddress);

                if (storage_is_valid(itemAddress) == false) {
                    // This hole is at the end, lets crop it
                    writeEnd(holeAddress);
                    break;
                }

                nextHoleAddress = findHole(itemAddress);

                if(storage_is_valid(nextHoleAddress) == false) {
                    // If there is the end after this group of item, lets copy
                    // up to the end
                    nextHoleAddress = findEnd(itemAddress);
                }

                auto lenghtToMove = nextHoleAddress - itemAddress;

                moveMemory(itemAddress, holeAddress, lenghtToMove);

                writeHole(holeAddress + lenghtToMove, 
                        itemAddress - holeAddress);

                holeAddress = holeAddress + lenghtToMove;
            }
        }

        // Utility function
        bool appendItemToEnd(size_t address, const char* key, 
                const void* buffer, size_t length) 
        {
            auto itemAddress = findEnd(address);

            // If it is over the end of the memory, table corrupted
            // Do not write anything ...
            if (storage_is_valid(itemAddress) == false) {
                return false;
            }

            // Test that there is enough space to write the item
            if ((itemAddress + sizeof(itemHeader_t) + strlen(key) + 
                        length + END_TAG_LENGTH) < _memorySize) {
                itemAddress += writeItem(itemAddress, key, buffer, length);
                writeEnd(itemAddress);
            } else {
                // Otherwise, defrag and try to insert again!
                defrag();

                itemAddress = findEnd(FIRST_ITEM_ADDRESS);

                if ((itemAddress + sizeof(itemHeader_t) + strlen(key) + 
                            length + END_TAG_LENGTH) < _memorySize) {
                    itemAddress += writeItem(itemAddress, key,
                            buffer, length); writeEnd(itemAddress);
                } else {
                    // Memory full!
                    return false;
                }
            }

            return true;
        }

        size_t findItemByKey(size_t address, const char * key) 
        {
            static char searchBuffer[255];
            auto currentAddress = address;
            uint16_t length = 0;
            uint8_t keyLength = 0;
            auto searchedKeyLength = strlen(key);


            while (currentAddress < (_memorySize - 3)) {
                _readFunc(currentAddress, searchBuffer, 3);
                length = searchBuffer[0] + (searchBuffer[1]<<8);
                keyLength = searchBuffer[2];

                if (length == END_TAG) {
                    return SIZE_MAX;
                }

                if (keyLength == searchedKeyLength) {
                    _readFunc(currentAddress + 3, &searchBuffer, searchedKeyLength);
                    if (!memcmp(key, searchBuffer, keyLength)) {
                        return currentAddress;
                    }
                }

                currentAddress += length;
            }

            return SIZE_MAX;
        }

        itemHeader_t getItemInfo(size_t address)
        {
            itemHeader_t header = {};

            _readFunc(address, &header, sizeof(header));

            return header;
        }

        size_t getBufferLength(itemHeader_t header)
        {
            return header.full_length - sizeof(header) - header.key_length;
        }

        size_t getBuffer(size_t address, itemHeader_t header, 
                void* buffer, size_t maxLength)
        {
            auto sizeToRead = min(getBufferLength(header), maxLength);

            _readFunc(address + sizeof(header) + header.key_length, buffer, sizeToRead);

            return sizeToRead;
        }

        // Find the first item from `address` with a key that has an overlapping
        // prefix with the one we supply.
        // We return the itemsize using return, and we return the key and itemAddress
        // using out-argumetns.
        size_t findItemByPrefix(size_t address, const char *prefix, 
                char *keyBuffer, size_t *itemAddress)
        {
            static uint8_t searchBuffer[255];
            auto currentAddress = address;
            uint16_t length = 0;
            uint8_t keyLength = 0;
            auto searchedKeyLength = strlen(prefix);

            while (currentAddress < (_memorySize - 3)) {
                _readFunc(currentAddress, searchBuffer, 3);
                length = searchBuffer[0] + (searchBuffer[1]<<8);
                keyLength = searchBuffer[2];

                if (length == END_TAG) {
                    *itemAddress = SIZE_MAX;
                    return SIZE_MAX;
                }

                if (keyLength >= searchedKeyLength) {
                    _readFunc(currentAddress + 3, &searchBuffer, keyLength);
                    if (!memcmp(prefix, searchBuffer, searchedKeyLength)) {
                        memcpy(keyBuffer, searchBuffer, keyLength);
                        keyBuffer[keyLength] = 0;
                        *itemAddress = currentAddress;
                        return length;
                    }
                }

                currentAddress += length;
            }

            *itemAddress = SIZE_MAX;
            return SIZE_MAX;
        }

};
