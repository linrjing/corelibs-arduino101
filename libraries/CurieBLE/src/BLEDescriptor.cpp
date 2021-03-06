/*
 * Copyright (c) 2015 Intel Corporation.  All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.

 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include "BLEDescriptor.h"

#include "internal/ble_client.h"

BLEDescriptor::BLEDescriptor(const char* uuid, const unsigned char value[], unsigned short valueLength) :
    BLEAttribute(uuid, BLETypeDescriptor)
{
    if (valueLength > BLE_MAX_ATTR_DATA_LEN) {
        valueLength = BLE_MAX_ATTR_DATA_LEN;
    }
    _value_length = valueLength;
    _value = (unsigned char*)malloc(_value_length);

    memcpy(_value, value, _value_length);
}

BLEDescriptor::~BLEDescriptor() {
    if (_value) {
        free(_value);
        _value = NULL;
    }
}

BLEDescriptor::BLEDescriptor(const char* uuid, const char* value) :
    BLEDescriptor(uuid, (const uint8_t*)value, strlen(value))
{
}

const unsigned char*
BLEDescriptor::value() const
{
    return _value;
}

unsigned short
BLEDescriptor::valueLength() const
{
    return _value_length;
}

unsigned char
BLEDescriptor::operator[] (int offset) const
{
    return _value[offset];
}

void BLEDescriptor::discover(const bt_gatt_attr_t *attr,
                             bt_gatt_discover_params_t *params)
{
    if (!attr)
    {
        // Discovery complete
        _discoverying = false;
        return;
    }
    
    // Chracteristic Char
    if (params->uuid == this->uuid())
    {
        // Set Discover CCCD parameter
        params->start_handle = attr->handle + 1;
        // Complete the discover
        _discoverying = false;
    }

}


void BLEDescriptor::discover(bt_gatt_discover_params_t *params)
{
    params->type = BT_GATT_DISCOVER_DESCRIPTOR;
    params->uuid = this->uuid();
    // Start discovering
    _discoverying = true;
}


