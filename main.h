/* tests/unit-test.h.  Generated from unit-test.h.in by configure.  */
/*
 * Copyright © 2008-2014 Stéphane Raimbault <stephane.raimbault@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _UNIT_TEST_H_
#define _UNIT_TEST_H_

/* Constants defined by configure.ac */
#define HAVE_INTTYPES_H 1
#define HAVE_STDINT_H 1

#ifdef HAVE_INTTYPES_H
#include <inttypes.h>
#endif
#ifdef HAVE_STDINT_H
# ifndef _MSC_VER
# include <stdint.h>
# else
# include "stdint.h"
# endif
#endif

#include <modbus/modbus.h>

#define SERVER_ID         17
#define INVALID_SERVER_ID 18

const uint16_t UT_BITS_ADDRESS = 0;
const uint16_t UT_BITS_NB = 0;
const uint8_t UT_BITS_TAB[] = {};

const uint16_t UT_INPUT_BITS_ADDRESS = 0;
const uint16_t UT_INPUT_BITS_NB = 0;
const uint8_t UT_INPUT_BITS_TAB[] = {};

const uint16_t UT_REGISTERS_ADDRESS = 0;
const uint16_t UT_REGISTERS_NB = 0x07FF;
const uint16_t UT_REGISTERS_NB_MAX = 0x800;
const uint16_t UT_REGISTERS_TAB[] = {};

const uint16_t UT_INPUT_REGISTERS_ADDRESS = 0;
const uint16_t UT_INPUT_REGISTERS_NB = 0;
const uint16_t UT_INPUT_REGISTERS_TAB[] = {};

#endif /* _UNIT_TEST_H_ */
